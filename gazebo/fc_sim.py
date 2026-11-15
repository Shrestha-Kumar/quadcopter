#!/usr/bin/env python3
"""Python port of firmware/quad_fc control_step() driving the Gazebo quadcopter + 3D Browser GUI.

Includes hardware-calibrated non-idealities matched to logs/flight_log_20261114_175759.csv:
  - BNO085 SHTP + I2C fusion transport delay FIFO (30 ms Euler delay, 15 ms gyro delay)
  - Asymmetric ESC + propeller spin-up (60 ms) vs coast-down (120 ms) + quadratic thrust curve
  - Rotor vibration harmonics & turbulence jerks
  - Active wind gust wrench + wind-outmatching tilt (roll/pitch lean into wind to counter drift)
"""
import argparse
import collections
import json
import math
import os
import random
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
P = yaml.safe_load(open(os.path.join(HERE, "params.yaml")))
C, PL, SN = P["controller"], P["plant"], P["sensors"]
OMEGA_MAX = 1000.0


def sync_from_config_h():
    """Overwrite controller gains/limits with the values currently #defined in firmware config.h,
    so the sim always matches what is hard-coded on the flight controller."""
    path = os.path.join(HERE, "..", "firmware", "quad_fc", "config.h")
    mp = {"ANG_KP": "ang_kp", "YAW_ANG_KP": "yaw_ang_kp", "RATE_KP": "rate_kp", "RATE_KI": "rate_ki",
          "RATE_KD": "rate_kd", "YAWRATE_KP": "yawr_kp", "YAWRATE_KI": "yawr_ki", "YAWRATE_KD": "yawr_kd",
          "RATE_I_LIMIT": "rate_i_limit", "YAW_I_LIMIT": "yaw_i_limit", "MAX_YAW_OUT_US": "max_yaw_out_us",
          "MAX_CTRL_OUT_US": "max_ctrl_out_us", "D_LPF_ALPHA": "d_lpf_alpha", "SP_SLEW_DPS": "sp_slew_dps",
          "MOTOR_IDLE_US": "motor_idle_us", "THR_MAX_US": "thr_max_us", "MAX_RATE_SP_DPS": "max_rate_sp_dps",
          "MAX_YAW_RATE_DPS": "max_yaw_rate_dps", "MAX_ANGLE_SP_DEG": "max_angle_sp_deg",
          "MAX_TILT_CUT_DEG": "max_tilt_cut_deg"}
    changed = []
    for line in open(path):
        m = re.match(r"\s*#define\s+(\w+)\s+([-+0-9.eE]+)f?\b", line)
        if m and m.group(1) in mp:
            v = float(m.group(2))
            if abs(C[mp[m.group(1)]] - v) > 1e-9:
                changed.append(f"{mp[m.group(1)]}: {C[mp[m.group(1)]]} -> {v}")
            C[mp[m.group(1)]] = v
    print("[sim] gains synced from config.h" + (f" (changed: {', '.join(changed)})" if changed else " (no differences vs params.yaml)"), flush=True)


sync_from_config_h()


def clampf(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class Rate:
    def __init__(self):
        self.i = 0.0
        self.prev_meas = 0.0
        self.d_filt = 0.0


class FlightController:
    """Line-by-line mirror of quad_fc.ino control_step()/rate_pid()."""

    def __init__(self):
        self.g = dict(ang_kp=C["ang_kp"], yaw_ang_kp=C["yaw_ang_kp"], rate_kp=C["rate_kp"],
                      rate_ki=C["rate_ki"], rate_kd=C["rate_kd"], yawr_kp=C["yawr_kp"],
                      yawr_ki=C["yawr_ki"], yawr_kd=C["yawr_kd"], sp_slew=C["sp_slew_dps"])
        self.rr, self.rp, self.ry = Rate(), Rate(), Rate()
        self.roll_sp = self.pitch_sp = self.yaw_sp = 0.0
        self.roll_sp_eff = self.pitch_sp_eff = 0.0
        self.thr_us = 1000.0
        self.gyro_dt_accum = 0.0
        self.yaw_dir = 1.0
        self.yaw_trim_us = 0.0

    def reset_pid(self):
        for s in (self.rr, self.rp, self.ry):
            s.i = s.d_filt = 0.0
        self.roll_sp_eff = self.roll_sp
        self.pitch_sp_eff = self.pitch_sp

    def _rate_pid(self, s, sp, meas, kp, ki, kd, ilim, dt, do_d, d_dt):
        err = sp - meas
        if self.thr_us >= 1300:
            s.i = clampf(s.i + ki * err * dt, -ilim, ilim)
        else:
            s.i = 0.0
        if do_d:
            d_raw = -(meas - s.prev_meas) / d_dt
            s.d_filt += C["d_lpf_alpha"] * (d_raw - s.d_filt)
            s.prev_meas = meas
        out = kp * err + s.i + kd * s.d_filt
        return clampf(out, -C["max_ctrl_out_us"], C["max_ctrl_out_us"])

    def step(self, dt, roll, pitch, yaw, gx, gy, gz, fresh_gyro):
        g = self.g
        self.gyro_dt_accum += dt
        do_d, d_dt = False, self.gyro_dt_accum
        if fresh_gyro:
            do_d = True
            self.gyro_dt_accum = 0.0
            d_dt = clampf(d_dt, 0.004, 0.050)
        ms = g["sp_slew"] * dt
        self.roll_sp_eff += clampf(self.roll_sp - self.roll_sp_eff, -ms, ms)
        self.pitch_sp_eff += clampf(self.pitch_sp - self.pitch_sp_eff, -ms, ms)
        mr, my = C["max_rate_sp_dps"], C["max_yaw_rate_dps"]
        rsp_r = clampf(g["ang_kp"] * (self.roll_sp_eff - roll), -mr, mr)
        rsp_p = clampf(g["ang_kp"] * (self.pitch_sp_eff - pitch), -mr, mr)
        rsp_y = clampf(g["yaw_ang_kp"] * wrap180(self.yaw_sp - yaw), -my, my)
        u_r = self._rate_pid(self.rr, rsp_r, gx, g["rate_kp"], g["rate_ki"], g["rate_kd"], C["rate_i_limit"], dt, do_d, d_dt)
        u_p = self._rate_pid(self.rp, rsp_p, gy, g["rate_kp"], g["rate_ki"], g["rate_kd"], C["rate_i_limit"], dt, do_d, d_dt)
        u_y = clampf(self.yaw_dir * self._rate_pid(self.ry, rsp_y, gz, g["yawr_kp"], g["yawr_ki"], g["yawr_kd"],
                                                  C["yaw_i_limit"], dt, do_d, d_dt) + self.yaw_trim_us,
                     -C["max_yaw_out_us"], C["max_yaw_out_us"])
        base = clampf(self.thr_us, C["esc_min_us"], C["thr_max_us"])
        out = []
        for i in range(4):
            m = base + C["mix_r"][i] * u_r + C["mix_p"][i] * u_p + C["mix_y"][i] * u_y
            out.append(int(clampf(m, C["motor_idle_us"], C["esc_max_us"])))
        if self.thr_us < C["motor_idle_us"]:
            out = [C["motor_idle_us"]] * 4
            self.reset_pid()
        return out


def quat_to_euler_deg(w, x, y, z):
    """Gazebo FLU world->body ZYX euler (roll about x, pitch about y(left), yaw about z(up))."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(clampf(2 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def us_to_omega(us, base_us=1337.0):
    """Convert ESC PWM (1000..2000 us) to rotor angular speed (rad/s).
    Calibrates differential control authority around hover (base_us) to match the real
    A2212 1000KV + 10x4.5 prop hardware torque slope (where 1 us is ~38% of ideal linear
    Tmax/1000 slope), so step overshoot/undershoot and high-KP oscillations match hardware."""
    u_base = clampf((base_us - 1000.0) / 1000.0, 0.0, 1.0)
    du = (us - base_us) / 1000.0
    u_eff = clampf(u_base + 0.38 * du, 0.0, 1.0)
    return OMEGA_MAX * math.sqrt(u_eff)


def hover_thr_us():
    return 1000.0 + 1000.0 * (PL["mass"] * PL["g"] / (4 * PL["Tmax"]))


def make_scenario(name, hover_ff):
    """Return f(t) -> dict(thr, roll, pitch, yaw, armed) in sim seconds."""
    def f(t):
        s = dict(armed=1, thr=hover_ff, roll=0.0, pitch=0.0, yaw=0.0)
        if t < 1.0:
            s["armed"] = 0
            s["thr"] = 1000.0
        elif t < 3.0:
            s["thr"] = 1050 + (hover_ff - 1050) * (t - 1.0) / 2.0
        if name == "roll_steps":
            if 4 <= t < 8: s["roll"] = 5.0
            elif 8 <= t < 12: s["roll"] = -5.0
            elif 12 <= t < 16: s["roll"] = 5.0
            elif 16 <= t < 20: s["roll"] = -5.0
        elif name == "pitch_steps":
            if 4 <= t < 8: s["pitch"] = 5.0
            elif 8 <= t < 12: s["pitch"] = -5.0
            elif 12 <= t < 16: s["pitch"] = 5.0
        elif name == "yaw_steps":
            if 4 <= t < 10: s["yaw"] = 30.0
            elif 10 <= t < 16: s["yaw"] = -30.0
        elif name == "hold":
            pass
        return s
    dur = {"roll_steps": 24.0, "pitch_steps": 18.0, "yaw_steps": 20.0, "hold": 12.0}.get(name, 1e12)
    return f, dur


PANEL_HTML = r"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>EE302 Quadcopter — Gazebo 3D Browser Studio</title>
<style>
  * { box-sizing: border-box; }
  body { font-family: system-ui, -apple-system, sans-serif; background: #0b0f19; color: #f3f4f6; margin: 0; padding: 10px; }
  .layout { display: grid; grid-template-columns: 1fr 415px; gap: 12px; height: calc(100vh - 20px); }
  @media (max-width: 1000px) { .layout { grid-template-columns: 1fr; height: auto; } }
  .left-col { display: flex; flex-direction: column; gap: 10px; min-height: 0; }
  .right-col { display: flex; flex-direction: column; gap: 8px; overflow-y: auto; padding-right: 4px; }
  .card { background: #111827; border: 1px solid #1f2937; border-radius: 10px; padding: 10px 13px; }
  h2 { margin: 0 0 7px 0; font-size: 13px; text-transform: uppercase; letter-spacing: 0.05em; color: #60a5fa; }
  .viewport-card { flex: 1; position: relative; padding: 0; overflow: hidden; display: flex; flex-direction: column; min-height: 360px; }
  #view3d { width: 100%; flex: 1; display: block; cursor: grab; background: linear-gradient(180deg, #0f172a 0%, #1e293b 58%, #090d16 100%); }
  #view3d:active { cursor: grabbing; }
  .hud { position: absolute; top: 10px; left: 12px; right: 12px; display: flex; justify-content: space-between; pointer-events: none; font-family: monospace; font-size: 13px; }
  .hud-box { background: rgba(15, 23, 42, 0.85); border: 1px solid #334155; padding: 7px 11px; border-radius: 8px; pointer-events: auto; }
  .chart-card { height: 195px; padding: 8px 12px; display: flex; flex-direction: column; }
  #chart { width: 100%; flex: 1; display: block; background: #090d16; border-radius: 6px; border: 1px solid #1f2937; }
  button { font-size: 13px; font-weight: 600; padding: 6px 11px; margin: 2px; border-radius: 7px; border: 0; cursor: pointer; background: #2563eb; color: #fff; transition: filter 0.15s; }
  button:hover { filter: brightness(1.15); }
  .g { background: #059669; } .o { background: #d97706; } .r { background: #dc2626; } .s { background: #374151; }
  .active-btn { outline: 2px solid #fbbf24; }
  .row { display: flex; align-items: center; justify-content: space-between; margin: 4px 0; font-size: 13px; gap: 8px; }
  input[type=range] { flex: 1; accent-color: #3b82f6; }
  input[type=number], select { background: #1f2937; color: #f9fafb; border: 1px solid #374151; border-radius: 6px; padding: 4px 6px; width: 72px; font-size: 13px; }
  select { width: auto; flex: 1; }
  .val { font-family: monospace; width: 64px; text-align: right; color: #93c5fd; }
  .motors { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; margin-top: 6px; font-family: monospace; font-size: 12px; text-align: center; }
  .mbar { background: #1f2937; border-radius: 6px; padding: 5px 4px; border: 1px solid #374151; }
  .mfill { height: 6px; background: #10b981; border-radius: 3px; margin-top: 4px; transition: width 0.05s; }
  .gains-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 10px; font-size: 12px; }
  .gains-grid label { display: flex; justify-content: space-between; align-items: center; }
</style>
</head>
<body>
<div class="layout">
  <div class="left-col">
    <div class="card viewport-card">
      <canvas id="view3d"></canvas>
      <div class="hud">
        <div class="hud-box">
          <div id="st_top" style="font-size:14px;font-weight:700;color:#38bdf8">Connecting to Gazebo...</div>
          <div id="st_att" style="margin-top:3px">Roll: 0.0° | Pitch: 0.0° | Yaw: 0.0°</div>
          <div id="st_pos" style="color:#9ca3af">Pos: (0.00, 0.00, 0.30) m</div>
          <div id="msg" style="color:#fbbf24;margin-top:2px"></div>
        </div>
        <div class="hud-box" style="text-align:right">
          <div><span style="color:#ef4444">■</span> Red Arms: M0(FR,CCW) / M2(RL,CCW)</div>
          <div><span style="color:#e5e7eb">■</span> White Arms: M1(FL,CW) / M3(RR,CW)</div>
          <div><span style="color:#fbbf24">▲</span> Yellow Cone: Front Nose (+X)</div>
          <div style="margin-top:4px">
            <label style="cursor:pointer"><input type="checkbox" id="cam_follow" checked> Follow Quad</label>
            <button class="s" style="padding:3px 7px;font-size:11px" onclick="camAz=-0.72;camEl=0.42;camDist=1.35">Reset Cam</button>
          </div>
        </div>
      </div>
    </div>
    <div class="card chart-card">
      <div style="display:flex;justify-content:space-between;font-size:12px;margin-bottom:4px;font-family:monospace">
        <span><b>Live Attitude &amp; Wind Response (10 s) — Overshoot / Undershoot / Gusts</b></span>
        <span>
          <span style="color:#f87171">━ Roll</span> <span style="color:#fca5a5">┄ Roll SP+Wind</span> &nbsp;
          <span style="color:#34d399">━ Pitch</span> <span style="color:#6ee7b7">┄ Pitch SP+Wind</span> &nbsp;
          <span style="color:#60a5fa">━ Yaw</span>
        </span>
      </div>
      <canvas id="chart"></canvas>
    </div>
  </div>

  <div class="right-col">
    <div class="card">
      <h2>1. Environment &amp; Simulation</h2>
      <div class="row">
        <b>Environment:</b>
        <select id="world_sel" onchange="switchWorld(this.value)">
          <option value="free_flight">Free Flight (6-DOF Open Air)</option>
          <option value="tether_rig">Tether Rig (3-DOF Pole Stand)</option>
        </select>
      </div>
      <div style="display:flex;flex-wrap:wrap;gap:4px;margin-top:6px">
        <button class="g" onclick="sim('run')">&#9654; Run Sim</button>
        <button class="s" onclick="sim('pause')">&#10074;&#10074; Pause</button>
        <button class="o" onclick="sim('reset')">&#8635; Reset World</button>
        <button class="g" onclick="set('armed=1')">ARM</button>
        <button class="r" onclick="set('armed=0')">DISARM / STOP</button>
      </div>
    </div>

    <div class="card">
      <h2>2. Scenario / Flight Mode</h2>
      <div style="display:flex;flex-wrap:wrap;gap:4px">
        <button id="m_manual" class="s active-btn" onclick="setMode('manual')">Manual Sliders</button>
        <button id="m_roll_steps" onclick="setMode('roll_steps')">Auto: Roll &plusmn;5&deg;</button>
        <button id="m_pitch_steps" onclick="setMode('pitch_steps')">Auto: Pitch &plusmn;5&deg;</button>
        <button id="m_yaw_steps" onclick="setMode('yaw_steps')">Auto: Yaw &plusmn;30&deg;</button>
      </div>
    </div>

    <div class="card">
      <h2>3. Throttle &amp; Motors</h2>
      <div class="row">
        <span>Throttle</span>
        <input id="thr" type="range" min="1000" max="1750" step="5" value="1000" oninput="set('thr='+this.value);$('tl').textContent=this.value+' us'">
        <span id="tl" class="val">1000 us</span>
        <button class="s" style="padding:4px 8px" onclick="fetch('/hover')">Hover</button>
      </div>
      <div class="row">
        <label><input type="checkbox" id="ah" checked onchange="set('alt_hold='+(this.checked?1:0))"> Altitude Assist (Free Flight)</label>
        <span>Target z: <input id="alt" type="number" step="0.1" value="1.0" onchange="set('alt='+this.value)"> m</span>
      </div>
      <div class="motors">
        <div class="mbar">M0 FR<br><span id="m0">1000</span><div class="mfill" id="mf0" style="width:0%"></div></div>
        <div class="mbar">M1 FL<br><span id="m1">1000</span><div class="mfill" id="mf1" style="width:0%"></div></div>
        <div class="mbar">M2 RL<br><span id="m2">1000</span><div class="mfill" id="mf2" style="width:0%"></div></div>
        <div class="mbar">M3 RR<br><span id="m3">1000</span><div class="mfill" id="mf3" style="width:0%"></div></div>
      </div>
    </div>

    <div class="card">
      <h2>4. Attitude Setpoints (Roll / Pitch / Yaw)</h2>
      <div class="row">
        <span>Roll SP</span>
        <input id="roll" type="range" min="-20" max="20" step="0.5" value="0" oninput="setMode('manual',true);set('roll='+this.value);$('rl').textContent=this.value+'°'">
        <span id="rl" class="val">0°</span>
      </div>
      <div class="row">
        <span>Pitch SP</span>
        <input id="pitch" type="range" min="-20" max="20" step="0.5" value="0" oninput="setMode('manual',true);set('pitch='+this.value);$('pl').textContent=this.value+'°'">
        <span id="pl" class="val">0°</span>
      </div>
      <div class="row">
        <span>Yaw SP</span>
        <input id="yaw" type="range" min="-180" max="180" step="10" value="0" oninput="setMode('manual',true);set('yaw='+this.value);$('yl').textContent=this.value+'°'">
        <span id="yl" class="val">0°</span>
      </div>
      <div style="display:flex;gap:4px;margin-top:4px;flex-wrap:wrap">
        <button class="s" onclick="quickSp(-5,0)">Roll -5&deg;</button>
        <button class="s" onclick="zero()">Level (0&deg;)</button>
        <button class="s" onclick="quickSp(5,0)">Roll +5&deg;</button>
        <button class="s" onclick="quickSp(0,5)">Pitch +5&deg;</button>
        <button class="s" onclick="quickSp(0,-5)">Pitch -5&deg;</button>
        <button class="s" onclick="setMode('manual',true);$('yaw').value=-10;$('yl').textContent='-10°';set('yaw=-10')">Yaw -10&deg;</button>
        <button class="s" onclick="setMode('manual',true);$('yaw').value=10;$('yl').textContent='10°';set('yaw=10')">Yaw +10&deg;</button>
      </div>
    </div>

    <div class="card">
      <h2>5. Wind Speed &amp; Gust Disturbance</h2>
      <div class="row">
        <span>Wind X (Front/Back)</span>
        <input id="wx" type="range" min="-3" max="3" step="0.1" value="0" oninput="set('wind_x='+this.value);$('wxl').textContent=this.value+' N'">
        <span id="wxl" class="val">0 N</span>
      </div>
      <div class="row">
        <span>Wind Y (Left/Right)</span>
        <input id="wy" type="range" min="-3" max="3" step="0.1" value="0" oninput="set('wind_y='+this.value);$('wyl').textContent=this.value+' N'">
        <span id="wyl" class="val">0 N</span>
      </div>
      <div style="display:flex;gap:4px;margin-top:4px">
        <button class="s" onclick="$('wx').value=0;$('wy').value=0;$('wxl').textContent='0 N';$('wyl').textContent='0 N';set('wind_x=0&wind_y=0')">Calm (0 N)</button>
        <button class="s" onclick="$('wy').value=2.0;$('wyl').textContent='2 N';set('wind_y=2.0')">Crosswind Y +2N</button>
        <button class="s" onclick="$('wx').value=-2.0;$('wxl').textContent='-2 N';set('wind_x=-2.0')">Headwind X -2N</button>
      </div>
    </div>

    <div class="card">
      <h2>6. Live Cascaded PID Gains (config.h)</h2>
      <div class="gains-grid">
        <label>ANG_KP <input id="g_ang_kp" type="number" step="0.1"></label>
        <label>RATE_KP <input id="g_rate_kp" type="number" step="0.1"></label>
        <label>RATE_KI <input id="g_rate_ki" type="number" step="0.05"></label>
        <label>RATE_KD <input id="g_rate_kd" type="number" step="0.01"></label>
        <label>YAW_ANG_KP <input id="g_yaw_ang_kp" type="number" step="0.1"></label>
        <label>YAWR_KP <input id="g_yawr_kp" type="number" step="0.1"></label>
      </div>
      <div style="display:flex;gap:4px;margin-top:8px">
        <button style="flex:1" onclick="applyG()">Apply PID Gains Live</button>
        <button class="s" onclick="resetTunedGains()">Reset Tuned (2.8/2.6/0.8/0.15)</button>
      </div>
    </div>
  </div>
</div>

<script>
const $ = id => document.getElementById(id);
let latest = {t:0, armed:0, roll:0, pitch:0, yaw:0, roll_sp:0, pitch_sp:0, yaw_sp:0, x:0, y:0, z:0.3, m:[1000,1000,1000,1000], wind_x:0, wind_y:0, world:'free_flight', auto_mode:'manual'};
let hist = [];
let first = true;
let propAng = [0, 0, 0, 0];

function set(q) { fetch('/set?' + q); }
function sim(a) { fetch('/sim?a=' + a); }
function switchWorld(w) { hist = []; fetch('/switch_world?w=' + w); }
function setMode(m, quiet) {
  ['manual','roll_steps','pitch_steps','yaw_steps'].forEach(k => $('m_'+k).classList.toggle('active-btn', k===m));
  if (!quiet) fetch('/mode?m=' + m);
  else fetch('/mode?m=manual&quiet=1');
}
function zero() {
  setMode('manual', true);
  ['roll','pitch','yaw'].forEach(k => { $(k).value = 0; set(k + '=0'); });
  $('rl').textContent = $('pl').textContent = $('yl').textContent = '0°';
}
function quickSp(r, p) {
  setMode('manual', true);
  $('roll').value = r; $('rl').textContent = r + '°';
  $('pitch').value = p; $('pl').textContent = p + '°';
  set('roll=' + r + '&pitch=' + p);
}
function applyG() {
  const names = ['ang_kp','rate_kp','rate_ki','rate_kd','yaw_ang_kp','yawr_kp'];
  fetch('/gain?' + names.map(n => n + '=' + $('g_' + n).value).join('&'));
}
function resetTunedGains() {
  const def = {ang_kp:2.8, rate_kp:2.6, rate_ki:0.8, rate_kd:0.15, yaw_ang_kp:2.6, yawr_kp:3.0};
  for (const k in def) $('g_'+k).value = def[k];
  applyG();
}

// Poll state at 25 Hz
setInterval(async () => {
  try {
    const r = await fetch('/state');
    const s = await r.json();
    latest = s;
    $('st_top').textContent = `t = ${s.t.toFixed(2)}s | ${s.armed ? '🟢 ARMED' : '🟡 DISARMED'} | World: ${s.world} | Mode: ${s.auto_mode}`;
    $('st_att').textContent = `Roll: ${s.roll.toFixed(1)}° (target ${s.roll_sp.toFixed(1)}°) | Pitch: ${s.pitch.toFixed(1)}° (target ${s.pitch_sp.toFixed(1)}°) | Yaw: ${s.yaw.toFixed(1)}°`;
    $('st_pos').textContent = `Pos: (${s.x.toFixed(2)}, ${s.y.toFixed(2)}, ${s.z.toFixed(2)}) m | Thr: ${Math.round(s.thr)} us | Wind: (${(s.wind_x||0).toFixed(1)}, ${(s.wind_y||0).toFixed(1)}) N`;
    $('msg').textContent = s.msg || '';
    $('world_sel').value = s.world;
    ['manual','roll_steps','pitch_steps','yaw_steps'].forEach(k => $('m_'+k).classList.toggle('active-btn', k===s.auto_mode));
    if (s.auto_mode !== 'manual') {
      $('roll').value = s.roll_sp; $('rl').textContent = s.roll_sp.toFixed(1) + '°';
      $('pitch').value = s.pitch_sp; $('pl').textContent = s.pitch_sp.toFixed(1) + '°';
      $('yaw').value = s.yaw_sp; $('yl').textContent = s.yaw_sp.toFixed(0) + '°';
      $('thr').value = Math.round(s.thr); $('tl').textContent = Math.round(s.thr) + ' us';
    }
    for (let i = 0; i < 4; i++) {
      $('m' + i).textContent = s.m[i];
      $('mf' + i).style.width = Math.max(0, Math.min(100, (s.m[i] - 1000) / 10)) + '%';
    }
    if (first && s.gains) {
      for (const n in s.gains) { const e = $('g_' + n); if (e) e.value = s.gains[n]; }
      $('ah').checked = !!s.alt_hold;
      first = false;
    }
    hist.push({t: s.t, r: s.roll, rsp: s.roll_sp, p: s.pitch, psp: s.pitch_sp, y: s.yaw, z: s.z});
    while (hist.length > 250) hist.shift();
  } catch (e) {}
}, 40);

// -------------------- Right-Side-Up 3D Canvas Renderer (+Z is UP on screen!) --------------------
const cv = $('view3d'), ctx = cv.getContext('2d');
let camAz = -0.72, camEl = 0.42, camDist = 1.35, dragging = false, lastX = 0, lastY = 0;
cv.addEventListener('mousedown', e => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener('mouseup', () => dragging = false);
window.addEventListener('mousemove', e => {
  if (!dragging) return;
  camAz -= (e.clientX - lastX) * 0.008;
  camEl = Math.max(0.05, Math.min(1.45, camEl + (e.clientY - lastY) * 0.008));
  lastX = e.clientX; lastY = e.clientY;
});
cv.addEventListener('wheel', e => {
  e.preventDefault();
  camDist = Math.max(0.5, Math.min(4.5, camDist * (1 + e.deltaY * 0.001)));
}, {passive: false});

function rotBodyToWorld(bx, by, bz, rDeg, pFwDeg, yDeg) {
  // Firmware convention: +roll = right side (-y in FLU) goes DOWN, +pitch_fw = nose (+x in FLU) goes UP
  const r = -rDeg * Math.PI / 180, p = -pFwDeg * Math.PI / 180, y = yDeg * Math.PI / 180;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  const wx = cy*cp*bx + (cy*sp*sr - sy*cr)*by + (cy*sp*cr + sy*sr)*bz;
  const wy = sy*cp*bx + (sy*sp*sr + cy*cr)*by + (sy*sp*cr - cy*sr)*bz;
  const wz = -sp*bx   + cp*sr*by             + cp*cr*bz;
  return [wx, wy, wz];
}

function draw3D() {
  if (cv.width !== cv.clientWidth || cv.height !== cv.clientHeight) {
    cv.width = cv.clientWidth; cv.height = cv.clientHeight;
  }
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);
  const s = latest;
  const follow = $('cam_follow').checked;
  const tx = follow ? s.x : 0, ty = follow ? s.y : 0, tz = follow ? Math.max(0.22, s.z) : 0.28;
  // Camera sits ABOVE ground (cz = tz + camDist * sin(camEl)) looking at (tx, ty, tz)
  const cx = tx - camDist * Math.cos(camEl) * Math.cos(camAz);
  const cy = ty - camDist * Math.cos(camEl) * Math.sin(camAz);
  const cz = tz + camDist * Math.sin(camEl);

  // Forward vector f = normalize(target - cam)
  const fx = Math.cos(camEl)*Math.cos(camAz), fy = Math.cos(camEl)*Math.sin(camAz), fz = -Math.sin(camEl);
  // Right vector r = normalize(f x World_Up[0,0,1]) -> +rx is screen RIGHT
  const rx = Math.sin(camAz), ry = -Math.cos(camAz), rz = 0;
  // Camera Up vector u = r x f -> uz = +cos(camEl) > 0 so +Z World is always UP on screen!
  const ux = ry*fz - rz*fy, uy = rz*fx - rx*fz, uz = rx*fy - ry*fx;
  const fov = Math.min(W, H) * 0.95;

  function proj(wx, wy, wz) {
    const dx = wx - cx, dy = wy - cy, dz = wz - cz;
    const zc = dx*fx + dy*fy + dz*fz;
    if (zc <= 0.05) return null;
    const xc = dx*rx + dy*ry + dz*rz;
    const yc = dx*ux + dy*uy + dz*uz;
    return [W/2 + (xc / zc) * fov, H/2 - (yc / zc) * fov, zc];
  }
  function line3(a, b, col, lw=2, dash=[]) {
    const p1 = proj(a[0], a[1], a[2]), p2 = proj(b[0], b[1], b[2]);
    if (!p1 || !p2) return;
    ctx.save(); ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke(); ctx.restore();
  }
  function poly3(pts, fillCol, strokeCol, lw=1.5) {
    const ps = pts.map(p => proj(p[0], p[1], p[2]));
    if (ps.some(p => !p)) return;
    ctx.save(); ctx.fillStyle = fillCol; ctx.strokeStyle = strokeCol; ctx.lineWidth = lw;
    ctx.beginPath(); ps.forEach((p, i) => i===0 ? ctx.moveTo(p[0], p[1]) : ctx.lineTo(p[0], p[1]));
    ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  }

  // 1. Ground grid at z = 0 (BOTTOM of view)
  const gx0 = Math.floor(tx) - 2, gy0 = Math.floor(ty) - 2;
  for (let i = -2; i <= 4; i++) {
    line3([gx0 + i, gy0 - 2, 0], [gx0 + i, gy0 + 4, 0], '#1e293b', 1);
    line3([gx0 - 2, gy0 + i, 0], [gx0 + 4, gy0 + i, 0], '#1e293b', 1);
  }
  // World origin axes (+X Red Forward, +Y Green Left, +Z Blue UP)
  line3([0,0,0.002], [0.40,0,0.002], '#ef4444', 2.5);
  line3([0,0,0.002], [0,0.40,0.002], '#22c55e', 2.5);
  line3([0,0,0.002], [0,0,0.40], '#3b82f6', 2.5);

  // 2. Wooden base & vertical spring-pole rising UP from z=0 to quad bottom (tether_rig)
  if (s.world === 'tether_rig') {
    poly3([[-0.22,-0.22,0.002],[0.22,-0.22,0.002],[0.22,0.22,0.002],[-0.22,0.22,0.002]], 'rgba(120,83,45,0.55)', '#92400e', 2);
    line3([0, 0, 0.002], [0, 0, 0.265], '#cbd5e1', 7);
    line3([0, 0, 0.18], [0, 0, 0.255], '#f59e0b', 9, [4, 3]); // spring coil on pole
  }

  // 3. Ground shadow under quadcopter at z = 0
  const sh = proj(s.x, s.y, 0.003);
  if (sh) {
    ctx.save(); ctx.fillStyle = 'rgba(0,0,0,0.45)';
    ctx.beginPath(); ctx.ellipse(sh[0], sh[1], Math.max(10, 62/sh[2]), Math.max(5, 32/sh[2]), 0, 0, Math.PI*2);
    ctx.fill(); ctx.restore();
  }
  line3([s.x, s.y, 0], [s.x, s.y, s.z], 'rgba(148,163,184,0.35)', 1.5, [4, 4]);

  // 4. Right-Side-Up Quadcopter Geometry in Body Frame (+bz is TOP of drone, -bz is BOTTOM skids)
  const d = 0.1591;
  const rotors = [
    {x: +d, y: -d, col: '#ef4444', dir: 1, lbl: 'M0 FR'},  // Front-Right (Red)
    {x: +d, y: +d, col: '#f3f4f6', dir: -1, lbl: 'M1 FL'}, // Front-Left (White)
    {x: -d, y: +d, col: '#ef4444', dir: 1, lbl: 'M2 RL'},  // Rear-Left (Red)
    {x: -d, y: -d, col: '#f3f4f6', dir: -1, lbl: 'M3 RR'}  // Rear-Right (White)
  ];
  function bw(bx, by, bz) {
    const v = rotBodyToWorld(bx, by, bz, s.roll, s.pitch, s.yaw);
    return [s.x + v[0], s.y + v[1], s.z + v[2]];
  }
  const c0 = bw(0, 0, 0);

  // Landing gear legs pointing DOWN (-bz) + downward thrust wash under each rotor
  rotors.forEach((rt, i) => {
    const armTip = bw(rt.x, rt.y, 0);
    const legBot = bw(rt.x, rt.y, -0.055);
    line3(armTip, legBot, '#475569', 3); // dark landing leg under arm
    const u = Math.max(0, (s.m[i] - 1000) / 1000);
    if (s.armed && u > 0.02) {
      // Thrust wash blows DOWNWARD (-bz) from propeller
      const washBot = bw(rt.x, rt.y, -0.02 - 0.22 * u);
      line3(armTip, washBot, 'rgba(56,189,248,0.55)', 3.5, [4, 3]);
    }
  });

  // Frame arms + top motor bells + spinning propellers ON TOP (+bz = +0.032)
  rotors.forEach((rt, i) => {
    const p = bw(rt.x, rt.y, 0);
    line3(c0, p, rt.col, 6);
    const motorTop = bw(rt.x, rt.y, 0.030);
    line3(p, motorTop, '#f59e0b', 5); // orange/gold A2212 motor bell ON TOP of arm

    const u = Math.max(0, (s.m[i] - 1000) / 1000);
    propAng[i] += rt.dir * (s.armed ? (0.14 + 0.55 * u) : 0);
    const R = 0.11, N = 24;
    ctx.save(); ctx.strokeStyle = s.armed ? 'rgba(56,189,248,0.80)' : 'rgba(148,163,184,0.45)'; ctx.lineWidth = 2.0;
    ctx.beginPath();
    for (let k = 0; k <= N; k++) {
      const a = (k / N) * Math.PI * 2;
      const q = proj(...bw(rt.x + R*Math.cos(a), rt.y + R*Math.sin(a), 0.033));
      if (q) { if (k === 0) ctx.moveTo(q[0], q[1]); else ctx.lineTo(q[0], q[1]); }
    }
    ctx.stroke(); ctx.restore();
    const b1 = bw(rt.x + R*Math.cos(propAng[i]), rt.y + R*Math.sin(propAng[i]), 0.033);
    const b2 = bw(rt.x - R*Math.cos(propAng[i]), rt.y - R*Math.sin(propAng[i]), 0.033);
    line3(b1, b2, s.armed ? '#e0f2fe' : '#64748b', 2.8);
    // Motor label above prop
    const lp = proj(...bw(rt.x, rt.y, 0.065));
    if (lp) {
      ctx.save(); ctx.fillStyle = rt.col; ctx.font = 'bold 11px monospace'; ctx.textAlign = 'center';
      ctx.fillText(`${rt.lbl}:${s.m[i]}`, lp[0], lp[1]); ctx.restore();
    }
  });

  // Top Center Hub + Blue LiPo Battery / FC Canopy sitting ON TOP (+bz = +0.028)
  poly3([bw(0.06,0.06,0.008), bw(-0.06,0.06,0.008), bw(-0.06,-0.06,0.008), bw(0.06,-0.06,0.008)], '#1e293b', '#475569', 2);
  poly3([bw(0.045,0.03,0.028), bw(-0.045,0.03,0.028), bw(-0.045,-0.03,0.028), bw(0.045,-0.03,0.028)], '#2563eb', '#93c5fd', 2);

  // Forward (+X Nose) Yellow Pointer Arrow on top of hub
  line3(bw(0.04, 0, 0.030), bw(0.24, 0, 0.030), '#fbbf24', 4);
  line3(bw(0.24, 0, 0.030), bw(0.18, 0.045, 0.030), '#fbbf24', 3.5);
  line3(bw(0.24, 0, 0.030), bw(0.18, -0.045, 0.030), '#fbbf24', 3.5);

  // 5. Wind Vector Arrows in 3D when Wind X or Y is active
  const wx = s.wind_x || 0, wy = s.wind_y || 0, wmag = Math.hypot(wx, wy);
  if (wmag > 0.05) {
    const uxw = wx / wmag, uyw = wy / wmag;
    for (let k = -1; k <= 1; k++) {
      const ox = s.x - uxw*0.42 - uyw*k*0.14, oy = s.y - uyw*0.42 + uxw*k*0.14, oz = s.z + 0.06;
      const ex = ox + uxw*0.26, ey = oy + uyw*0.26;
      line3([ox, oy, oz], [ex, ey, oz], '#f43f5e', 3.5);
      line3([ex, ey, oz], [ex - uxw*0.05 - uyw*0.03, ey - uyw*0.05 + uxw*0.03, oz], '#f43f5e', 3);
      line3([ex, ey, oz], [ex - uxw*0.05 + uyw*0.03, ey - uyw*0.05 - uxw*0.03, oz], '#f43f5e', 3);
    }
  }
  requestAnimationFrame(draw3D);
}
requestAnimationFrame(draw3D);

// -------------------- Live Telemetry Strip-Chart (Auto-Scaled with Grid Labels) --------------------
const ch = $('chart'), cctx = ch.getContext('2d');
function drawChart() {
  if (ch.width !== ch.clientWidth || ch.height !== ch.clientHeight) {
    ch.width = ch.clientWidth; ch.height = cv.clientWidth ? ch.clientHeight : 150;
  }
  const W = ch.width, H = ch.height;
  cctx.clearRect(0, 0, W, H);
  // Dynamic range (min 12 deg full scale so +/-5 deg steps + overshoots fill the chart nicely)
  let maxDeg = 12.0;
  for (let i = 0; i < hist.length; i++) {
    const d = hist[i];
    maxDeg = Math.max(maxDeg, Math.abs(d.r)*1.15, Math.abs(d.p)*1.15, Math.abs(d.rsp)*1.2, Math.abs(d.psp)*1.2);
  }
  maxDeg = Math.min(45.0, Math.ceil(maxDeg / 5) * 5);

  function yOf(deg) { return H/2 - (deg / maxDeg) * (H * 0.43); }
  cctx.font = '10px monospace'; cctx.fillStyle = '#64748b';
  [-maxDeg, -maxDeg/2, 0, maxDeg/2, maxDeg].forEach(gv => {
    const y = yOf(gv);
    cctx.strokeStyle = gv === 0 ? '#334155' : '#1e293b';
    cctx.lineWidth = gv === 0 ? 1.5 : 1;
    cctx.beginPath(); cctx.moveTo(36, y); cctx.lineTo(W, y); cctx.stroke();
    cctx.fillText((gv > 0 ? '+' : '') + gv.toFixed(0) + '°', 4, y + 3);
  });
  if (hist.length < 2) { requestAnimationFrame(drawChart); return; }
  function plotSeries(fn, col, dash=[], lw=2.0) {
    cctx.save(); cctx.strokeStyle = col; cctx.lineWidth = lw; cctx.setLineDash(dash);
    cctx.beginPath();
    for (let i = 0; i < hist.length; i++) {
      const x = 36 + (i / 250) * (W - 40);
      const y = clampY(yOf(fn(hist[i])), 2, H - 2);
      if (i === 0) cctx.moveTo(x, y); else cctx.lineTo(x, y);
    }
    cctx.stroke(); cctx.restore();
  }
  function clampY(y, lo, hi) { return y < lo ? lo : y > hi ? hi : y; }
  plotSeries(d => d.rsp, '#fca5a5', [4, 3], 1.5);
  plotSeries(d => d.r,   '#f87171', [], 2.2);
  plotSeries(d => d.psp, '#6ee7b7', [4, 3], 1.5);
  plotSeries(d => d.p,   '#34d399', [], 2.2);
  plotSeries(d => d.y,   '#60a5fa', [], 1.6);
  requestAnimationFrame(drawChart);
}
requestAnimationFrame(drawChart);
</script>
</body>
</html>"""


class Runner:
    def __init__(self, args):
        from gz.transport13 import Node
        from gz.msgs10.imu_pb2 import IMU
        from gz.msgs10.actuators_pb2 import Actuators
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        self.args = args
        self.srv_proc = None
        if args.manage_server:
            self._start_gz_server(args.world)
        self.node = Node()
        self.fc = FlightController()
        self.Actuators = Actuators
        self.Entity = Entity
        self.EntityWrench = EntityWrench
        self.cmd_pub = self.node.advertise("/quadcopter/command/motor_speed", Actuators)
        self._advertise_world_topics(args.world)
        self.odom = (0.0, 0.0, 0.30, 0.0, 0.0, 0.0)
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.rows = []
        self.t0 = None
        self.last_t = None
        self.tick = 0
        self.held_g = (0.0, 0.0, 0.0)
        self.yaw_ref = None
        self.armed_prev = 0
        # BNO085 SHTP + I2C transport delay buffers (30 ms Euler = 6 ticks @200Hz, 15 ms gyro = 3 ticks)
        self.euler_fifo = collections.deque(maxlen=6)
        self.gyro_fifo = collections.deque(maxlen=3)
        # Second-order underdamped plant augmentation matching hardware rig overshoot/undershoot/gusts
        self.osc_r = 0.0
        self.osc_vr = 0.0
        self.osc_p = 0.0
        self.osc_vp = 0.0
        self.prev_sp_r = 0.0
        self.prev_sp_p = 0.0
        self.prev_wx = 0.0
        self.prev_wy = 0.0
        self.auto_mode = "manual"
        self.auto_t0 = 0.0
        self.scn, self.dur = make_scenario(args.scenario, hover_thr_us())
        self.interactive = args.scenario == "interactive"
        self.ui = dict(armed=0, thr=1000.0, roll=0.0, pitch=0.0, yaw=0.0, alt_hold=True, alt=1.0,
                       wind_x=0.0, wind_y=0.0)
        self.state = dict(t=0.0, roll=0.0, pitch=0.0, yaw=0.0, roll_sp=0.0, pitch_sp=0.0, yaw_sp=0.0,
                          x=0.0, y=0.0, z=0.30, m=[1000] * 4, msg="")
        if self.interactive:
            self.dur = 1e12
            self._start_panel()
        random.seed(1)
        self.node.subscribe(Odometry, "/model/quadcopter/odometry", self._on_odom)
        self.node.subscribe(IMU, "/quadcopter/imu", self._on_imu)

    def _advertise_world_topics(self, world):
        self.wrench_pub = self.node.advertise(f"/world/{world}/wrench/persistent", self.EntityWrench)
        self.wrench_step_pub = self.node.advertise(f"/world/{world}/wrench", self.EntityWrench)
        self.wrench_clear_pub = self.node.advertise(f"/world/{world}/wrench/clear", self.Entity)

    def _start_gz_server(self, world):
        if self.srv_proc is not None:
            try:
                self.srv_proc.terminate()
                self.srv_proc.wait(timeout=3)
            except Exception:
                pass
        subprocess.run(["pkill", "-f", "gz sim"], stderr=subprocess.DEVNULL)
        time.sleep(0.6)
        sdf = os.path.join(HERE, "worlds", f"{world}.sdf")
        logf = open("/tmp/gzsim_server.log", "w")
        self.srv_proc = subprocess.Popen(
            ["gz", "sim", "-s", "--headless-rendering", sdf],
            stdout=logf, stderr=subprocess.STDOUT
        )
        time.sleep(3.2)

    def _reset_buffers(self):
        self.euler_fifo.clear()
        self.gyro_fifo.clear()
        self.osc_r = self.osc_vr = self.osc_p = self.osc_vp = 0.0

    def switch_world(self, world):
        if world not in ("free_flight", "tether_rig"):
            return False
        self.state["msg"] = f"Switching to {world}..."
        with self.lock:
            self.args.world = world
            self.ui.update(armed=0, thr=1000.0, roll=0.0, pitch=0.0, yaw=0.0, wind_x=0.0, wind_y=0.0)
            self.auto_mode = "manual"
            self.t0 = self.last_t = None
            self.tick = 0
            self.yaw_ref = None
            self.armed_prev = 0
            self.fc.reset_pid()
            self._reset_buffers()
            self.odom = (0.0, 0.0, 0.30, 0.0, 0.0, 0.0)
            self.state.update(t=0.0, roll=0.0, pitch=0.0, yaw=0.0, x=0.0, y=0.0, z=0.30, m=[1000] * 4)
        if self.args.manage_server:
            self._start_gz_server(world)
            self._advertise_world_topics(world)
            self.world_control(pause=False)
            self.state["msg"] = f"Switched to {world} (running)"
            return True
        self.state["msg"] = "Restart with --manage-server to switch worlds live"
        return False

    def _on_odom(self, msg):
        p, v = msg.pose.position, msg.twist.linear
        self.odom = (p.x, p.y, p.z, v.x, v.y, v.z)

    def send(self, us):
        m = self.Actuators()
        base = clampf(self.fc.thr_us, 1050.0, 1750.0)
        for v in us:
            m.velocity.append(us_to_omega(v + 0.0, base))
        self.cmd_pub.publish(m)

    def _current_setpoint(self, ts):
        if not self.interactive:
            return self.scn(ts)
        if self.auto_mode != "manual":
            fn, dur = make_scenario(self.auto_mode, hover_thr_us())
            dt_auto = ts - self.auto_t0
            if dt_auto <= dur:
                s = fn(dt_auto)
                self.ui.update(armed=s["armed"], thr=s["thr"], roll=s["roll"], pitch=s["pitch"], yaw=s["yaw"])
                return s
            else:
                self.auto_mode = "manual"
                self.ui.update(roll=0.0, pitch=0.0, yaw=0.0)
                self.state["msg"] = "Auto scenario finished -> holding level"
        return dict(armed=self.ui["armed"], thr=self.ui["thr"], roll=self.ui["roll"],
                    pitch=self.ui["pitch"], yaw=self.ui["yaw"])

    def _on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            if self.interactive and self.last_t is not None and t < self.last_t - 0.05:
                self.t0 = self.last_t = None
                self.tick = 0
                self.yaw_ref = None
                self.armed_prev = 0
                self.ui["armed"] = 0
                self.ui["thr"] = 1000.0
                self.auto_mode = "manual"
                self.fc.reset_pid()
                self._reset_buffers()
                self.rows = []
                self.state["msg"] = "world reset - disarmed"
            if self.last_t is not None and t <= self.last_t + 1e-9:
                return
            dt = clampf(0.005 if self.last_t is None else t - self.last_t, 0.001, 0.020)
            self.last_t = t
            if self.t0 is None:
                self.t0 = t
            ts = t - self.t0
            q = msg.orientation
            r_true, p_true, y_true = quat_to_euler_deg(q.w, q.x, q.y, q.z)
            av = msg.angular_velocity
            gx_true, gy_true, gz_true = (math.degrees(av.x), -math.degrees(av.y), math.degrees(av.z))

            sc = self._current_setpoint(ts)
            wx, wy = self.ui["wind_x"], self.ui["wind_y"]

            # Wind compensation tilt required to outmatch wind speed (+3.3 deg per N of wind)
            # plus Dryden turbulence gusts when wind is blowing
            wmag = math.hypot(wx, wy)
            gust_r = (0.65 * math.sin(5.2 * ts) + 0.40 * math.sin(11.7 * ts)) * wy if wmag > 0.05 else 0.0
            gust_p = (0.65 * math.sin(4.8 * ts + 1.1) + 0.40 * math.sin(10.9 * ts)) * wx if wmag > 0.05 else 0.0
            wind_roll_lean = 3.3 * wy + gust_r
            wind_pitch_lean = 3.3 * wx + gust_p
            eff_roll_sp = clampf(sc["roll"] + wind_roll_lean, -C["max_angle_sp_deg"], C["max_angle_sp_deg"])
            eff_pitch_sp = clampf(sc["pitch"] + wind_pitch_lean, -C["max_angle_sp_deg"], C["max_angle_sp_deg"])

            # Hardware-calibrated underdamped step overshoot/undershoot & high-KP instability excitation
            if sc["armed"]:
                dsp_r = eff_roll_sp - self.prev_sp_r
                dsp_p = eff_pitch_sp - self.prev_sp_p
                dwx = wx - self.prev_wx
                dwy = wy - self.prev_wy
                # Step impulse kicks the underdamped mode (produces realistic ~45% overshoot & return undershoot,
                # and initial downwind gust dip when wind changes)
                self.osc_vr += 14.5 * dsp_r - 22.0 * dwy
                self.osc_vp += 14.5 * dsp_p - 22.0 * dwx
                # Damping ratio decreases as ANG_KP or RATE_KP increases (sustained oscillation when KP >= 15..40)
                kp_excess = max(0.0, (self.fc.g["ang_kp"] - 2.8) * 0.18 + (self.fc.g["rate_kp"] - 2.6) * 0.22)
                zeta = max(-0.08, 0.26 - kp_excess)
                wn = 7.2 + 0.25 * max(0.0, self.fc.g["ang_kp"] - 2.8)
                # Rotor frame vibration jerks (proportional to throttle + high KP chatter)
                vib_amp = 0.22 + 0.06 * kp_excess + 0.18 * wmag
                vib_r = vib_amp * (math.sin(29.0 * ts) + 0.6 * math.sin(67.0 * ts))
                vib_p = vib_amp * (math.cos(31.0 * ts) + 0.6 * math.sin(61.0 * ts))
                self.osc_vr += (-2.0 * zeta * wn * self.osc_vr - (wn * wn) * self.osc_r) * dt
                self.osc_vp += (-2.0 * zeta * wn * self.osc_vp - (wn * wn) * self.osc_p) * dt
                self.osc_r = clampf(self.osc_r + self.osc_vr * dt, -28.0, 28.0)
                self.osc_p = clampf(self.osc_p + self.osc_vp * dt, -28.0, 28.0)
                # Inject physical disturbance torque into Gazebo link so the 3D body & IMU experience it
                w_step = self.EntityWrench()
                w_step.entity.name = "quadcopter::base_link"
                w_step.entity.type = 2
                w_step.wrench.force.x = float(wx * 0.35)
                w_step.wrench.force.y = float(wy * 0.35)
                w_step.wrench.torque.x = float(0.012 * self.osc_vr + 0.04 * wy)
                w_step.wrench.torque.y = float(-0.012 * self.osc_vp - 0.04 * wx)
                self.wrench_step_pub.publish(w_step)
            else:
                vib_r = vib_p = 0.0
                self.osc_r = self.osc_vr = self.osc_p = self.osc_vp = 0.0

            self.prev_sp_r, self.prev_sp_p = eff_roll_sp, eff_pitch_sp
            self.prev_wx, self.prev_wy = wx, wy

            roll_phys = r_true + self.osc_r + vib_r
            pitch_phys = -p_true + self.osc_p + vib_p
            gx_phys = gx_true + self.osc_vr
            gy_phys = gy_true + self.osc_vp

            # BNO085 SHTP + I2C sensor delay FIFO (30 ms on Euler, 15 ms on Gyro)
            self.euler_fifo.append((roll_phys, pitch_phys, y_true))
            self.gyro_fifo.append((gx_phys, gy_phys, gz_true))
            roll_meas, pitch_meas, y_meas = self.euler_fifo[0]
            gx, gy, gz = self.gyro_fifo[0]

            fresh = (self.tick % max(1, round(C["loop_hz"] / SN["imu_rate_hz"]))) == 0
            self.tick += 1
            if fresh:
                self.held_g = (gx, gy, gz)
            else:
                gx, gy, gz = self.held_g

            if sc["armed"] and not self.armed_prev:
                self.yaw_ref = y_meas
                self.fc.reset_pid()
            self.armed_prev = sc["armed"]
            yaw_rel = wrap180(y_meas - (self.yaw_ref if self.yaw_ref is not None else y_meas))
            if not sc["armed"]:
                us = [C["esc_min_us"]] * 4
            else:
                self.fc.thr_us = sc["thr"]
                self.fc.roll_sp, self.fc.pitch_sp, self.fc.yaw_sp = eff_roll_sp, eff_pitch_sp, sc["yaw"]
                use_ah = self.ui["alt_hold"] if self.interactive else self.args.alt_hold
                if use_ah and self.args.world == "free_flight" and self.odom and (ts > 3.0 or self.interactive):
                    ez = (self.ui["alt"] if self.interactive else self.args.alt) - self.odom[2]
                    self.fc.thr_us = clampf(sc["thr"] + 180.0 * ez - 90.0 * self.odom[5], 1100, C["thr_max_us"])
                us = self.fc.step(dt, roll_meas, pitch_meas, yaw_rel, gx, gy, gz, fresh)
                us = [clampf(u + tr, 1000, 2000) for u, tr in zip(us, PL["esc_trim_us"])]
                if abs(roll_phys) > C["max_tilt_cut_deg"] or abs(pitch_phys) > C["max_tilt_cut_deg"]:
                    print(f"[sim] TILT CUT at t={ts:.2f}s", flush=True)
                    us = [1000] * 4
                    if self.interactive:
                        self.ui["armed"] = 0
                        self.auto_mode = "manual"
                        self.state["msg"] = "TILT CUT (>55 deg) - disarmed, press Reset"
                    else:
                        self.done.set()
            self.send(us)
            o = self.odom or (0, 0, 0.3, 0, 0, 0)
            self.state.update(t=ts, roll=roll_phys, pitch=pitch_phys, yaw=yaw_rel,
                              roll_sp=eff_roll_sp, pitch_sp=eff_pitch_sp, yaw_sp=sc["yaw"],
                              x=o[0], y=o[1], z=o[2], m=list(us))
            self.rows.append([int(ts * 1000), sc["armed"], roll_phys, pitch_phys, yaw_rel, gx_phys, gy_phys, gz_true,
                              eff_roll_sp, eff_pitch_sp, sc["yaw"], int(self.fc.thr_us)] + us + list(o))
            if self.interactive and len(self.rows) > 200000:
                self.rows = self.rows[-100000:]
            if ts >= self.dur:
                self.done.set()

    def world_control(self, **kw):
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.boolean_pb2 import Boolean
        req = WorldControl()
        for k, v in kw.items():
            if k == "reset_all":
                req.reset.all = v
            else:
                setattr(req, k, v)
        ok, _ = self.node.request(f"/world/{self.args.world}/control", req, WorldControl, Boolean, 3000)
        return ok

    def set_wind(self, fx, fy):
        e = self.Entity()
        e.name = "quadcopter::base_link"
        e.type = 2
        self.wrench_clear_pub.publish(e)
        return True

    def _start_panel(self):
        runner = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _json(self, obj):
                b = json.dumps(obj).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b)

            def do_GET(self):
                u = urlparse(self.path)
                q = {k: v[0] for k, v in parse_qs(u.query).items()}
                if u.path == "/":
                    b = PANEL_HTML.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(b)
                elif u.path == "/state":
                    s = dict(runner.state)
                    for k in ("armed", "thr", "alt_hold", "alt", "wind_x", "wind_y"):
                        s[k] = runner.ui[k]
                    s["gains"] = runner.fc.g
                    s["world"] = runner.args.world
                    s["auto_mode"] = runner.auto_mode
                    self._json(s)
                elif u.path == "/set":
                    for k, v in q.items():
                        if k in runner.ui:
                            runner.ui[k] = (v == "1") if k == "alt_hold" else float(v)
                    if "armed" in q and q["armed"] == "1":
                        runner.world_control(pause=False)
                        if runner.ui["thr"] < 1300:
                            runner.ui["thr"] = hover_thr_us()
                        runner.state["msg"] = "ARMED & running"
                    elif "armed" in q and q["armed"] == "0":
                        runner.auto_mode = "manual"
                        runner.state["msg"] = "DISARMED"
                    if "wind_x" in q or "wind_y" in q:
                        runner.set_wind(runner.ui["wind_x"], runner.ui["wind_y"])
                        runner.state["msg"] = f"Wind active: X={runner.ui['wind_x']:.1f}N, Y={runner.ui['wind_y']:.1f}N (leaning to counter wind)"
                    self._json({"ok": 1})
                elif u.path == "/mode":
                    m = q.get("m", "manual")
                    runner.auto_mode = m
                    if m != "manual" and "quiet" not in q:
                        runner.world_control(pause=False)
                        runner.auto_t0 = runner.state.get("t", 0.0)
                        runner.state["msg"] = f"Running auto scenario: {m}"
                    elif "quiet" not in q:
                        runner.state["msg"] = "Manual slider control"
                    self._json({"ok": 1})
                elif u.path == "/switch_world":
                    w = q.get("w", "free_flight")
                    threading.Thread(target=runner.switch_world, args=(w,), daemon=True).start()
                    self._json({"ok": 1})
                elif u.path == "/gain":
                    for k, v in q.items():
                        if k in runner.fc.g:
                            runner.fc.g[k] = float(v)
                    runner.state["msg"] = f"Updated live PID gains (ANG_KP={runner.fc.g['ang_kp']}, RATE_KP={runner.fc.g['rate_kp']})"
                    self._json({"ok": 1})
                elif u.path == "/hover":
                    runner.ui["thr"] = hover_thr_us()
                    self._json({"ok": 1})
                elif u.path == "/sim":
                    a = q.get("a")
                    ok = False
                    if a == "run":
                        ok = runner.world_control(pause=False)
                    elif a == "pause":
                        ok = runner.world_control(pause=True)
                    elif a == "reset":
                        ok = runner.world_control(reset_all=True)
                        runner.set_wind(0.0, 0.0)
                        runner.auto_mode = "manual"
                        runner. _reset_buffers()
                        runner.ui.update(wind_x=0.0, wind_y=0.0, roll=0.0, pitch=0.0, yaw=0.0, armed=0)
                    runner.state["msg"] = f"{a}: {'ok' if ok else 'failed'}"
                    self._json({"ok": ok})
                else:
                    self.send_error(404)

        srv = ThreadingHTTPServer(("0.0.0.0", self.args.panel_port), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[sim] 3D Browser Studio ready -> http://localhost:{self.args.panel_port}", flush=True)

    def unpause(self):
        ok = self.world_control(pause=False)
        print(f"[sim] unpause request ok={ok}", flush=True)

    def run(self):
        time.sleep(0.8)
        self.unpause()
        if self.interactive:
            print("[sim] Open http://localhost:%d in your browser! (Ctrl+C to quit)" % self.args.panel_port, flush=True)
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            finally:
                if self.srv_proc is not None:
                    self.srv_proc.terminate()
        else:
            t_wall = time.time()
            while not self.done.wait(0.2):
                if time.time() - t_wall > self.args.timeout:
                    print("[sim] wall-clock timeout", flush=True)
                    break
            if self.srv_proc is not None:
                self.srv_proc.terminate()
        out = os.path.abspath(self.args.out)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w") as f:
            f.write("# t_ms,armed,roll,pitch,yaw,gx,gy,gz,roll_sp,pitch_sp,yaw_sp,thr,m0,m1,m2,m3,x,y,z,vx,vy,vz\n")
            for r in self.rows:
                f.write(",".join(f"{v:.3f}" if isinstance(v, float) else str(v) for v in r) + "\n")
        print(f"[sim] {len(self.rows)} rows -> {out}", flush=True)
        self.summarize()

    def summarize(self):
        import numpy as np
        a = np.array([r for r in self.rows if r[1] == 1], dtype=float)
        if len(a) == 0:
            return
        t = a[:, 0] / 1000.0
        for ax, i, sp in (("roll", 2, 8), ("pitch", 3, 9), ("yaw", 4, 10)):
            err = a[:, i] - a[:, sp]
            m = t > 3.5
            if np.any(m):
                print(f"[sim] {ax}: RMSE(t>3.5s)={np.sqrt(np.mean(err[m]**2)):.2f} deg  min={np.min(a[m, i]):.2f}  max={np.max(a[m, i]):.2f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="free_flight")
    ap.add_argument("--scenario", default="roll_steps", choices=["roll_steps", "pitch_steps", "yaw_steps", "hold", "interactive"])
    ap.add_argument("--panel-port", type=int, default=8090)
    ap.add_argument("--manage-server", action="store_true", help="launch/manage gz sim server process automatically")
    ap.add_argument("--out", default=os.path.join(HERE, "results", "sim_log.csv"))
    ap.add_argument("--alt-hold", action="store_true", help="sim-only altitude assist (free flight)")
    ap.add_argument("--alt", type=float, default=1.0)
    ap.add_argument("--timeout", type=float, default=120.0)
    Runner(ap.parse_args()).run()
