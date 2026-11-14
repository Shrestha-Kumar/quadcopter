#!/usr/bin/env python3
"""
Wi-Fi Web Ground Station & Real-Time Telemetry Logger for EE302 Quadcopter.
Connects to the Arduino Mega 2560 (/dev/ttyUSB0, 115200 baud) and serves an
interactive mobile/desktop control dashboard over your Wi-Fi network ("Me")
at http://0.0.0.0:8080 (e.g., http://10.76.89.94:8080 on your phone).

Features:
- Big red EMERGENCY STOP (instant DISARM)
- Live IMU Roll/Pitch/Yaw & Motor 1..4 PWM display
- 1-click I2C SCAN, IMU_TEST, and Props-Off ESC_TEST (Motors 0..3)
- ARM / DISARM, UP / DOWN (+/-10 us), Throttle slider, Roll/Pitch/Yaw step buttons
- Automatic background PING keepalive while ARMED (prevents 1s watchdog timeout)
- Automatic logging of all "T,..." telemetry lines to logs/flight_log_YYYYMMDD_HHMMSS.csv
"""
import os
import glob
import time
import json
import select
import termios
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

PORT_DEV = "/dev/ttyUSB0"
HTTP_PORT = 8080
LOGS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
os.makedirs(LOGS_DIR, exist_ok=True)


class SerialBridge:
    def __init__(self, dev=PORT_DEV):
        self.dev = dev
        self.fd = None
        self.lock = threading.RLock()
        self.recent_lines = []
        self.latest_tel = {
            "t_ms": 0, "armed": 0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
            "roll_sp": 0.0, "pitch_sp": 0.0, "yaw_sp": 0.0,
            "thr": 1000, "m": [1000, 1000, 1000, 1000]
        }
        self.keepalive_enabled = False
        self.last_tx_time = 0.0
        self.log_path = os.path.join(LOGS_DIR, f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.log_file = open(self.log_path, "a", buffering=1)
        self.connect()
        threading.Thread(target=self._reader_loop, daemon=True).start()
        threading.Thread(target=self._keepalive_loop, daemon=True).start()

    def connect(self):
        with self.lock:
            if self.fd is not None:
                try:
                    os.close(self.fd)
                except OSError:
                    pass
                self.fd = None
            ports = sorted(glob.glob("/dev/ttyUSB*"))
            if ports:
                self.dev = ports[0]
            try:
                fd = os.open(self.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                attrs = termios.tcgetattr(fd)
                attrs[0] = 0
                attrs[1] = 0
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.B115200
                attrs[3] = 0
                attrs[4] = termios.B115200
                attrs[5] = termios.B115200
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                self.fd = fd
                self._append_line(f"[SYS] Connected to {self.dev} @ 115200 baud")
                threading.Timer(0.2, lambda: self.send("TEL:1", quiet=True)).start()
                threading.Timer(2.5, lambda: self.send("TEL:1", quiet=True)).start()
                threading.Timer(4.5, lambda: self.send("TEL:1", quiet=True)).start()
            except Exception as e:
                self._append_line(f"[SYS] Waiting for {self.dev}: {e}")

    def _append_line(self, line):
        self.recent_lines.append(line)
        if len(self.recent_lines) > 80:
            self.recent_lines = self.recent_lines[-80:]

    def send(self, cmd, quiet=False):
        cmd = cmd.strip()
        if not quiet and cmd.upper() != "PING":
            self._append_line(f"> {cmd}")
        if cmd.upper() == "ARM":
            self.keepalive_enabled = True
        elif cmd.upper() in ("DISARM", "STOP"):
            self.keepalive_enabled = False
        with self.lock:
            if self.fd is None:
                self.connect()
            if self.fd is not None:
                try:
                    # Pace rapid non-emergency commands so Mega's 64-byte UART RX buffer never drops bytes
                    if cmd.upper() not in ("STOP", "DISARM"):
                        gap = time.monotonic() - self.last_tx_time
                        if gap < 0.025:
                            time.sleep(0.025 - gap)
                    os.write(self.fd, (cmd + "\n").encode("utf-8"))
                    self.last_tx_time = time.monotonic()
                except Exception as e:
                    self._append_line(f"[ERR] Write failed: {e}")
                    self.connect()

    def _keepalive_loop(self):
        while True:
            time.sleep(0.35)
            if self.keepalive_enabled:
                self.send("PING", quiet=True)

    def _reader_loop(self):
        buf = ""
        while True:
            if self.fd is None:
                time.sleep(1.0)
                self.connect()
                continue
            try:
                r, _, _ = select.select([self.fd], [], [], 0.05)
                if r:
                    chunk = os.read(self.fd, 4096)
                    if not chunk:
                        with self.lock:
                            try:
                                os.close(self.fd)
                            except OSError:
                                pass
                            self.fd = None
                        continue
                    buf += chunk.decode("utf-8", errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        if line.startswith("T,"):
                            self.log_file.write(line + "\n")
                            parts = line.split(",")
                            if len(parts) >= 17:
                                try:
                                    self.latest_tel = {
                                        "t_ms": int(parts[1]),
                                        "armed": int(parts[2]),
                                        "roll": float(parts[3]),
                                        "pitch": float(parts[4]),
                                        "yaw": float(parts[5]),
                                        "gx": float(parts[6]),
                                        "gy": float(parts[7]),
                                        "gz": float(parts[8]),
                                        "roll_sp": float(parts[9]),
                                        "pitch_sp": float(parts[10]),
                                        "yaw_sp": float(parts[11]),
                                        "thr": int(parts[12]),
                                        "m": [int(parts[13]), int(parts[14]), int(parts[15]), int(parts[16])]
                                    }
                                    self.keepalive_enabled = (self.latest_tel["armed"] == 1)
                                except ValueError:
                                    pass
                        elif line != "PONG":
                            self._append_line(line)
            except Exception:
                time.sleep(0.5)
                self.connect()


HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>EE302 Quadcopter Wi-Fi Ground Station</title>
<style>
  body { font-family: system-ui, sans-serif; background: #111827; color: #f9fafb; margin: 0; padding: 14px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(290px, 1fr)); gap: 12px; }
  .card { background: #1f2937; border: 1px solid #374151; border-radius: 10px; padding: 14px; }
  h2 { margin: 0 0 10px 0; font-size: 16px; color: #93c5fd; }
  button { font-size: 15px; font-weight: 600; padding: 10px 14px; margin: 4px; border-radius: 8px; border: none; cursor: pointer; }
  .btn-stop { background: #dc2626; color: white; width: 100%; font-size: 20px; padding: 16px; margin: 0 0 12px 0; }
  .btn-arm { background: #059669; color: white; }
  .btn-disarm { background: #d97706; color: white; }
  .btn-cmd { background: #2563eb; color: white; }
  .btn-sec { background: #4b5563; color: white; }
  .stat { font-size: 22px; font-weight: bold; font-family: monospace; }
  pre { background: #030712; color: #10b981; padding: 10px; border-radius: 6px; height: 210px; overflow-y: auto; font-size: 12px; margin: 0; }
</style>
</head>
<body>
<button class="btn-stop" onclick="send('STOP')">EMERGENCY STOP / DISARM (SPACEBAR)</button>
<div class="grid">
  <div class="card">
    <h2>1. Live Attitude &amp; Motors</h2>
    <div id="state_badge" class="stat" style="color:#fbbf24">DISARMED</div>
    <div id="imu_badge" style="font-size:13px;font-weight:700;margin:4px 0;color:#fbbf24">IMU: CHECKING...</div>
    <p class="stat">Roll: <span id="roll">0.0</span>&deg; | Pitch: <span id="pitch">0.0</span>&deg; | Yaw: <span id="yaw">0.0</span>&deg;</p>
    <p class="stat">Thr: <span id="thr">1000</span> us</p>
    <p style="font-family:monospace">M0(FR,Pin9): <span id="m0">1000</span> | M1(FL,Pin8): <span id="m1">1000</span><br>
       M2(RL,Pin7): <span id="m2">1000</span> | M3(RR,Pin6): <span id="m3">1000</span></p>
    <button class="btn-cmd" onclick="send('SCAN')">1. SCAN I2C (IMU)</button>
    <button class="btn-cmd" onclick="send('CAL')">Zero Level (CAL)</button>
    <button class="btn-cmd" onclick="send('TEL:1')">Stream Telemetry ON</button>
    <button class="btn-sec" onclick="send('STATUS')">STATUS</button>
  </div>

  <div class="card">
    <h2>2. Props-OFF Bench ESC Test (2s each)</h2>
    <p style="font-size:13px;color:#9ca3af">Only works while DISARMED with propellers REMOVED:</p>
    <button class="btn-sec" onclick="send('ESC_TEST:0:1150')">Test M0 (FR, Pin 9)</button>
    <button class="btn-sec" onclick="send('ESC_TEST:1:1150')">Test M1 (FL, Pin 8)</button>
    <button class="btn-sec" onclick="send('ESC_TEST:2:1150')">Test M2 (RL, Pin 7)</button>
    <button class="btn-sec" onclick="send('ESC_TEST:3:1150')">Test M3 (RR, Pin 6)</button>
  </div>

  <div class="card">
    <h2>3. Flight Control (Tether Rig — Up to 1700 us)</h2>
    <button class="btn-arm" onclick="document.getElementById('thr_slider').value=140;document.getElementById('thr_lbl').textContent='140 (1095 us)';send('THR:1000');setTimeout(()=>send('ARM'),100)">ARM (Slow Spin)</button>
    <button class="btn-disarm" onclick="document.getElementById('thr_slider').value=0;document.getElementById('thr_lbl').textContent='0 (1000 us)';send('DISARM')">DISARM</button>
    <button class="btn-cmd" onclick="send('UP')">THR UP (+10)</button>
    <button class="btn-cmd" onclick="send('DOWN')">THR DOWN (-10)</button>
    <div style="margin:12px 0;padding:8px;background:#111827;border-radius:8px">
      <label style="font-size:14px;font-weight:600">Speed Slider (0 - 1000 &rarr; 1000 - 1700 us): <span id="thr_lbl">0 (1000 us)</span></label><br>
      <input id="thr_slider" type="range" min="0" max="1000" step="20" value="0" style="width:100%;margin-top:6px"
        oninput="const us=Math.round(1000+this.value*0.7);document.getElementById('thr_lbl').textContent=this.value+' ('+us+' us)';send('THR:'+us)">
    </div>
    <div style="margin:10px 0;padding:10px;background:#111827;border-radius:8px;font-size:13px">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px">
        <b>Attitude Setpoint Sliders (&plusmn;20&deg;)</b>
        <button class="btn-cmd" style="padding:5px 10px;font-size:12px;margin:0" onclick="levelAll()">Level All (0&deg;)</button>
      </div>
      <div style="margin:6px 0">
        <label>Roll SP (-20&deg;..+20&deg;): <b id="r_sp_lbl" style="color:#93c5fd">0&deg;</b></label>
        <input id="r_slider" type="range" min="-20" max="20" step="1" value="0" style="width:100%"
          oninput="setAngle('ROLL',this.value,'r_sp_lbl')">
        <div style="display:flex;gap:4px;flex-wrap:wrap;margin-top:2px">
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('r_slider',-10,'ROLL','r_sp_lbl')">-10&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('r_slider',-5,'ROLL','r_sp_lbl')">-5&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('r_slider',0,'ROLL','r_sp_lbl')">0&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('r_slider',5,'ROLL','r_sp_lbl')">+5&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('r_slider',10,'ROLL','r_sp_lbl')">+10&deg;</button>
        </div>
      </div>
      <div style="margin:8px 0">
        <label>Pitch SP (-20&deg;..+20&deg;): <b id="p_sp_lbl" style="color:#93c5fd">0&deg;</b></label>
        <input id="p_slider" type="range" min="-20" max="20" step="1" value="0" style="width:100%"
          oninput="setAngle('PITCH',this.value,'p_sp_lbl')">
        <div style="display:flex;gap:4px;flex-wrap:wrap;margin-top:2px">
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('p_slider',-10,'PITCH','p_sp_lbl')">-10&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('p_slider',-5,'PITCH','p_sp_lbl')">-5&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('p_slider',0,'PITCH','p_sp_lbl')">0&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('p_slider',5,'PITCH','p_sp_lbl')">+5&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('p_slider',10,'PITCH','p_sp_lbl')">+10&deg;</button>
        </div>
      </div>
      <div style="margin:8px 0 2px 0">
        <label>Yaw SP (-20&deg;..+20&deg;, &plusmn;10&deg; steps): <b id="y_sp_lbl" style="color:#93c5fd">0&deg;</b></label>
        <input id="y_slider" type="range" min="-20" max="20" step="10" value="0" style="width:100%"
          oninput="setAngle('YAW',this.value,'y_sp_lbl')">
        <div style="display:flex;gap:4px;flex-wrap:wrap;margin-top:2px">
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('y_slider',-20,'YAW','y_sp_lbl')">-20&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('y_slider',-10,'YAW','y_sp_lbl')">-10&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('y_slider',0,'YAW','y_sp_lbl')">0&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('y_slider',10,'YAW','y_sp_lbl')">+10&deg;</button>
          <button class="btn-sec" style="padding:4px 8px;font-size:12px" onclick="setSlider('y_slider',20,'YAW','y_sp_lbl')">+20&deg;</button>
          <button class="btn-cmd" style="padding:4px 8px;font-size:12px" onclick="stepYaw(-10)">Yaw -10&deg; Step</button>
          <button class="btn-cmd" style="padding:4px 8px;font-size:12px" onclick="stepYaw(10)">Yaw +10&deg; Step</button>
        </div>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>4. Live PID Tuning &amp; Yaw Lock (Instant, No Re-Upload!)</h2>
    <p style="font-size:12px;color:#9ca3af;margin:0 0 6px 0">1-Click Presets (takes effect immediately):</p>
    <button class="btn-arm" onclick="setPID(2.60,1.65,0.20,0.022)">Calmer (2.60 / 0.20)</button>
    <button class="btn-cmd" onclick="setPID(2.80,2.60,0.80,0.150)">Tuned (Hardcoded Default)</button>
    <button class="btn-sec" onclick="setPID(4.20,1.95,0.50,0.032)">Sharp (High Stiffness)</button>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin:8px 0;font-size:12px">
      <label>ANG_KP <input id="g_akp" type="number" step="0.1" value="2.80" style="width:65px;background:#111827;color:white;border:1px solid #4b5563;border-radius:4px;padding:3px"></label>
      <label>RATE_KP <input id="g_rkp" type="number" step="0.05" value="2.60" style="width:65px;background:#111827;color:white;border:1px solid #4b5563;border-radius:4px;padding:3px"></label>
      <label>RATE_KI <input id="g_rki" type="number" step="0.05" value="0.80" style="width:65px;background:#111827;color:white;border:1px solid #4b5563;border-radius:4px;padding:3px"></label>
      <label>RATE_KD <input id="g_rkd" type="number" step="0.002" value="0.150" style="width:65px;background:#111827;color:white;border:1px solid #4b5563;border-radius:4px;padding:3px"></label>
    </div>
    <button class="btn-cmd" onclick="applyCustomPID()">Apply PID Live</button>
    <button class="btn-sec" onclick="send('GET')">Show Gains (GET)</button>
    <div id="active_gains" style="font-size:11px;font-family:monospace;color:#10b981;margin:6px 0">Active: (click GET or Apply PID Live)</div>
    <button class="btn-sec" onclick="send('SET:YAW_TRIM=-15')">Yaw Trim -15</button>
    <button class="btn-sec" onclick="send('SET:YAW_TRIM=0')">Yaw Trim 0</button>
    <button class="btn-sec" onclick="send('SET:YAW_TRIM=15')">Yaw Trim +15</button>
    <button class="btn-sec" onclick="send('SET:YAW_DIR=1')">Yaw Dir +1</button>
    <button class="btn-sec" onclick="send('SET:YAW_DIR=-1')">Yaw Dir -1</button>
  </div>

  <div class="card">
    <h2>5. Serial Console &amp; Custom Command</h2>
    <div style="display:flex;gap:6px;margin-bottom:8px">
      <input id="cmd_in" style="flex:1;padding:8px;border-radius:6px;border:1px solid #4b5563;background:#111827;color:white" placeholder="e.g. GET or SET:RATE_KP=0.56">
      <button class="btn-cmd" onclick="send(document.getElementById('cmd_in').value)">Send</button>
    </div>
    <pre id="console"></pre>
  </div>
</div>
<script>
async function send(c) {
  if (!c) return;
  await fetch('/api/cmd?c=' + encodeURIComponent(c));
}
function setAngle(axis, val, lblId) {
  document.getElementById(lblId).innerHTML = val + '&deg;';
  send(axis + ':' + val);
}
function setSlider(sliderId, val, axis, lblId) {
  const el = document.getElementById(sliderId);
  el.value = val;
  setAngle(axis, val, lblId);
}
function stepYaw(delta) {
  const el = document.getElementById('y_slider');
  const next = Math.max(-20, Math.min(20, parseInt(el.value || '0', 10) + delta));
  setSlider('y_slider', next, 'YAW', 'y_sp_lbl');
}
function levelAll() {
  document.getElementById('r_slider').value = 0; document.getElementById('r_sp_lbl').innerHTML = '0&deg;';
  document.getElementById('p_slider').value = 0; document.getElementById('p_sp_lbl').innerHTML = '0&deg;';
  document.getElementById('y_slider').value = 0; document.getElementById('y_sp_lbl').innerHTML = '0&deg;';
  send('ROLL:0'); send('PITCH:0'); send('YAW:0');
}
async function setPID(akp, rkp, rki, rkd) {
  document.getElementById('g_akp').value = akp;
  document.getElementById('g_rkp').value = rkp;
  document.getElementById('g_rki').value = rki;
  document.getElementById('g_rkd').value = rkd;
  await applyCustomPID();
}
async function applyCustomPID() {
  const akp = document.getElementById('g_akp').value;
  const rkp = document.getElementById('g_rkp').value;
  const rki = document.getElementById('g_rki').value;
  const rkd = document.getElementById('g_rkd').value;
  await send('PID:' + akp + ',' + rkp + ',' + rki + ',' + rkd);
}
let lastSig = '', staleTicks = 0;
document.addEventListener('keydown', (e) => {
  if (e.code === 'Space') { e.preventDefault(); send('STOP'); }
});
setInterval(async () => {
  try {
    const r = await fetch('/api/state');
    const d = await r.json();
    const t = d.tel;
    const sb = document.getElementById('state_badge');
    sb.textContent = t.armed ? 'ARMED (ACTIVE)' : 'DISARMED (SAFE)';
    sb.style.color = t.armed ? '#10b981' : '#fbbf24';
    const sig = [t.roll, t.pitch, t.yaw, t.gx, t.gy, t.gz].join(',');
    if (sig !== lastSig) { lastSig = sig; staleTicks = 0; } else { staleTicks++; }
    const ib = document.getElementById('imu_badge');
    if (t.t_ms === 0) { ib.textContent = 'IMU: WAITING FOR USB...'; ib.style.color = '#fbbf24'; }
    else if (staleTicks >= 15) { ib.textContent = 'IMU FROZEN! (Check wires & click SCAN)'; ib.style.color = '#ef4444'; }
    else { ib.textContent = 'IMU LIVE (50 Hz)'; ib.style.color = '#10b981'; }
    document.getElementById('roll').textContent = t.roll.toFixed(1);
    document.getElementById('pitch').textContent = t.pitch.toFixed(1);
    document.getElementById('yaw').textContent = t.yaw.toFixed(1);
    document.getElementById('thr').textContent = t.thr;
    for (let i=0; i<4; i++) document.getElementById('m'+i).textContent = t.m[i];
    const con = document.getElementById('console');
    con.textContent = d.lines.join('\\n');
    con.scrollTop = con.scrollHeight;
    for (let i = d.lines.length - 1; i >= 0; i--) {
      if (d.lines[i].startsWith('GAINS ')) {
        document.getElementById('active_gains').textContent = 'Active: ' + d.lines[i].slice(6);
        break;
      }
    }
  } catch (e) {}
}, 200);
</script>
</body>
</html>
"""

bridge = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
        elif parsed.path == "/api/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            payload = {"tel": bridge.latest_tel, "lines": bridge.recent_lines[-30:], "log": bridge.log_path}
            self.wfile.write(json.dumps(payload).encode("utf-8"))
        elif parsed.path == "/api/cmd":
            qs = parse_qs(parsed.query)
            c = qs.get("c", [""])[0]
            if c:
                bridge.send(c)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_response(404)
            self.end_headers()


if __name__ == "__main__":
    bridge = SerialBridge()
    ThreadingHTTPServer.allow_reuse_address = True
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"=== Quadcopter Wi-Fi Ground Station listening on http://0.0.0.0:{HTTP_PORT} ===")
    print(f"Logging telemetry to: {bridge.log_path}")
    srv.serve_forever()
