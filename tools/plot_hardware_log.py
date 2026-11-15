#!/usr/bin/env python3
"""
Generate Fig 4: Experimental Hardware Closed-Loop PID Validation Plot
from recorded 50 Hz BNO085 + Arduino Mega telemetry (logs/final_flight_log.csv),
using the SVG + rsvg-convert renderer from generate_results.py.
"""
import os
import sys
import shutil
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_results import render_svg_grid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LOG = os.path.join(ROOT, "logs", "final_flight_log.csv")


def load_longest_armed_block(path):
    blocks = []
    cur = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) == 17 and p[0] == "T":
                try:
                    vals = [float(x) for x in p[1:]]
                except ValueError:
                    continue
                if int(vals[1]) == 1:
                    if cur and vals[0] < cur[-1][0]:
                        blocks.append(np.array(cur))
                        cur = [vals]
                    else:
                        cur.append(vals)
                else:
                    if cur:
                        blocks.append(np.array(cur))
                        cur = []
    if cur:
        blocks.append(np.array(cur))
    if not blocks:
        return None
    blocks.sort(key=lambda b: len(b), reverse=True)
    return blocks[0]


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG
    seg = load_longest_armed_block(log_path)
    if seg is None or len(seg) == 0:
        print("No armed data in", log_path)
        return

    # Skip initial 2.5 s arming spin-up transient if block is long (> 15 s)
    if len(seg) > 750:
        seg = seg[250:]

    t = (seg[:, 0] - seg[0, 0]) / 1000.0
    tmax = float(t[-1])

    roll = seg[:, 2]
    pitch = seg[:, 3]
    yaw = seg[:, 4]
    gx = seg[:, 5]
    gy = seg[:, 6]
    gz = seg[:, 7]
    roll_sp = seg[:, 8]
    pitch_sp = seg[:, 9]
    yaw_sp = seg[:, 10]
    thr = seg[:, 11]
    m0 = seg[:, 12]
    m1 = seg[:, 13]
    m2 = seg[:, 14]
    m3 = seg[:, 15]

    # Analytical outer+inner P-D motor command model (ANG_KP=2.80, RATE_KP=2.60)
    rate_sp_r = np.clip(2.80 * (roll_sp - roll), -150.0, 150.0)
    rate_sp_p = np.clip(2.80 * (pitch_sp - pitch), -150.0, 150.0)
    u_r_sim = np.clip(2.60 * (rate_sp_r - gx), -250.0, 250.0)
    u_p_sim = np.clip(2.60 * (rate_sp_p - gy), -250.0, 250.0)
    m0_model = np.clip(thr - u_r_sim + u_p_sim + 35.0, 1050.0, 1750.0)

    out_png = render_svg_grid(
        "fig4_hardware_validation",
        "Fig 4: Experimental Hardware Closed-Loop PID Validation (Arduino Mega 2560 + BNO085 @ 50 Hz Telemetry)",
        [
            dict(
                title="A. Roll & Pitch Setpoint Tracking (+-20 deg Steps & 0 deg Disturbance Rejection)",
                xlabel="Time [s]", ylabel="Angle [deg]",
                xlim=(0, tmax), ylim=(-30, 26),
                series=[
                    dict(x=t, y=roll_sp, color="#1f77b4", dashed=True, label="Roll Setpoint (deg)"),
                    dict(x=t, y=roll, color="#1f77b4", label="Measured Roll (deg)"),
                    dict(x=t, y=pitch_sp, color="#d62728", dashed=True, label="Pitch Setpoint (deg)"),
                    dict(x=t, y=pitch, color="#d62728", label="Measured Pitch (deg)"),
                ],
            ),
            dict(
                title="B. Yaw Heading Tracking (10 deg Steps) & Calibrated BNO085 Gyro Rates",
                xlabel="Time [s]", ylabel="Yaw [deg] / Rate [deg/s]",
                xlim=(0, tmax), ylim=(-65, 65),
                series=[
                    dict(x=t, y=yaw_sp, color="#2ca02c", dashed=True, label="Yaw Setpoint (deg)"),
                    dict(x=t, y=yaw, color="#2ca02c", label="Measured Yaw (deg)"),
                    dict(x=t, y=gx, color="#9467bd", label="Roll Gyro Rate gx (deg/s)"),
                    dict(x=t, y=gz, color="#8c564b", label="Yaw Gyro Rate gz (deg/s)"),
                ],
            ),
            dict(
                title="C. Hardware Closed-Loop Motor PWM Outputs (M0..M3 @ 1350 us Hover Throttle)",
                xlabel="Time [s]", ylabel="Motor PWM [us]",
                xlim=(0, tmax), ylim=(1040, 1720),
                series=[
                    dict(x=t, y=m0, color="#d62728", label="M0 (Front-Right, Pin 9)"),
                    dict(x=t, y=m1, color="#1f77b4", label="M1 (Front-Left, Pin 8)"),
                    dict(x=t, y=m2, color="#2ca02c", label="M2 (Rear-Left, Pin 7)"),
                    dict(x=t, y=m3, color="#ff7f0e", label="M3 (Rear-Right, Pin 6)"),
                ],
            ),
            dict(
                title="D. Hardware vs. Analytical Cascaded PID Mixer Response (Front-Right M0 Channel)",
                xlabel="Time [s]", ylabel="PWM Command [us]",
                xlim=(0, tmax), ylim=(1080, 1680),
                series=[
                    dict(x=t, y=m0, color="#d62728", label="Measured Hardware M0 (us)"),
                    dict(x=t, y=m0_model, color="#1f77b4", dashed=True, label="Analytical Cascaded PID Model (us)"),
                ],
            ),
        ],
        ncols=2,
    )
    res_dir = os.path.join(ROOT, "matlab", "results")
    os.makedirs(res_dir, exist_ok=True)
    dst_png = os.path.join(res_dir, "fig4_hardware_validation.png")
    if os.path.abspath(out_png) != os.path.abspath(dst_png):
        shutil.copy2(out_png, dst_png)
        out_svg = out_png.replace(".png", ".svg")
        if os.path.exists(out_svg):
            shutil.copy2(out_svg, os.path.join(res_dir, "fig4_hardware_validation.svg"))
    print(f"Saved hardware validation figure -> {out_png} (and {dst_png})")


if __name__ == "__main__":
    main()
