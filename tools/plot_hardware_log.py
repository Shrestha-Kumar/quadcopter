#!/usr/bin/env python3
"""
Generate Fig 4: Experimental Hardware Closed-Loop PID Validation Plot
from recorded 50 Hz BNO085 + Arduino Mega telemetry (logs/flight_log_*.csv),
using the SVG + rsvg-convert renderer from generate_results.py.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_results import render_svg_grid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_LOG = os.path.join(ROOT, "logs", "flight_log_20261112_170528.csv")


def load_log(path):
    rows = []
    with open(path) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) == 17 and p[0] == "T":
                try:
                    rows.append([float(x) for x in p[1:]])
                except ValueError:
                    pass
    return np.array(rows)


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG
    data = load_log(log_path)
    armed_idx = np.where(data[:, 1] == 1)[0]
    if len(armed_idx) == 0:
        print("No armed data in", log_path)
        return

    seg = data[armed_idx]
    if len(seg) > 300:
        seg = seg[-300:]
    t = np.arange(len(seg)) * 0.02
    tmax = float(t[-1])

    roll = seg[:, 2]
    pitch = seg[:, 3]
    yaw = seg[:, 4]
    gx = seg[:, 5]
    gy = seg[:, 6]
    gz = seg[:, 7]
    m0 = seg[:, 12]
    m1 = seg[:, 13]
    m2 = seg[:, 14]
    m3 = seg[:, 15]

    # Theoretical instantaneous P-D motor command for comparison on Roll/Pitch
    u_r_sim = -4.0 * 0.5656 * roll - 0.5656 * gx
    u_p_sim = -4.0 * 0.5656 * pitch - 0.5656 * gy
    m3_model = np.clip(1150.0 - u_r_sim - u_p_sim + 70.0, 1100.0, 1400.0)

    out_png = render_svg_grid(
        "fig4_hardware_validation",
        "Fig 4: Experimental Hardware Closed-Loop PID Validation (Arduino Mega 2560 + BNO085 @ 50 Hz Telemetry)",
        [
            dict(
                title="A. Real-Time BNO085 Quaternion Attitude Tracking Under External Hand-Tilt Disturbances",
                xlabel="Time [s]", ylabel="Angle [deg]",
                xlim=(0, tmax), ylim=(-20, 25),
                series=[
                    dict(x=t, y=roll, color="#1f77b4", label="Measured Roll (deg)"),
                    dict(x=t, y=pitch, color="#d62728", label="Measured Pitch (deg)"),
                    dict(x=t, y=yaw, color="#2ca02c", label="Measured Yaw (deg)"),
                    dict(x=[0, tmax], y=[0, 0], color="#666666", dashed=True, label="Setpoint (0 deg)"),
                ],
            ),
            dict(
                title="B. Calibrated BNO085 3-Axis Gyroscope Angular Rates (Inner Rate PID Loop Feedback)",
                xlabel="Time [s]", ylabel="Angular Rate [deg/s]",
                xlim=(0, tmax), ylim=(-60, 50),
                series=[
                    dict(x=t, y=gx, color="#1f77b4", label="Roll Rate gx (deg/s)"),
                    dict(x=t, y=gy, color="#d62728", label="Pitch Rate gy (deg/s)"),
                    dict(x=t, y=gz, color="#2ca02c", label="Yaw Rate gz (deg/s)"),
                ],
            ),
            dict(
                title="C. Hardware Closed-Loop Motor PWM Outputs (M0..M3) Opposing Tilt Disturbances",
                xlabel="Time [s]", ylabel="Motor PWM [us]",
                xlim=(0, tmax), ylim=(1080, 1380),
                series=[
                    dict(x=t, y=m0, color="#d62728", label="M0 (Front-Right, Pin 9)"),
                    dict(x=t, y=m1, color="#1f77b4", label="M1 (Front-Left, Pin 8)"),
                    dict(x=t, y=m2, color="#2ca02c", label="M2 (Rear-Left, Pin 7)"),
                    dict(x=t, y=m3, color="#ff7f0e", label="M3 (Rear-Right, Pin 6)"),
                ],
            ),
            dict(
                title="D. Hardware vs. Analytical Cascaded PID Mixer Response (Rear-Right M3 Channel)",
                xlabel="Time [s]", ylabel="PWM Command [us]",
                xlim=(0, tmax), ylim=(1080, 1380),
                series=[
                    dict(x=t, y=m3, color="#ff7f0e", label="Measured Hardware M3 (us)"),
                    dict(x=t, y=m3_model, color="#1f77b4", dashed=True, label="Analytical PID Model (us)"),
                ],
            ),
        ],
        ncols=2,
    )
    print(f"Saved hardware validation figure -> {out_png}")


if __name__ == "__main__":
    main()
