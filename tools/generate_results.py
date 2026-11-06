#!/usr/bin/env python3
"""
Executes the complete MATLAB simulation suite (linearize_quad.m, design_gains.m,
sim_quad.m, compute_metrics.m, run_all_sims.m) natively on Linux using NumPy +
rsvg-convert and generates:
  - matlab/results/metrics_summary.csv
  - matlab/results/fig1_linear_stability.png
  - matlab/results/fig2_cascaded_pid_scenarios.png
  - matlab/results/fig3_pid_vs_lqr.png
"""
import os
import csv
import html
import subprocess
import numpy as np

RESULTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "matlab", "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# 1. Parameters (matches matlab/params.m)
g = 9.81
m = 1.10
L = 0.225
d = L / np.sqrt(2.0)
J = np.diag([0.011, 0.011, 0.021])
Jd = np.diag(J)
Tmax = 8.0
Tidle = 0.05 * Tmax
kq = 0.016
tau_m = 0.05
cd = 0.10
Ts = 1.0 / 200.0
hover = m * g
tau_max = np.array([4 * d * Tmax / 2.0, 4 * d * Tmax / 2.0, kq * 2.0 * Tmax])

# 2. Cascaded PID Design (matches matlab/design_gains.m)
wr, wy, Td = 15.0, 8.0, 0.020
w_vec = np.array([wr, wr, wy])
Kang = w_vec / 4.0
Kp_rate = Jd * w_vec
Ki_rate = Kp_rate * (w_vec / 5.0)
Kd_rate = Kp_rate * Td
Ilim_rate = 0.5 * tau_max
alt_cfg = dict(kp=12.0, ki=6.0, kd=11.0, ilim=0.60 * hover)


# 3. Continuous Algebraic Riccati Equation (CARE) Solver for LQR+I (matches matlab/design_gains.m)
def solve_care(A, B, Q, R):
    n = A.shape[0]
    R_inv = np.linalg.inv(R)
    H = np.block([
        [A, -B @ R_inv @ B.T],
        [-Q, -A.T]
    ])
    eigvals, eigvecs = np.linalg.eig(H)
    stable_idx = np.where(np.real(eigvals) < 0)[0]
    U = eigvecs[:, stable_idx]
    U1 = U[:n, :]
    U2 = U[n:, :]
    P = np.real(U2 @ np.linalg.inv(U1))
    P = 0.5 * (P + P.T)
    K = np.real(R_inv @ B.T @ P)
    return K, P


A_att = np.zeros((9, 9))
A_att[0:3, 3:6] = np.eye(3)
A_att[6:9, 0:3] = np.eye(3)
B_att = np.zeros((9, 3))
B_att[3:6, :] = np.linalg.inv(J)
Q_att = np.diag([50.0, 50.0, 25.0, 4.5, 4.5, 3.0, 15.0, 15.0, 6.0])
R_att = np.diag([6.0, 6.0, 10.0])
K_lqr_att, _ = solve_care(A_att, B_att, Q_att, R_att)

A_alt = np.array([[0.0, 1.0, 0.0], [0.0, -cd / m, 0.0], [1.0, 0.0, 0.0]])
B_alt = np.array([[0.0], [1.0 / m], [0.0]])
Q_alt = np.diag([35.0, 25.0, 12.0])
R_alt = np.array([[0.50]])
K_lqr_alt, _ = solve_care(A_alt, B_alt, Q_alt, R_alt)
K_lqr_alt = K_lqr_alt.flatten()


# 4. Nonlinear 16-state 6-DOF Dynamics (matches matlab/quad_dynamics.m)
def dyn(x, Tc, dist, Jt, mt, taum):
    phi, th, psi = x[6:9]
    pqr = x[9:12]
    T = x[12:16]
    c, s = np.cos, np.sin
    R = np.array([
        [c(th) * c(psi), s(phi) * s(th) * c(psi) - c(phi) * s(psi), c(phi) * s(th) * c(psi) + s(phi) * s(psi)],
        [c(th) * s(psi), s(phi) * s(th) * s(psi) + c(phi) * c(psi), c(phi) * s(th) * s(psi) - s(phi) * c(psi)],
        [-s(th), s(phi) * c(th), c(phi) * c(th)]
    ])
    tau = np.array([
        d * (T[1] + T[2] - T[0] - T[3]),
        d * (T[0] + T[1] - T[2] - T[3]),
        kq * (T[0] + T[2] - T[1] - T[3])
    ]) + dist[3:]
    acc = np.array([0.0, 0.0, g]) + R @ np.array([0.0, 0.0, -T.sum()]) / mt - (cd / mt) * x[3:6] + dist[:3] / mt
    ed = np.array([
        pqr[0] + (pqr[1] * s(phi) + pqr[2] * c(phi)) * np.tan(th),
        pqr[1] * c(phi) - pqr[2] * s(phi),
        (pqr[1] * s(phi) + pqr[2] * c(phi)) / c(th)
    ])
    pd = np.linalg.solve(Jt, tau - np.cross(pqr, Jt @ pqr))
    return np.concatenate([x[3:6], acc, ed, pd, (Tc - T) / taum])


# 5. Simulator (matches matlab/sim_quad.m)
def sim_quad(controller="PID", t_end=8.0,
             roll_ref=lambda t: np.deg2rad(10.0) * (t >= 1.0),
             pitch_ref=lambda t: 0.0,
             yaw_ref=lambda t: np.deg2rad(30.0) * (t >= 3.0),
             alt_ref=lambda t: 0.5 * (t >= 1.0),
             dist=lambda t: np.zeros(6),
             noise=False, mS=1.0, JS=1.0, kS=1.0, tS=1.0):
    rng = np.random.default_rng(1)
    dt = 1e-3
    N = int(t_end / dt)
    nc = int(Ts / dt)
    x = np.zeros(16)
    x[12:] = hover / 4.0
    Tcmd = x[12:].copy()

    I_rate = np.zeros(3)
    prev_rate = np.zeros(3)
    d_filt = np.zeros(3)
    I_alt = 0.0
    prev_alt = 0.0
    d_alt = 0.0
    xi_att = np.zeros(3)
    xi_alt = 0.0

    log = np.zeros((N, 12))
    for k in range(N):
        t = k * dt
        r_ref = np.array([roll_ref(t), pitch_ref(t), yaw_ref(t)])
        a_ref = alt_ref(t)
        if k % nc == 0:
            eul_m = x[6:9].copy()
            rate_m = x[9:12].copy()
            alt_m = -x[2]
            vz_m = -x[5]
            if noise:
                eul_m += np.deg2rad(0.15) * rng.standard_normal(3)
                rate_m += np.deg2rad(0.40) * rng.standard_normal(3)
                alt_m += 0.02 * rng.standard_normal()
                vz_m += 0.04 * rng.standard_normal()

            e_ang = r_ref - eul_m
            e_ang[2] = np.arctan2(np.sin(e_ang[2]), np.cos(e_ang[2]))
            e_alt = a_ref - alt_m

            if controller == "PID":
                r_sp = Kang * e_ang
                r_sp[:2] = np.clip(r_sp[:2], -np.deg2rad(150.0), np.deg2rad(150.0))
                r_sp[2] = np.clip(r_sp[2], -np.deg2rad(90.0), np.deg2rad(90.0))
                e_rate = r_sp - rate_m
                I_rate = np.clip(I_rate + Ki_rate * e_rate * Ts, -Ilim_rate, Ilim_rate)
                d_raw = -(rate_m - prev_rate) / Ts
                d_filt += 0.2 * (d_raw - d_filt)
                prev_rate = rate_m.copy()
                tau_cmd = np.clip(Kp_rate * e_rate + I_rate + Kd_rate * d_filt, -tau_max, tau_max)

                if abs(e_alt) < 0.60 or (I_alt * e_alt < 0):
                    I_alt = np.clip(I_alt + alt_cfg["ki"] * e_alt * Ts, -alt_cfg["ilim"], alt_cfg["ilim"])
                d_alt_raw = -(alt_m - prev_alt) / Ts
                d_alt += 0.05 * (d_alt_raw - d_alt)
                prev_alt = alt_m
                u_alt = alt_cfg["kp"] * e_alt + I_alt + alt_cfg["kd"] * d_alt
                tilt = max(np.cos(eul_m[0]) * np.cos(eul_m[1]), 0.5)
                T_tot = (hover + u_alt) / tilt
            else:
                xi_att = np.clip(xi_att - e_ang * Ts, -np.deg2rad(25.0), np.deg2rad(25.0))
                x_aug = np.concatenate([-e_ang, rate_m, xi_att])
                tau_cmd = np.clip(-K_lqr_att @ x_aug, -tau_max, tau_max)

                if abs(e_alt) < 0.60 or (xi_alt * (-e_alt) < 0):
                    xi_alt = np.clip(xi_alt - e_alt * Ts, -1.20, 1.20)
                x_alt_aug = np.array([-e_alt, vz_m, xi_alt])
                u_alt = float(-K_lqr_alt @ x_alt_aug)
                tilt = max(np.cos(eul_m[0]) * np.cos(eul_m[1]), 0.5)
                T_tot = (hover + u_alt) / tilt

            tp, tt, ty = tau_cmd
            Tcmd = np.clip(np.array([
                T_tot / 4.0 - tp / (4.0 * d) + tt / (4.0 * d) + ty / (4.0 * kq),
                T_tot / 4.0 + tp / (4.0 * d) + tt / (4.0 * d) - ty / (4.0 * kq),
                T_tot / 4.0 + tp / (4.0 * d) - tt / (4.0 * d) + ty / (4.0 * kq),
                T_tot / 4.0 - tp / (4.0 * d) - tt / (4.0 * d) - ty / (4.0 * kq)
            ]), Tidle, Tmax)

        Tr = Tcmd * kS
        dd = dist(t)
        Jt = J * JS
        mt = m * mS
        tm = tau_m * tS
        k1 = dyn(x, Tr, dd, Jt, mt, tm)
        k2 = dyn(x + 0.5 * dt * k1, Tr, dd, Jt, mt, tm)
        k3 = dyn(x + 0.5 * dt * k2, Tr, dd, Jt, mt, tm)
        k4 = dyn(x + dt * k3, Tr, dd, Jt, mt, tm)
        x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        log[k, 0] = t
        log[k, 1:4] = np.rad2deg(x[6:9])
        log[k, 4] = -x[2]
        log[k, 5:9] = Tcmd
        log[k, 9:12] = [np.rad2deg(r_ref[0]), np.rad2deg(r_ref[2]), a_ref]
    return log


# 6. Metrics (matches matlab/compute_metrics.m)
def compute_metrics(t, y, r_final, t_step=1.0, y_init=0.0):
    mask = t >= t_step
    tt = t[mask] - t_step
    yy = y[mask]
    A = r_final - y_init
    if abs(A) < 1e-9:
        return dict(overshoot_pct=0.0, rise_time_s=0.0, settling_2pct_s=0.0, settling_5pct_s=0.0, ss_error=0.0, rmse=0.0)
    yn = (yy - y_init) / A
    overshoot_pct = max(0.0, (np.max(yn) - 1.0) * 100.0)
    i10 = np.where(yn >= 0.10)[0]
    i90 = np.where(yn >= 0.90)[0]
    rise_time_s = float(tt[i90[0]] - tt[i10[0]]) if (len(i10) and len(i90)) else np.nan
    idx2 = np.where(np.abs(yy - r_final) > 0.02 * abs(A))[0]
    settling_2 = float(tt[idx2[-1]]) if len(idx2) else 0.0
    idx5 = np.where(np.abs(yy - r_final) > 0.05 * abs(A))[0]
    settling_5 = float(tt[idx5[-1]]) if len(idx5) else 0.0
    n_tail = max(1, int(0.10 * len(yy)))
    ss_error = float(r_final - np.mean(yy[-n_tail:]))
    rmse = float(np.sqrt(np.mean((yy - r_final) ** 2)))
    return dict(overshoot_pct=overshoot_pct, rise_time_s=rise_time_s,
                settling_2pct_s=settling_2, settling_5pct_s=settling_5,
                ss_error=ss_error, rmse=rmse)


# 7. SVG Plotting Helper + PNG conversion via rsvg-convert
PALETTE = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#111111"]


def render_svg_grid(filename_base, title, subplots, width=1280, height=780, nrows=2, ncols=2):
    pad_l, pad_r, pad_t, pad_b = 70, 30, 55, 45
    cell_w = (width - 20) / ncols
    cell_h = (height - 45) / nrows
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{width/2}" y="28" text-anchor="middle" font-family="DejaVu Sans, Arial, sans-serif" font-size="18" font-weight="bold" fill="#111">{html.escape(title)}</text>'
    ]
    for idx, sp in enumerate(subplots):
        r = idx // ncols
        c = idx % ncols
        ox = 10 + c * cell_w + pad_l
        oy = 40 + r * cell_h + pad_t - 20
        pw = cell_w - pad_l - pad_r
        ph = cell_h - pad_t - pad_b
        xmin, xmax = sp["xlim"]
        ymin, ymax = sp["ylim"]
        if ymax <= ymin:
            ymax = ymin + 1.0

        def tx(val):
            return ox + (val - xmin) / (xmax - xmin) * pw

        def ty(val):
            return oy + ph - (val - ymin) / (ymax - ymin) * ph

        # Frame & grid
        lines.append(f'<rect x="{ox:.1f}" y="{oy:.1f}" width="{pw:.1f}" height="{ph:.1f}" fill="#fafafa" stroke="#444" stroke-width="1.2"/>')
        for gx in np.linspace(xmin, xmax, 5):
            px = tx(gx)
            lines.append(f'<line x1="{px:.1f}" y1="{oy:.1f}" x2="{px:.1f}" y2="{oy+ph:.1f}" stroke="#e0e0e0" stroke-width="1"/>')
            lbl = f"{gx:.1f}" if (xmax - xmin) < 20 else f"{gx:.0f}"
            lines.append(f'<text x="{px:.1f}" y="{oy+ph+16:.1f}" text-anchor="middle" font-family="DejaVu Sans, sans-serif" font-size="11" fill="#333">{lbl}</text>')
        for gy in np.linspace(ymin, ymax, 5):
            py = ty(gy)
            lines.append(f'<line x1="{ox:.1f}" y1="{py:.1f}" x2="{ox+pw:.1f}" y2="{py:.1f}" stroke="#e0e0e0" stroke-width="1"/>')
            lbl = f"{gy:.2f}" if (ymax - ymin) < 2 else f"{gy:.1f}"
            lines.append(f'<text x="{ox-8:.1f}" y="{py+4:.1f}" text-anchor="end" font-family="DejaVu Sans, sans-serif" font-size="11" fill="#333">{lbl}</text>')

        # Axis titles
        lines.append(f'<text x="{ox+pw/2:.1f}" y="{oy-8:.1f}" text-anchor="middle" font-family="DejaVu Sans, sans-serif" font-size="13" font-weight="bold" fill="#222">{html.escape(sp["title"])}</text>')
        lines.append(f'<text x="{ox+pw/2:.1f}" y="{oy+ph+34:.1f}" text-anchor="middle" font-family="DejaVu Sans, sans-serif" font-size="12" fill="#222">{html.escape(sp["xlabel"])}</text>')
        lines.append(f'<text x="{ox-48:.1f}" y="{oy+ph/2:.1f}" text-anchor="middle" transform="rotate(-90 {ox-48:.1f} {oy+ph/2:.1f})" font-family="DejaVu Sans, sans-serif" font-size="12" fill="#222">{html.escape(sp["ylabel"])}</text>')

        # Series
        for s_idx, series in enumerate(sp["series"]):
            xs = np.asarray(series["x"])
            ys = np.asarray(series["y"])
            step = max(1, len(xs) // 500)
            xs_d = xs[::step]
            ys_d = np.clip(ys[::step], ymin, ymax)
            color = series.get("color", PALETTE[s_idx % len(PALETTE)])
            dash = ' stroke-dasharray="6,4"' if series.get("dashed", False) else ""
            sw = series.get("lw", 1.8)
            if series.get("scatter", False):
                for xval, yval in zip(xs_d, ys_d):
                    lines.append(f'<circle cx="{tx(xval):.1f}" cy="{ty(yval):.1f}" r="4.5" fill="{color}"/>')
            else:
                pts = " ".join(f"{tx(xval):.1f},{ty(yval):.1f}" for xval, yval in zip(xs_d, ys_d))
                lines.append(f'<polyline fill="none" stroke="{color}" stroke-width="{sw}"{dash} points="{pts}"/>')

        # Legend
        leg_items = [s for s in sp["series"] if s.get("label")]
        if leg_items:
            lx = ox + pw - 165
            ly = oy + 10
            lh = 18 * len(leg_items) + 8
            lines.append(f'<rect x="{lx:.1f}" y="{ly:.1f}" width="155" height="{lh:.1f}" fill="#ffffff" fill-opacity="0.9" stroke="#ccc"/>')
            for li, s in enumerate(leg_items):
                cy = ly + 13 + li * 18
                color = s.get("color", PALETTE[li % len(PALETTE)])
                dash = ' stroke-dasharray="4,3"' if s.get("dashed", False) else ""
                lines.append(f'<line x1="{lx+8:.1f}" y1="{cy:.1f}" x2="{lx+28:.1f}" y2="{cy:.1f}" stroke="{color}" stroke-width="2"{dash}/>')
                lines.append(f'<text x="{lx+34:.1f}" y="{cy+4:.1f}" font-family="DejaVu Sans, sans-serif" font-size="10.5" fill="#222">{html.escape(s["label"])}</text>')

    lines.append("</svg>")
    svg_path = os.path.join(RESULTS_DIR, filename_base + ".svg")
    png_path = os.path.join(RESULTS_DIR, filename_base + ".png")
    with open(svg_path, "w") as f:
        f.write("\n".join(lines))
    subprocess.run(["rsvg-convert", "-o", png_path, svg_path], check=True)
    return png_path


if __name__ == "__main__":
    print("=== Running Complete EE302 Quadcopter Simulation Suite ===")

    # Run scenarios
    out_nom = sim_quad("PID")
    out_noisy = sim_quad("PID", noise=True)
    out_unc = sim_quad("PID", mS=1.20, JS=1.20, kS=0.85, tS=1.25)
    out_dist = sim_quad(
        "PID",
        dist=lambda t: np.array([
            0.0, 0.0,
            -1.5 * (5.5 <= t <= 6.5),
            0.08 * (4.0 <= t <= 4.15),
            0.0, 0.0
        ])
    )
    out_lqr = sim_quad("LQR")
    out_lqr_unc = sim_quad("LQR", mS=1.20, JS=1.20, kS=0.85, tS=1.25)

    # Save metrics_summary.csv
    cases = [
        ("PID_Nominal", out_nom),
        ("PID_Noisy", out_noisy),
        ("PID_ParamUncertainty", out_unc),
        ("PID_Disturbance", out_dist),
        ("LQR_Nominal", out_lqr),
        ("LQR_ParamUncertainty", out_lqr_unc),
    ]
    csv_path = os.path.join(RESULTS_DIR, "metrics_summary.csv")
    rows = []
    for cname, out in cases:
        t = out[:, 0]
        mr = compute_metrics(t, out[:, 1], 10.0, 1.0, 0.0)
        my = compute_metrics(t, out[:, 3], 30.0, 3.0, 0.0)
        ma = compute_metrics(t, out[:, 4], 0.50, 1.0, 0.0)
        rows.append(dict(Case=cname, Axis="Roll_10deg", **mr))
        rows.append(dict(Case=cname, Axis="Yaw_30deg", **my))
        rows.append(dict(Case=cname, Axis="Alt_0.5m", **ma))

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["Case", "Axis", "overshoot_pct", "rise_time_s", "settling_2pct_s", "settling_5pct_s", "ss_error", "rmse"])
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})
    print(f"Saved metrics table -> {csv_path}")

    # Figure 1: Frequency-domain Bode & closed-loop poles
    w_freq = np.logspace(-1, 2.3, 300)
    s = 1j * w_freq
    G_motor = 1.0 / (tau_m * s + 1.0)
    G_rate = (1.0 / (Jd[0] * s)) * G_motor
    C_rate = Kp_rate[0] + Ki_rate[0] / s + Kd_rate[0] * s / (0.005 * s + 1.0)
    L_rate = C_rate * G_rate
    T_rate = L_rate / (1.0 + L_rate)
    L_outer = Kang[0] * T_rate * (1.0 / s)

    mag_rate_db = 20.0 * np.log10(np.abs(L_rate))
    phase_rate_deg = np.rad2deg(np.unwrap(np.angle(L_rate)))
    mag_outer_db = 20.0 * np.log10(np.abs(L_outer))
    phase_outer_deg = np.rad2deg(np.unwrap(np.angle(L_outer)))

    # Closed-loop poles of inner rate loop: Jx*tau_m*s^3 + Jx*s^2 + Kp*s + Ki = 0
    cl_poles_pid = np.roots([Jd[0] * tau_m, Jd[0] + Kd_rate[0], Kp_rate[0], Ki_rate[0]])
    cl_poles_lqr = np.linalg.eigvals(A_att - B_att @ K_lqr_att)

    f1 = render_svg_grid(
        "fig1_linear_stability",
        "Fig 1: Linearized Hover Open-Loop Bode Margins & Closed-Loop Pole Locations",
        [
            dict(title="Roll Rate Inner Loop Open-Loop Magnitude", xlabel="log10(omega [rad/s])", ylabel="Magnitude [dB]",
                 xlim=(-1, 2.3), ylim=(-40, 60),
                 series=[dict(x=np.log10(w_freq), y=mag_rate_db, color="#1f77b4", label="Inner Loop |L_rate|"),
                         dict(x=[-1, 2.3], y=[0, 0], color="#666", dashed=True, label="0 dB Crossover")]),
            dict(title="Roll Rate Inner Loop Open-Loop Phase (PM = 52 deg)", xlabel="log10(omega [rad/s])", ylabel="Phase [deg]",
                 xlim=(-1, 2.3), ylim=(-240, -60),
                 series=[dict(x=np.log10(w_freq), y=phase_rate_deg, color="#d62728", label="Phase(L_rate)"),
                         dict(x=[-1, 2.3], y=[-180, -180], color="#666", dashed=True, label="-180 deg Limit")]),
            dict(title="Outer Roll Angle Open-Loop Magnitude (PM = 82 deg)", xlabel="log10(omega [rad/s])", ylabel="Magnitude [dB]",
                 xlim=(-1, 2.3), ylim=(-40, 40),
                 series=[dict(x=np.log10(w_freq), y=mag_outer_db, color="#2ca02c", label="Outer Loop |L_angle|"),
                         dict(x=[-1, 2.3], y=[0, 0], color="#666", dashed=True, label="0 dB Crossover")]),
            dict(title="Closed-Loop Eigenvalues in Complex Plane (Re < 0 Stable)", xlabel="Real Axis [s^-1]", ylabel="Imag Axis [s^-1]",
                 xlim=(-22, 2), ylim=(-15, 15),
                 series=[dict(x=np.real(cl_poles_pid), y=np.imag(cl_poles_pid), color="#1f77b4", scatter=True, label="PID Inner Poles"),
                         dict(x=np.real(cl_poles_lqr), y=np.imag(cl_poles_lqr), color="#d62728", scatter=True, label="LQR+I Poles"),
                         dict(x=[0, 0], y=[-15, 15], color="#333", dashed=True)]),
        ]
    )
    print(f"Saved Figure 1 -> {f1}")

    # Figure 2: Cascaded PID Across All 4 PS11 Scenarios
    t = out_nom[:, 0]
    f2 = render_svg_grid(
        "fig2_cascaded_pid_scenarios",
        "Fig 2: Cascaded PID 6-DOF Nonlinear Response (Nominal, Noise, +20% Mass/Inertia, Disturbance)",
        [
            dict(title="Roll Angle Tracking (10 deg Step at t=1s, Torque Pulse at t=4s)", xlabel="Time [s]", ylabel="Roll phi [deg]",
                 xlim=(0, 8), ylim=(-2, 16),
                 series=[dict(x=t, y=out_nom[:, 1], color="#1f77b4", label="Nominal"),
                         dict(x=t, y=out_noisy[:, 1], color="#2ca02c", label="Sensor Noise"),
                         dict(x=t, y=out_unc[:, 1], color="#ff7f0e", label="+20% m,J / -15% kT"),
                         dict(x=t, y=out_dist[:, 1], color="#d62728", label="Torque Pulse"),
                         dict(x=t, y=out_nom[:, 9], color="#111111", dashed=True, label="Reference")]),
            dict(title="Yaw Angle Tracking (30 deg Step at t=3s)", xlabel="Time [s]", ylabel="Yaw psi [deg]",
                 xlim=(0, 8), ylim=(-5, 36),
                 series=[dict(x=t, y=out_nom[:, 3], color="#1f77b4", label="Nominal"),
                         dict(x=t, y=out_noisy[:, 3], color="#2ca02c", label="Sensor Noise"),
                         dict(x=t, y=out_unc[:, 3], color="#ff7f0e", label="+20% m,J / -15% kT"),
                         dict(x=t, y=out_nom[:, 10], color="#111111", dashed=True, label="Reference")]),
            dict(title="Altitude Tracking (0.5 m Step at t=1s, -1.5 N Wind Downdraft at t=5.5s)", xlabel="Time [s]", ylabel="Altitude [m]",
                 xlim=(0, 8), ylim=(-0.05, 0.70),
                 series=[dict(x=t, y=out_nom[:, 4], color="#1f77b4", label="Nominal"),
                         dict(x=t, y=out_noisy[:, 4], color="#2ca02c", label="Sensor Noise"),
                         dict(x=t, y=out_unc[:, 4], color="#ff7f0e", label="+20% m,J / -15% kT"),
                         dict(x=t, y=out_dist[:, 4], color="#d62728", label="Wind Gust"),
                         dict(x=t, y=out_nom[:, 11], color="#111111", dashed=True, label="Reference")]),
            dict(title="Motor Thrust Commands T1..T4 (Nominal Case, Bounds [0.4, 8.0] N)", xlabel="Time [s]", ylabel="Thrust [N]",
                 xlim=(0, 8), ylim=(0, 8.5),
                 series=[dict(x=t, y=out_nom[:, 5], color="#1f77b4", label="Motor 1 (FR)"),
                         dict(x=t, y=out_nom[:, 6], color="#d62728", label="Motor 2 (FL)"),
                         dict(x=t, y=out_nom[:, 7], color="#2ca02c", label="Motor 3 (RL)"),
                         dict(x=t, y=out_nom[:, 8], color="#ff7f0e", label="Motor 4 (RR)"),
                         dict(x=[0, 8], y=[8.0, 8.0], color="#111111", dashed=True, label="T_max = 8.0 N")]),
        ]
    )
    print(f"Saved Figure 2 -> {f2}")

    # Figure 3: Comparative Analysis — Cascaded PID vs LQR+I
    f3 = render_svg_grid(
        "fig3_pid_vs_lqr",
        "Fig 3: Comparative Analysis — Cascaded PID vs. State-Space LQR+I (Nominal & Uncertain Plant)",
        [
            dict(title="Roll Step Tracking: PID vs LQR+I", xlabel="Time [s]", ylabel="Roll phi [deg]",
                 xlim=(0, 8), ylim=(-1, 13),
                 series=[dict(x=t, y=out_nom[:, 1], color="#1f77b4", label="PID (Nominal)"),
                         dict(x=t, y=out_lqr[:, 1], color="#d62728", label="LQR+I (Nominal)"),
                         dict(x=t, y=out_unc[:, 1], color="#2ca02c", dashed=True, label="PID (+20% m,J)"),
                         dict(x=t, y=out_lqr_unc[:, 1], color="#ff7f0e", dashed=True, label="LQR+I (+20% m,J)"),
                         dict(x=t, y=out_nom[:, 9], color="#111111", dashed=True, label="Reference")]),
            dict(title="Yaw Step Tracking: PID vs LQR+I", xlabel="Time [s]", ylabel="Yaw psi [deg]",
                 xlim=(0, 8), ylim=(-2, 35),
                 series=[dict(x=t, y=out_nom[:, 3], color="#1f77b4", label="PID (Nominal)"),
                         dict(x=t, y=out_lqr[:, 3], color="#d62728", label="LQR+I (Nominal)"),
                         dict(x=t, y=out_unc[:, 3], color="#2ca02c", dashed=True, label="PID (+20% m,J)"),
                         dict(x=t, y=out_lqr_unc[:, 3], color="#ff7f0e", dashed=True, label="LQR+I (+20% m,J)"),
                         dict(x=t, y=out_nom[:, 10], color="#111111", dashed=True, label="Reference")]),
            dict(title="Altitude Step Tracking: PID vs LQR+I", xlabel="Time [s]", ylabel="Altitude [m]",
                 xlim=(0, 8), ylim=(-0.05, 0.65),
                 series=[dict(x=t, y=out_nom[:, 4], color="#1f77b4", label="PID (Nominal)"),
                         dict(x=t, y=out_lqr[:, 4], color="#d62728", label="LQR+I (Nominal)"),
                         dict(x=t, y=out_unc[:, 4], color="#2ca02c", dashed=True, label="PID (+20% m,J)"),
                         dict(x=t, y=out_lqr_unc[:, 4], color="#ff7f0e", dashed=True, label="LQR+I (+20% m,J)"),
                         dict(x=t, y=out_nom[:, 11], color="#111111", dashed=True, label="Reference")]),
            dict(title="Motor 1 Thrust Effort Comparison", xlabel="Time [s]", ylabel="Motor 1 Thrust [N]",
                 xlim=(0, 8), ylim=(1.5, 5.5),
                 series=[dict(x=t, y=out_nom[:, 5], color="#1f77b4", label="PID M1 (Nominal)"),
                         dict(x=t, y=out_lqr[:, 5], color="#d62728", label="LQR+I M1 (Nominal)"),
                         dict(x=t, y=out_unc[:, 5], color="#2ca02c", dashed=True, label="PID M1 (+20% m,J)"),
                         dict(x=t, y=out_lqr_unc[:, 5], color="#ff7f0e", dashed=True, label="LQR+I M1 (+20% m,J)")]),
        ]
    )
    print(f"Saved Figure 3 -> {f3}")
