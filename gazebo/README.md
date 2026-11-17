# Gazebo Harmonic (`gz-sim 8`) 3D Quadcopter Digital Twin & Browser Studio

Interactive 3D physics simulation and browser-based flight control studio (`http://localhost:8090`) that mirrors the exact C++ `control_step()` cascaded PID algorithm and hardware gains (`firmware/quad_fc/config.h`).

## 1. Environment Setup (`micromamba` / `conda`)

Create the `gzsim` environment containing Gazebo Harmonic (`libgz-sim8`) and its Python bindings:

```bash
micromamba env create -f environment.yml
# (or with conda: conda env create -f environment.yml)
```

## 2. Architecture & Files

- **`params.yaml`** — Single source of truth for physical plant constants (synced with `matlab/params.m`) and controller limits.
- **`generate_sdf.py`** — Programmatically generates the 3D SDF models and worlds:
  - `models/quadcopter/model.sdf` — F450 X-frame ($m = 0.818\text{ kg}$, $L = 0.225\text{ m}$, $I_{xx}=I_{yy}=0.0086\text{ kg}\cdot\text{m}^2$, $I_{zz}=0.0152\text{ kg}\cdot\text{m}^2$) with 4 rotor joints and `gz-sim-multicopter-motor-model-system` plugins.
  - `worlds/free_flight.sdf` — Unconstrained 6-DOF outdoor flight arena.
  - `worlds/tether_rig.sdf` — Laboratory spherical ball-joint tether rig matching the hardware test stand.
- **`fc_sim.py`** — Deterministic 200 Hz Python port of `firmware/quad_fc/quad_fc.ino` with hardware-calibrated non-idealities:
  - Automatically syncs PID gains from `firmware/quad_fc/config.h` on startup.
  - Models BNO085 SHTP + I2C fusion transport delay FIFO ($30\text{ ms}$ Euler delay, $15\text{ ms}$ gyro delay), asymmetric ESC spin-up ($60\text{ ms}$) vs. coast-down ($120\text{ ms}$), and 3D Dryden wind gusts.
  - Serves the interactive Three.js 3D Web Studio and 50 Hz telemetry strip-chart at `http://localhost:8090`.
- **`run_sim.sh`** — One-command launcher for interactive or automated step-response scenarios.

## 3. Running the Simulator

### Interactive 3D Browser Studio (Recommended)
```bash
cd gazebo
./run_sim.sh free_flight interactive --alt-hold
```
Then open **`http://localhost:8090`** in your browser to:
- Switch live between **Free Flight (6-DOF)** and **Tether Rig (3-DOF Attitude)** environments.
- Click **ARM**, raise **Throttle** (or enable **Altitude Hold**), and command **Roll / Pitch (±20°)** and **Yaw (-180° to +180° in 10° steps)**.
- Inject 3D **Wind Gust Forces** ($W_N,\ W_E,\ W_D$) and tune PID gains in real time.

### Automated Benchmark Scenarios
```bash
./run_sim.sh free_flight roll_steps --alt-hold
./run_sim.sh free_flight pitch_steps --alt-hold
./run_sim.sh free_flight yaw_steps --alt-hold
./run_sim.sh tether_rig hold
```
Telemetry logs are saved automatically to `gazebo/results/`.
