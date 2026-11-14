# Gazebo (gz-sim 8) quadcopter model

Env: `micromamba` env `gzsim` (python 3.10.22, pytorch 2.5.1 cuda12.4 cudnn9.1, numpy 2.2.6 = same as `dl`; libgz-sim8 8.10 + python bindings).

- `params.yaml` – single source of truth (plant from `matlab/params.m`, gains from `firmware/quad_fc/config.h`)
- `generate_sdf.py` – writes `models/quadcopter/model.sdf`, `worlds/free_flight.sdf`, `worlds/tether_rig.sdf`
- `fc_sim.py` – Python mirror of `control_step()` (angle->rate cascade, 100 Hz fresh-gyro D, slew, mixer), sim-time driven; automatically syncs controller gains from `firmware/quad_fc/config.h` on startup
- `run_sim.sh <world> <scenario> [--alt-hold]` – scenarios: `interactive`, `roll_steps`, `pitch_steps`, `yaw_steps`, `hold`; CSV -> `results/`
  - Interactive live control + 3D GUI:
    ```bash
    GUI=1 ./run_sim.sh free_flight interactive
    ```
    Then open `http://localhost:8090` to press **Run sim**, **ARM**, adjust **Throttle / Hover throttle / Altitude assist**, drag **Roll / Pitch / Yaw setpoints**, apply **Wind force**, tweak **Live gains**, or **Pause / Reset world**.

Caveats: plant numbers are the MATLAB *estimates* (not measured); the tether rig is a ball joint at the CG (the real
spring-pole stiffness/friction is unknown); firmware's +35 us M0 trim is not modelled (ideal ESCs); `--alt-hold` uses
ground-truth z (sim-only, the real FC has no altitude sensor).
