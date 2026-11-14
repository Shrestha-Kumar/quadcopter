# PS11 Quadcopter – Project Status & Plan (living document)

> Keep this file updated after every task. Other agents or models should read it first.
> Last updated: 2026-11-15 04:05

## 1. Goal
EE302 PS11: model, control, Simulink simulation and hardware validation of a quadcopter. Altitude hold plus roll, pitch and yaw tracking. See `PS11.pdf`. Deliverables: model, MATLAB/Simulink sim, design calcs, block diagram, experimental data, sim-vs-hardware comparison, report, demo. Safety: props off for initial checks, supervised tests, kill switch.

## 2. Hardware overview (from photos, disassembly of original flash, and live USB probing)
- Frame: F450-style, X configuration, mounted on a spring-loaded centre pole (tether rig) on a wooden base.
- **Physical Frame Landmark Convention:**
  - **FRONT (Nose):** USB port + black DC barrel jack end of the Arduino Mega 2560.
  - **BACK (Tail):** BNO085 IMU end of the Arduino Mega 2560.
- **Motors & ESC Status:** 4x A2212/13T 1000KV motors + 4 orange ESCs, powered from the PDB via XT60 — **ALL 4 WORKING**:
  - `M0` (`PIN_MOTOR_0 = 9`) = **Front-Right (`FR`, CCW)** (`+35 us` ESC deadband trim in firmware) — **WORKING**
  - `M1` (`PIN_MOTOR_1 = 8`) = **Front-Left (`FL`, CW)** — **WORKING**
  - `M2` (`PIN_MOTOR_2 = 7`) = **Rear-Left (`RL`, CCW)** — **WORKING**
  - `M3` (`PIN_MOTOR_3 = 6`) = **Rear-Right (`RR`, CW)** — **WORKING**
- **Final Hardcoded Cascaded PID Gains (`firmware/quad_fc/config.h`):**
  - Outer Angle Loop: `ANG_KP = 2.80f`, `YAW_ANG_KP = 2.60f`, `SP_SLEW_DPS = 22.0f`
  - Inner Rate Loop (Roll/Pitch): `RATE_KP = 2.60f`, `RATE_KI = 0.80f`, `RATE_KD = 0.150f` (computed on fresh 100 Hz BNO085 gyro samples, `D_LPF_ALPHA = 0.30f`, `RATE_I_LIMIT = 40.0f`)
  - Inner Rate Loop (Yaw): `YAWRATE_KP = 3.00f`, `YAWRATE_KI = 0.65f`, `YAWRATE_KD = 0.00f`, `YAW_I_LIMIT = 80.0f`, `MAX_YAW_OUT_US = 115.0f`
- MCU: Arduino Mega 2560 + ESP8266 combo board (Mega+WiFi R3, 8-switch DIP bank). USB: CH340 -> `/dev/ttyUSB0`, 115200 baud.
- **ESC jumper wiring:**
  - 4 signal wires on Mega PWM **Pins 6, 7, 8, 9** (`PIN_MOTOR_0..3 = 9, 8, 7, 6`).
  - 2 ground wires on the Mega POWER header (`GND` pins).
- **IMU hardware & firmware hardening (`Adafruit BNO085 9-DoF` at `0x4A`):**
  - Connected via 4 jumpers (`VIN`->`5V`, `GND`->`GND`, `SDA`->`20`, `SCL`->`21`).
  - Verified axis conventions in `firmware/quad_fc/config.h`: `IMU_SWAP_RP = 1`, `ROLL_SIGN = -1.0f`, `PITCH_SIGN = +1.0f`, `YAW_SIGN = +1.0f`, `GYRO_ROLL_SIGN = -1.0f`, `GYRO_PITCH_SIGN = +1.0f`, `GYRO_YAW_SIGN = +1.0f`.
  - Hardened `firmware/quad_fc/quad_fc.ino` against EMI/SHTP packet glitches during ESC switching: 100 kHz I2C + `25000 us` wire timeout, drains up to 3 SHTP events per tick, auto-recovers frozen BNO085 streams (`500 ms` armed / `1500 ms` disarmed).
  - Updated `tools/wifi_ground_station.py` with `±20°` Roll, Pitch, and Yaw sliders (`±10°` Yaw steps + quick step buttons) and `25 ms` serial TX pacing.

## 3. Original firmware (backed up, do not lose)
- `backup/flash_original.hex`, `.bin`, `eeprom_original.hex`, and `backup/README.md` (restore command).
- Restore anytime with:
  `avrdude -c wiring -p m2560 -P /dev/ttyUSB0 -b 115200 -D -U flash:w:backup/flash_original.hex:i`

## 4. Plan & Phase Status
| # | Phase | Status |
|---|-------|--------|
| 0 | Read PS, produce the plan (`implementation_plan.md`) | DONE |
| 1a | Photos reviewed, safety checks, USB connected, serial read | DONE |
| 1b | Backup of the original firmware (`backup/`) | DONE |
| 1c | Read candidate repo READMEs, pick a base (`reference/`) | DONE |
| 1d | Confirm pin mapping from close-up photos & disassembly (PWM 9, 8, 7, 6 + 2x GND) | DONE |
| 2 | Mathematical model (`matlab/params.m`, `quad_dynamics.m`, `linearize_quad.m`) | DONE |
| 3 | Control design: Cascaded PID + LQR+I comparison (`matlab/design_gains.m`, updated with `opts.hw = true`) | DONE |
| 4 | MATLAB/Simulink suite + Linux plots (`matlab/results/fig1..fig3.png` & `matlab/results/hw/fig1..fig3.png`) | DONE |
| 5 | Custom flight controller firmware (`firmware/quad_fc/quad_fc.ino`, `config.h`) + USB/Wi-Fi Ground Station (`tools/wifi_ground_station.py`) | DONE |
| 6 | BNO085 100 Hz quaternion + gyro verification, I2C recovery, all 4 ESCs verified & calibrated | DONE |
| 7 | Closed-loop Cascaded PID hardware tuning (`2.80 / 2.60 / 0.80 / 0.150`) + Gazebo 8 3D Browser Studio (`gazebo/`) | DONE |
| 8 | Final clean hardware log recording (`±5° / ±10°` Roll, Pitch, Yaw steps), regenerate `fig4_hardware_validation.png`, re-pack `quadcopter_matlab.zip`, and compile final report & presentation | NEXT (Tomorrow) |

## 5. Remaining Tasks for Tomorrow (Checklist So Nothing Is Forgotten)
1. **Record Final Clean Hardware Flight Log (`logs/flight_log_YYYYMMDD_HHMMSS.csv`):**
   - Start the USB Ground Station: `python3 tools/wifi_ground_station.py` and open `http://localhost:8080`.
   - Level the quadcopter on the rig and click **Zero Level (CAL)**.
   - Click **ARM (Slow Spin)**, raise the speed slider to hover test throttle (`~1340–1380 us`), and record in one clean session:
     - **Level hold (`0°`)** for ~8 seconds + gentle hand-push disturbance recovery on Roll, Pitch, and Yaw.
     - **Roll step sequence** using the new slider/buttons: `0° -> +5° -> 0° -> -5° -> 0° -> +10° -> 0° -> -10° -> 0°` (hold ~5 seconds at each angle).
     - **Pitch step sequence:** `0° -> +5° -> 0° -> -5° -> 0°`.
     - **Yaw step sequence (`±10°` steps):** `0° -> +10° -> +20° -> 0° -> -10° -> -20° -> 0°`.
   - Click **DISARM**.
2. **Generate Final Hardware Validation Plots (`fig4_hardware_validation.png` & Sim-vs-Hardware Overlay):**
   - Run `python3 tools/plot_hardware_log.py logs/<newest_flight_log>.csv` to produce the final publication-ready `matlab/results/fig4_hardware_validation.png` (and `.svg`).
   - Re-pack `quadcopter_matlab.zip` with the updated `matlab/*.m` files (`opts.hw = true`) and the final hardware CSV log.
3. **Final Presentation & Report (`PS11.pdf` Deliverables):**
   - Assemble the slide deck & report covering:
     1. System Architecture & Hardware Pinout (Mega 2560 + BNO085 100 Hz I2C + 4x A2212 1000KV).
     2. 6-DOF Nonlinear Model & Linearized Transfer Functions (`fig1_linear_stability.png`).
     3. Cascaded Angle-to-Rate PID vs. LQR+I Control Design (`fig2`, `fig3`, `metrics_summary.csv`).
     4. Gazebo 8 3D Digital Twin & Browser Studio (`gazebo/`, showing overshoot, undershoot, and wind compensation).
     5. Experimental Hardware Step-Response & Disturbance Rejection Results (`fig4_hardware_validation.png`).
