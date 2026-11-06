# PS11 Quadcopter – Project Status & Plan (living document)

> Keep this file updated after every task. Other agents or models should read it first.
> Last updated: 2026-11-06 16:00

## 1. Goal
EE302 PS11: model, control, Simulink simulation and hardware validation of a quadcopter. Altitude hold plus roll, pitch and yaw tracking. See `PS11.pdf`. Deliverables: model, MATLAB/Simulink sim, design calcs, block diagram, experimental data, sim-vs-hardware comparison, report, demo. Safety: props off for initial checks, supervised tests, kill switch.

## 2. Hardware overview (from photos, disassembly of original flash, and live USB probing)
- Frame: F450-style, X configuration, mounted on a spring-loaded centre pole (tether rig) on a wooden base.
- Motors: 4x A2212/13T 1000KV. ESCs: 4 (orange units), powered from the PDB, fed by XT60 (battery or bench DC supply).
- MCU: Arduino Mega 2560 + ESP8266 combo board (Mega+WiFi R3, 8-switch DIP bank). USB: CH340 -> `/dev/ttyUSB0`, 115200 baud.
- **ESC jumper wiring (confirmed from photos AND verified in the disassembly of the original firmware at `0x121e..0x124a`):**
  - 4 signal wires on Mega PWM **Pins 6, 7, 8, 9** (`PIN_MOTOR_0..3 = 9, 8, 7, 6`).
  - 2 ground wires on the Mega POWER header (`GND` pins).
- **IMU hardware finding (IMPORTANT):**
  - The board zip-tied on top of the Mega is silkscreened **`Adafruit BNO085 9-DoF`** (not BNO055).
  - Live I2C scan (`SCAN` command on `/dev/ttyUSB0`) currently returns `I2C_SCAN: none` because the BNO085 breakout is only zip-tied onto the Mega and **has no power (`VIN`/`GND`) or I2C (`SDA`/`SCL` -> Mega pins 20/21) wires connected yet** (the STEMMA QT white sockets on the BNO085 are empty and its header pins are unsoldered).
  - Disassembly of the original flash (`backup/flash_original.bin`) confirmed that the original sketch (`0x0000..0x1a10`) was *only* an ESC + Serial3 Wi-Fi hover test (`Servo.attach(6..9)` + `ARM/DISARM/UP/DOWN/THR:`) and never initialized any IMU! The strings from `0x2000..0x4212` were leftover flash pages from an earlier MPU6050 sketch that used a larger binary.
  - Our new `quad_fc.ino` now auto-detects **BNO085 (`0x4A`/`0x4B`)**, **BNO055 (`0x28`/`0x29`)**, and **MPU6050 (`0x68`)** on I2C (`SDA`=pin 20, `SCL`=pin 21). As soon as 4 jumpers (`VIN`->`5V`, `GND`->`GND`, `SDA`->`20`, `SCL`->`21`) or a STEMMA QT 4-pin cable are connected to the BNO085, sending `SCAN` initializes it immediately.

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
| 3 | Control design: Cascaded PID + LQR+I comparison (`matlab/design_gains.m`) | DONE |
| 4 | MATLAB/Simulink suite (`sim_quad.m`, `compute_metrics.m`, `run_all_sims.m`, `build_simulink_model.m`, `analyse_flight_logs.m`), `quadcopter_matlab.zip` for MATLAB Online, and Linux simulation + plots (`tools/generate_results.py` -> `matlab/results/fig1..fig3.png` & `metrics_summary.csv`) | DONE |
| 5 | Custom flight controller firmware (`firmware/quad_fc/quad_fc.ino`, `config.h`) with dual USB + onboard ESP8266 Wi-Fi (`Serial3`) + Wi-Fi Ground Station (`tools/wifi_ground_station.py`) flashed & verified on Mega over `/dev/ttyUSB0` | DONE |
| 6a | Connect 4 I2C jumpers to BNO085 (`0x4A`), verify live 100 Hz quaternion + gyro telemetry (`imu=ok`) and `localhost:8080` Ground Station | DONE |
| 6b | Verify BNO085 Roll/Pitch/Yaw axis signs (`IMU_SWAP_RP=1`, `ROLL_SIGN=-1`, `PITCH_SIGN=+1` flashed) | DONE |
| 6c | Connect ESC wires (pins `9, 8, 7, 6` + `GND`) and run props-off `ESC_TEST` | NEXT |
| 7 | Fine tuning & experimental data logging | TODO |
| 8 | Report, presentation, demo | TODO |
