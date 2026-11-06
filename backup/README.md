# Backup of original board firmware (taken 2026-11-06)

Board: Mega 2560 + ESP8266 (Mega+WiFi R3), CH340 USB-serial, /dev/ttyUSB0.

Files:
- `flash_original.hex` – full flash read (app + bootloader). sha256 62dafba4...13b12a2
- `flash_original.bin` – same, as a raw binary image
- `eeprom_original.hex` – EEPROM read. sha256 e6804337...3829307a... (see `sha256sum`)

## Restore the original firmware
```
avrdude -c wiring -p m2560 -P /dev/ttyUSB0 -b 115200 -D -U flash:w:backup/flash_original.hex:i
```
The bootloader is "Arduino explorer stk500V2 by MLS" in the top flash region. The `-D` flag skips the chip erase.
Flashing through the bootloader never overwrites the bootloader itself.

## Findings (strings in the original app, ~40 KB)
- Serial banner `=== WIFI HOVER TEST READY ===` at 115200 baud
- Commands: `ARM`, `DISARM`, `UP`, `DOWN`, `THR:<value>`, `ESC_TEST`, `IMU_TEST`, `STOP` (emergency stop)
- Replies: `ACK:ARM`, `ACK:DISARM`, `ACK:UP`, `ACK:DOWN`, `Mode: ESC TEST`, `Mode: IMU TEST`, `Mode: IDLE`
- IMU calibration: "Calibrating gyro... Keep IMU still" (gyro calibration at start-up)
- Roll/Pitch/Yaw printout
- MPU6050 library strings are also present (possibly leftover test code; the board carries a BNO055)
- No PID gains are visible as text. The code looks like an ESC and IMU test with manual throttle over Wi-Fi/serial, not a closed-loop controller.
