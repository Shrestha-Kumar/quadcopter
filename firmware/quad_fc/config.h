// config.h - all tunable parameters for the quad_fc firmware.
// Target: Arduino Mega 2560 + Adafruit BNO055 (I2C) + 4 ESC (A2212 1000KV).
// Layout inspired by DanieleVeri/quadcopter (angle -> rate cascade, config.h).
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------- Motors
// !!! UNVERIFIED PIN MAP (read from photo, 2026-11-06). Confirm with a multimeter
// !!! BEFORE connecting the ESCs: see PROJECT_STATUS.md section 9.
// Motor index convention (looking from above, nose = front):
//   0 = front-right, 1 = front-left, 2 = rear-left, 3 = rear-right
#define PIN_MOTOR_0 9
#define PIN_MOTOR_1 8
#define PIN_MOTOR_2 7
#define PIN_MOTOR_3 6

#define ESC_MIN_US      1000   // disarmed / zero throttle
#define ESC_MAX_US      2000
#define MOTOR_IDLE_US   1100   // minimum output while armed (motors just spinning)
// Hard cap on the throttle command. Raise gradually during tests on the rig.
#define THR_MAX_US      1400
#define THR_ARM_MAX_US  1050   // throttle must be below this to arm

// Mixer: motor_i = thr + MIX_R[i]*roll + MIX_P[i]*pitch + MIX_Y[i]*yaw
//   +roll  = right side goes DOWN  (speed up left motors)
//   +pitch = nose goes UP          (speed up front motors)
//   +yaw   = counter-clockwise seen from above (speed up CW-spinning motors)
// Assumes the props: FL and RR spin CW, FR and RL spin CCW (standard).
// Verify direction with the props OFF (ESC_TEST) and flip signs here if needed.
static const int8_t MIX_R[4] = { -1, +1, +1, -1 };
static const int8_t MIX_P[4] = { +1, +1, -1, -1 };
static const int8_t MIX_Y[4] = { -1, +1, -1, +1 };

// ---------------------------------------------------------------- IMU
// Auto-detects IMU on I2C (pins 20=SDA, 21=SCL):
//   - Adafruit BNO085 / BNO080 (0x4A or 0x4B) -> onboard sensor mounted on the Mega
//   - Adafruit BNO055 (0x28 or 0x29)
//   - MPU6050 / MPU9250 (0x68 or 0x69)
#define BNO055_ADDR 0x28
#define BNO085_ADDR 0x4A
#define MPU6050_ADDR 0x68
// Sign flips so that +roll = right side down, +pitch = nose up, +yaw = CCW.
// Fix these on the bench (IMU_TEST) after mounting. Axis swap: set IMU_SWAP_RP to 1
// if the IMU is rotated 90 degrees on the frame.
#define IMU_SWAP_RP   1
#define ROLL_SIGN   (-1.0f)
#define PITCH_SIGN  (+1.0f)
#define YAW_SIGN    (-1.0f)     // BNO055/BNO085 heading sign
#define GYRO_ROLL_SIGN   (-1.0f)
#define GYRO_PITCH_SIGN  (+1.0f)
#define GYRO_YAW_SIGN    (+1.0f)
// Static trim (deg) if the IMU is not perfectly level on the frame
#define ROLL_TRIM_DEG   0.0f
#define PITCH_TRIM_DEG  0.0f

// ---------------------------------------------------------------- Loop timing
#define LOOP_US        5000UL   // 200 Hz control loop
#define TELEMETRY_MS   20UL     // 50 Hz telemetry when enabled
#define CMD_TIMEOUT_MS 1000UL   // if no command/PING for this long while armed -> disarm
#define CMD_SERIAL     Serial   // USB. Use Serial1/2/3 if the ESP8266 is routed there.
#define CMD_BAUD       115200

// ---------------------------------------------------------------- Safety limits
#define MAX_TILT_ARM_DEG   20.0f   // must be within +/-20 deg to arm (or send CAL to level)
#define MAX_TILT_CUT_DEG   55.0f   // above this angle -> immediate disarm
#define MAX_ANGLE_SP_DEG   20.0f   // limit for commanded roll/pitch
#define MAX_RATE_SP_DPS    150.0f  // limit on the rate setpoint from the angle loop
#define MAX_YAW_RATE_DPS   90.0f
#define MAX_CTRL_OUT_US    250.0f  // limit of each axis correction (us)

// ---------------------------------------------------------------- Controller gains (from MATLAB design_gains.m)
// Analytical MATLAB values:
//   ang_kp = 3.7500, yaw_ang_kp = 2.0000
//   rate_kp = 0.5656, rate_ki = 1.6969, rate_kd = 0.0113
//   yawr_kp = 5.7269, yawr_ki = 9.1630, yawr_kd = 0.1145
// Outer loop: angle P (deg -> deg/s)
#define ANG_KP        3.75f
#define YAW_ANG_KP    2.00f
// Inner loop: rate PID (deg/s -> us of motor command)
#define RATE_KP       0.5656f
#define RATE_KI       0.50f    // conservative starting I (raise toward 1.6969f on rig)
#define RATE_KD       0.0113f
#define YAWRATE_KP    2.00f    // conservative starting yaw P (raise toward 5.7269f on rig)
#define YAWRATE_KI    0.30f
#define YAWRATE_KD    0.05f
#define RATE_I_LIMIT  80.0f   // anti-windup clamp of the integral term (us)
#define D_LPF_ALPHA   0.2f    // low-pass on the derivative (0..1, smaller = smoother)

#endif
