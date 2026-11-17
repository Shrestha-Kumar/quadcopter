// config.h - all tunable parameters for the quad_fc firmware.
// Target: Arduino Mega 2560 + Adafruit BNO055 (I2C) + 4 ESC (A2212 1000KV).
// Layout inspired by DanieleVeri/quadcopter (angle -> rate cascade, config.h).
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------- Motors
// Verified hardware pin mapping on Arduino Mega 2560 (PWM pins 9, 8, 7, 6).
// Motor index convention (looking from above, nose = front):
//   0 = front-right (CCW), 1 = front-left (CW), 2 = rear-left (CCW), 3 = rear-right (CW)
#define PIN_MOTOR_0 9
#define PIN_MOTOR_1 8
#define PIN_MOTOR_2 7
#define PIN_MOTOR_3 6

#define ESC_MIN_US      1000   // disarmed / zero throttle
#define ESC_MAX_US      2000
#define MOTOR_IDLE_US   1050   // low floor while armed so slowed-down motors visibly slow down
// Hard cap on the throttle command. Raised to 1750 us for full hover lift on the tether rig.
#define THR_MAX_US      1750
#define THR_ARM_MAX_US  1050   // throttle must be below this to arm

// Mixer: motor_i = thr + MIX_R[i]*roll + MIX_P[i]*pitch + MIX_Y[i]*yaw
//   +roll  = right side goes DOWN  (speed up right motors M0, M3 when u_r < 0)
//   +pitch = nose goes UP          (speed up rear motors M2, M3 when u_p < 0)
//   +yaw   = CCW seen from above   (speed up CW motors M1, M3 when u_y > 0)
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
#define YAW_SIGN    (+1.0f)     // BNO085 quaternion yaw is +CCW (matches +CCW gyro Z)
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
#define SP_SLEW_DPS        22.0f   // max setpoint ramp rate (deg/s) so +5 <-> -5 reversals don't slam
#define MAX_RATE_SP_DPS    150.0f  // limit on the rate setpoint from the angle loop
#define MAX_YAW_RATE_DPS   90.0f
#define MAX_CTRL_OUT_US    220.0f  // full balancing torque with mild safety cap (was 250)

// ---------------------------------------------------------------- Controller gains (from MATLAB design_gains.m)
// Analytical MATLAB values:
//   ang_kp = 3.7500, yaw_ang_kp = 2.0000
//   rate_kp = 0.5656, rate_ki = 1.6969, rate_kd = 0.0113
//   yawr_kp = 5.7269, yawr_ki = 9.1630, yawr_kd = 0.1145
// Outer loop: angle P (deg -> deg/s)
#define ANG_KP        2.80f    // tuned on the rig (screenshot 2026-11-14): calm outer loop
#define YAW_ANG_KP    2.60f    // firmer heading lock to hold Yaw near 0 deg
// Inner loop: rate PID (deg/s -> us of motor command)
#define RATE_KP       2.60f    // tuned on the rig: stiff inner rate hold
#define RATE_KI       0.80f    // tuned on the rig: cancels CG & rig tilt offset
#define RATE_KD       0.150f   // tuned on the rig: damping on fresh 100 Hz gyro samples
#define YAWRATE_KP    3.00f    // stronger yaw rate damping against roll-induced yaw kicks
#define YAWRATE_KI    0.65f    // absorbs ~45-55 us CW/CCW drag asymmetry quickly above 1300 us
#define YAWRATE_KD    0.00f
#define RATE_I_LIMIT  40.0f    // anti-windup clamp of roll/pitch integral term (us)
#define YAW_I_LIMIT   80.0f    // large enough to hold full CW/CCW steady drag offset
#define MAX_YAW_OUT_US 115.0f  // leaves ~65+ us dynamic yaw authority above steady bias
#define D_LPF_ALPHA   0.30f    // low-pass on fresh 100 Hz derivative (0..1, smaller = smoother)

#endif
