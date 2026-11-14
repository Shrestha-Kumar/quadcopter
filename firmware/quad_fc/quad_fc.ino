// quad_fc.ino - cascaded PID attitude controller for Arduino Mega 2560.
// Supports BNO085/BNO080 (0x4A/0x4B), BNO055 (0x28/0x29), and MPU6050 (0x68/0x69) on I2C (pins 20/21).
// Control structure: angle P (outer) -> rate PID (inner) -> X mixer -> ESC (Servo.h).
// Commands arrive as text lines on CMD_SERIAL; see HELP below.
//
// SAFETY: ALWAYS test with propellers removed first. Motors only run when ARMED.
// Armed state ends on: DISARM, STOP, tilt > MAX_TILT_CUT_DEG, IMU failure or command timeout.
//
// Original board firmware is backed up in ../../backup/ (restore: see backup/README.md).

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>
#include <utility/imumaths.h>
#include "config.h"

// ------------------------------------------------------------------ state
enum State { DISARMED, ARMED };
static State state = DISARMED;

enum ImuType { IMU_NONE = 0, IMU_BNO085, IMU_BNO055, IMU_MPU6050 };
static ImuType imu_type = IMU_NONE;

static Adafruit_BNO055 bno055 = Adafruit_BNO055(55, BNO055_ADDR);
static Adafruit_BNO08x bno085(-1);
static sh2_SensorValue_t sensorValue;

static Servo esc[4];
static const uint8_t motor_pin[4] = { PIN_MOTOR_0, PIN_MOTOR_1, PIN_MOTOR_2, PIN_MOTOR_3 };
static int motor_us[4] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };

// Gains (runtime-adjustable via SET)
static float ang_kp = ANG_KP, yaw_ang_kp = YAW_ANG_KP;
static float rate_kp = RATE_KP, rate_ki = RATE_KI, rate_kd = RATE_KD;
static float yawr_kp = YAWRATE_KP, yawr_ki = YAWRATE_KI, yawr_kd = YAWRATE_KD;
static float sp_slew_dps = SP_SLEW_DPS;

// Measurements
static float raw_r = 0, raw_p = 0, raw_h = 0;
static float raw_gx = 0, raw_gy = 0, raw_gz = 0;
static float roll_trim = ROLL_TRIM_DEG, pitch_trim = PITCH_TRIM_DEG;
static float roll = 0, pitch = 0, yaw = 0;       // deg
static float gx = 0, gy = 0, gz = 0;             // deg/s (roll, pitch, yaw rates)
static bool imu_ok = false;
static bool imu_new_gyro = false;
static unsigned long imu_last_event_ms = 0, imu_last_recover_ms = 0;

// Setpoints
static float roll_sp = 0, pitch_sp = 0, yaw_sp = 0;  // deg (commanded)
static float roll_sp_eff = 0, pitch_sp_eff = 0;      // deg (slew-ramped target)
static int thr_us = ESC_MIN_US;

// PID state
struct Rate { float i, prev_meas, d_filt; };
static Rate rr = {0,0,0}, rp = {0,0,0}, ry = {0,0,0};
static float gyro_dt_accum = 0.0f;

static unsigned long last_loop_us = 0, last_tel_ms = 0, last_cmd_ms = 0;
static bool telemetry_on = true;
static bool imu_test_on = false;
static bool esc_test_active = false;
static unsigned long esc_test_until_ms = 0;
static float yaw_ref_offset = 0;   // yaw is relative to the value when armed
static bool  cal_locked = false;   // true once user clicks CAL so ARM/DISARM keeps level zero
static float yaw_dir = 1.0f;       // +1 or -1 runtime yaw mixer flip
static float yaw_trim_us = 0.0f;   // manual CW/CCW yaw torque trim (us)

static String line_usb;
static String line_wifi;

class DualSerial : public Print {
public:
  size_t write(uint8_t b) override {
    Serial.write(b);
    Serial3.write(b);
    return 1;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    Serial.write(buffer, size);
    Serial3.write(buffer, size);
    return size;
  }
};
static DualSerial dual_out;
#undef CMD_SERIAL
#define CMD_SERIAL dual_out

// ------------------------------------------------------------------ helpers
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

static int motor_trim_us[4] = { 35, 0, 0, 0 }; // M0 (FR, Pin 9) ESC deadband trim (+35 us)

static void write_motors_us(const int us[4]) {
  for (int i = 0; i < 4; i++) {
    int v = us[i];
    if (v > ESC_MIN_US) v += motor_trim_us[i];
    motor_us[i] = constrain(v, ESC_MIN_US, ESC_MAX_US);
    esc[i].writeMicroseconds(motor_us[i]);
  }
}

static void motors_off() {
  int z[4] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
  write_motors_us(z);
}

static void reset_pid() {
  rr = {0,0,0}; rp = {0,0,0}; ry = {0,0,0};
  gyro_dt_accum = 0.0f;
  imu_new_gyro = false;
  roll_sp_eff = roll_sp;
  pitch_sp_eff = pitch_sp;
}

static void disarm(const __FlashStringHelper* why) {
  state = DISARMED;
  motors_off();
  reset_pid();
  thr_us = ESC_MIN_US;
  esc_test_active = false;
  CMD_SERIAL.print(F("DISARMED: ")); CMD_SERIAL.println(why);
}

// ------------------------------------------------------------------ IMU
static void i2c_bus_recover() {
  TWCR &= ~(_BV(TWEN));
  pinMode(20, INPUT_PULLUP);
  pinMode(21, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWrite(21, LOW);
    delayMicroseconds(10);
    digitalWrite(21, HIGH);
    delayMicroseconds(10);
  }
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
  delayMicroseconds(10);
  digitalWrite(21, HIGH);
  delayMicroseconds(10);
  digitalWrite(20, HIGH);
  delayMicroseconds(10);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(100000);
}

static void print_i2c_scan() {
  i2c_bus_recover();
  CMD_SERIAL.print(F("I2C_SCAN:"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      CMD_SERIAL.print(F(" 0x"));
      if (a < 16) CMD_SERIAL.print('0');
      CMD_SERIAL.print(a, HEX);
      n++;
    }
  }
  if (n == 0) CMD_SERIAL.print(F(" none"));
  CMD_SERIAL.println();
}

static void set_bno085_reports() {
  bno085.enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 100 Hz quaternion
  bno085.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000); // 100 Hz gyro
}

static bool init_imu(bool force_reinit = false) {
  imu_last_recover_ms = millis();
  if (imu_type == IMU_BNO085 && !force_reinit) {
    Wire.beginTransmission(0x4A);
    if (Wire.endTransmission() == 0) {
      Wire.setClock(100000);
      Wire.setWireTimeout(25000, true);
      set_bno085_reports();
      imu_last_event_ms = millis();
      CMD_SERIAL.println(F("IMU: BNO085 ok (0x4A)"));
      return true;
    }
  }
  // 1. Try BNO085 / BNO080 (0x4A)
  Wire.setWireTimeout(0, false); // disable timeout during BNO085 SHTP handshake
  Wire.beginTransmission(0x4A);
  if (Wire.endTransmission() == 0) {
    // Drain any unread SHTP packets from warm reset before opening SH2
    for (int d = 0; d < 8; d++) {
      Wire.requestFrom((uint8_t)0x4A, (uint8_t)32);
      while (Wire.available()) Wire.read();
      delay(5);
    }
    for (int attempt = 0; attempt < 3; attempt++) {
      if (bno085.begin_I2C(0x4A, &Wire)) {
        set_bno085_reports();
        Wire.setClock(100000);           // 100 kHz for strong EMI immunity over jumper wires
        Wire.setWireTimeout(25000, true); // 25 ms safety timeout after init
        imu_type = IMU_BNO085;
        imu_last_event_ms = millis();
        CMD_SERIAL.println(F("IMU: BNO085 initialized"));
        return true;
      }
      delay(50);
    }
  }
  Wire.setWireTimeout(25000, true);
  // 2. Try BNO055 (0x28 or 0x29)
  bno055 = Adafruit_BNO055(55, 0x28);
  if (bno055.begin(OPERATION_MODE_IMUPLUS)) {
    bno055.setExtCrystalUse(true);
    imu_type = IMU_BNO055;
    CMD_SERIAL.println(F("IMU: BNO055 (0x28) initialized"));
    return true;
  }
  bno055 = Adafruit_BNO055(55, 0x29);
  if (bno055.begin(OPERATION_MODE_IMUPLUS)) {
    bno055.setExtCrystalUse(true);
    imu_type = IMU_BNO055;
    CMD_SERIAL.println(F("IMU: BNO055 (0x29) initialized"));
    return true;
  }
  // 3. Try MPU6050 (0x68)
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    imu_type = IMU_MPU6050;
    CMD_SERIAL.println(F("IMU: MPU6050 (0x68) initialized"));
    return true;
  }
  imu_type = IMU_NONE;
  return false;
}

static bool read_imu() {
  if (imu_type == IMU_NONE) return false;

  if (imu_type == IMU_BNO085) {
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
    }
    if (bno085.wasReset()) {
      set_bno085_reports();
    }
    // Drain up to 3 queued SHTP events per tick so GameRV + Gyro stay synchronized
    for (int k = 0; k < 3 && bno085.getSensorEvent(&sensorValue); k++) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        float qw = sensorValue.un.gameRotationVector.real;
        float qx = sensorValue.un.gameRotationVector.i;
        float qy = sensorValue.un.gameRotationVector.j;
        float qz = sensorValue.un.gameRotationVector.k;
        float n2 = qw*qw + qx*qx + qy*qy + qz*qz;
        if (!isnan(n2) && n2 > 0.8f && n2 < 1.2f) {
          raw_r = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * RAD_TO_DEG;
          float sinp = clampf(2.0f*(qw*qy - qz*qx), -1.0f, 1.0f);
          raw_p = asinf(sinp) * RAD_TO_DEG;
          raw_h = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * RAD_TO_DEG;
          imu_last_event_ms = millis();
        }
      } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        float gx_t = sensorValue.un.gyroscope.x * RAD_TO_DEG;
        float gy_t = sensorValue.un.gyroscope.y * RAD_TO_DEG;
        float gz_t = sensorValue.un.gyroscope.z * RAD_TO_DEG;
        if (!isnan(gx_t) && !isnan(gy_t) && !isnan(gz_t) &&
            fabsf(gx_t) <= 1000.0f && fabsf(gy_t) <= 1000.0f && fabsf(gz_t) <= 1000.0f) {
          raw_gx = gx_t;
          raw_gy = gy_t;
          raw_gz = gz_t;
          imu_new_gyro = true;
          imu_last_event_ms = millis();
        }
      }
    }
  } else if (imu_type == IMU_BNO055) {
    imu::Vector<3> e = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> g = bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    if (isnan(e.x()) || isnan(e.y()) || isnan(e.z())) return false;
    raw_r = e.y(); raw_p = e.z(); raw_h = e.x();
    raw_gx = g.x(); raw_gy = g.y(); raw_gz = g.z();
    imu_new_gyro = true;
  } else if (imu_type == IMU_MPU6050) {
    Wire.beginTransmission(0x68); Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)0x68, (uint8_t)14) != 14) return false;
    int16_t ax = (Wire.read()<<8)|Wire.read();
    int16_t ay = (Wire.read()<<8)|Wire.read();
    int16_t az = (Wire.read()<<8)|Wire.read();
    Wire.read(); Wire.read();
    int16_t rx = (Wire.read()<<8)|Wire.read();
    int16_t ry_raw = (Wire.read()<<8)|Wire.read();
    int16_t rz = (Wire.read()<<8)|Wire.read();
    raw_gx = rx / 131.0f; raw_gy = ry_raw / 131.0f; raw_gz = rz / 131.0f;
    imu_new_gyro = true;
    float acc_r = atan2f((float)ay, (float)az) * RAD_TO_DEG;
    float acc_p = atan2f(-(float)ax, sqrtf((float)ay*ay + (float)az*az)) * RAD_TO_DEG;
    raw_r = 0.98f*(raw_r + raw_gx*0.005f) + 0.02f*acc_r;
    raw_p = 0.98f*(raw_p + raw_gy*0.005f) + 0.02f*acc_p;
    raw_h = raw_h + raw_gz*0.005f;
  }

  float r = raw_r, p = raw_p, h = raw_h;
  float grx = raw_gx, gry = raw_gy, grz = raw_gz;
#if IMU_SWAP_RP
  float t = r; r = p; p = t;
  t = grx; grx = gry; gry = t;
#endif
  roll  = ROLL_SIGN  * r + roll_trim;
  pitch = PITCH_SIGN * p + pitch_trim;
  yaw   = wrap180(YAW_SIGN * h - yaw_ref_offset);
  gx = GYRO_ROLL_SIGN * grx;
  gy = GYRO_PITCH_SIGN * gry;
  gz = GYRO_YAW_SIGN * grz;
  return true;
}

// ------------------------------------------------------------------ control
static float rate_pid(Rate& s, float sp, float meas, float kp, float ki, float kd, float ilim, float dt, bool do_d, float d_dt) {
  float err = sp - meas;
  if (thr_us >= 1300) {
    s.i = clampf(s.i + ki * err * dt, -ilim, ilim);
  } else {
    s.i = 0.0f;
  }
  if (do_d) {
    float d_raw = -(meas - s.prev_meas) / d_dt;
    s.d_filt += D_LPF_ALPHA * (d_raw - s.d_filt);
    s.prev_meas = meas;
  }
  float out = kp * err + s.i + kd * s.d_filt;
  return clampf(out, -MAX_CTRL_OUT_US, MAX_CTRL_OUT_US);
}

static void control_step(float dt) {
  gyro_dt_accum += dt;
  bool do_d = false;
  float d_dt = gyro_dt_accum;
  if (imu_new_gyro) {
    do_d = true;
    imu_new_gyro = false;
    gyro_dt_accum = 0.0f;
    if (d_dt < 0.004f) d_dt = 0.004f;
    if (d_dt > 0.050f) d_dt = 0.050f;
  }

  float max_step = sp_slew_dps * dt;
  roll_sp_eff  += clampf(roll_sp  - roll_sp_eff,  -max_step, max_step);
  pitch_sp_eff += clampf(pitch_sp - pitch_sp_eff, -max_step, max_step);

  float rate_sp_r = clampf(ang_kp * (roll_sp_eff  - roll),  -MAX_RATE_SP_DPS, MAX_RATE_SP_DPS);
  float rate_sp_p = clampf(ang_kp * (pitch_sp_eff - pitch), -MAX_RATE_SP_DPS, MAX_RATE_SP_DPS);
  float rate_sp_y = clampf(yaw_ang_kp * wrap180(yaw_sp - yaw), -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

  float u_r = rate_pid(rr, rate_sp_r, gx, rate_kp, rate_ki, rate_kd, RATE_I_LIMIT, dt, do_d, d_dt);
  float u_p = rate_pid(rp, rate_sp_p, gy, rate_kp, rate_ki, rate_kd, RATE_I_LIMIT, dt, do_d, d_dt);
  float u_y = clampf(yaw_dir * rate_pid(ry, rate_sp_y, gz, yawr_kp, yawr_ki, yawr_kd, YAW_I_LIMIT, dt, do_d, d_dt) + yaw_trim_us, -MAX_YAW_OUT_US, MAX_YAW_OUT_US);

  int out[4];
  float base = clampf(thr_us, ESC_MIN_US, THR_MAX_US);
  for (int i = 0; i < 4; i++) {
    float m = base + MIX_R[i] * u_r + MIX_P[i] * u_p + MIX_Y[i] * u_y;
    out[i] = (int)clampf(m, MOTOR_IDLE_US, ESC_MAX_US);
  }
  if (thr_us < MOTOR_IDLE_US) { for (int i = 0; i < 4; i++) out[i] = MOTOR_IDLE_US; reset_pid(); }
  write_motors_us(out);
}

// ------------------------------------------------------------------ telemetry
static void send_telemetry() {
  CMD_SERIAL.print(F("T,")); CMD_SERIAL.print(millis());
  CMD_SERIAL.print(','); CMD_SERIAL.print(state == ARMED ? 1 : 0);
  CMD_SERIAL.print(','); CMD_SERIAL.print(roll, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(pitch, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(yaw, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(gx, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(gy, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(gz, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(roll_sp, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(pitch_sp, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(yaw_sp, 2);
  CMD_SERIAL.print(','); CMD_SERIAL.print(thr_us);
  for (int i = 0; i < 4; i++) { CMD_SERIAL.print(','); CMD_SERIAL.print(motor_us[i]); }
  CMD_SERIAL.println();
}

// ------------------------------------------------------------------ commands
static void print_help() {
  CMD_SERIAL.println(F("Commands:"));
  CMD_SERIAL.println(F(" ARM | DISARM | STOP(emergency) | PING | SCAN | STATUS | GET | HELP"));
  CMD_SERIAL.println(F(" THR:<us 1000..2000> | UP | DOWN (+/-10us)"));
  CMD_SERIAL.println(F(" ROLL:<deg> | PITCH:<deg> | YAW:<deg>  (setpoints, YAW is relative to arming)"));
  CMD_SERIAL.println(F(" SET:<name>=<value>  names: ang_kp yaw_ang_kp rate_kp rate_ki rate_kd yawr_kp yawr_ki yawr_kd"));
  CMD_SERIAL.println(F(" TEL:1|0 (telemetry CSV 'T,...')  IMU_TEST:1|0 (print angles)"));
  CMD_SERIAL.println(F(" ESC_TEST:<motor 0-3>:<us 1000..1200>  (disarmed only, props OFF, runs 2 s)"));
}

static void print_gains() {
  CMD_SERIAL.print(F("GAINS ang_kp=")); CMD_SERIAL.print(ang_kp);
  CMD_SERIAL.print(F(" yaw_ang_kp=")); CMD_SERIAL.print(yaw_ang_kp);
  CMD_SERIAL.print(F(" rate_kp=")); CMD_SERIAL.print(rate_kp);
  CMD_SERIAL.print(F(" rate_ki=")); CMD_SERIAL.print(rate_ki);
  CMD_SERIAL.print(F(" rate_kd=")); CMD_SERIAL.print(rate_kd, 3);
  CMD_SERIAL.print(F(" yawr_kp=")); CMD_SERIAL.print(yawr_kp);
  CMD_SERIAL.print(F(" yawr_ki=")); CMD_SERIAL.print(yawr_ki);
  CMD_SERIAL.print(F(" yawr_kd=")); CMD_SERIAL.print(yawr_kd, 3);
  CMD_SERIAL.print(F(" sp_slew=")); CMD_SERIAL.println(sp_slew_dps);
}

static void zero_level(bool update_rp) {
  float r = raw_r, p = raw_p, h = raw_h;
#if IMU_SWAP_RP
  float t = r; r = p; p = t;
#endif
  if (update_rp) {
    roll_trim  = -(ROLL_SIGN * r);
    pitch_trim = -(PITCH_SIGN * p);
    roll = 0; pitch = 0;
  }
  yaw_ref_offset = YAW_SIGN * h;
  yaw = 0;
}

static void try_arm() {
  if (!imu_ok) { CMD_SERIAL.println(F("REFUSED: IMU not ok (send SCAN to check I2C)")); return; }
  if (imu_type == IMU_BNO085 && (millis() - imu_last_event_ms > 400UL)) {
    CMD_SERIAL.println(F("REFUSED: IMU frozen/stale - wait 2s or click SCAN"));
    return;
  }
  zero_level(!cal_locked);
  roll_sp = pitch_sp = yaw_sp = 0;
  reset_pid();
  rr.prev_meas = gx; rp.prev_meas = gy; ry.prev_meas = gz;
  thr_us = 1095;
  state = ARMED;
  last_cmd_ms = millis();
  CMD_SERIAL.println(F("ACK:ARM"));
}

static void handle_command(String raw_c, bool from_wifi = false) {
  raw_c.trim();
  if (raw_c.length() == 0) return;
  if (!from_wifi && (raw_c.startsWith("ESP:") || raw_c.startsWith("esp:"))) {
    String esp_cmd = raw_c.substring(4);
    Serial.print(F("[TO_ESP] ")); Serial.println(esp_cmd);
    Serial3.print(esp_cmd); Serial3.print("\r\n");
    return;
  }
  String c = raw_c;
  c.toUpperCase();
  last_cmd_ms = millis();

  if (c == "STOP") { disarm(F("EMERGENCY STOP")); return; }
  if (c == "DISARM") { disarm(F("by command")); CMD_SERIAL.println(F("ACK:DISARM")); return; }
  if (c == "ARM") { try_arm(); return; }
  if (c == "PING") { CMD_SERIAL.println(F("PONG")); return; }
  if (c == "SCAN") { print_i2c_scan(); imu_ok = init_imu(true); return; }
  if (c == "CAL") {
    zero_level(true);
    cal_locked = true;
    CMD_SERIAL.println(F("ACK:CAL (level locked across ARM/DISARM)"));
    return;
  }
  if (c == "HELP") { print_help(); return; }
  if (c == "GET") { print_gains(); return; }
  if (c == "STATUS") {
    CMD_SERIAL.print(F("STATUS state=")); CMD_SERIAL.print(state == ARMED ? F("ARMED") : F("DISARMED"));
    CMD_SERIAL.print(F(" imu=")); CMD_SERIAL.print(imu_ok ? F("ok") : F("FAIL"));
    CMD_SERIAL.print(F(" thr=")); CMD_SERIAL.print(thr_us);
    CMD_SERIAL.print(F(" roll=")); CMD_SERIAL.print(roll, 1);
    CMD_SERIAL.print(F(" pitch=")); CMD_SERIAL.print(pitch, 1);
    CMD_SERIAL.print(F(" yaw=")); CMD_SERIAL.println(yaw, 1);
    return;
  }
  if (c == "UP")   { thr_us = constrain(thr_us + 10, ESC_MIN_US, THR_MAX_US); CMD_SERIAL.println(F("ACK:UP")); return; }
  if (c == "DOWN") { thr_us = constrain(thr_us - 10, ESC_MIN_US, THR_MAX_US); CMD_SERIAL.println(F("ACK:DOWN")); return; }

  if (c.startsWith("THR:")) {
    thr_us = constrain(c.substring(4).toInt(), ESC_MIN_US, THR_MAX_US);
    CMD_SERIAL.print(F("THR:")); CMD_SERIAL.println(thr_us); return;
  }
  if (c.startsWith("ROLL:"))  { roll_sp  = clampf(c.substring(5).toFloat(), -MAX_ANGLE_SP_DEG, MAX_ANGLE_SP_DEG); CMD_SERIAL.println(F("ACK:ROLL")); return; }
  if (c.startsWith("PITCH:")) { pitch_sp = clampf(c.substring(6).toFloat(), -MAX_ANGLE_SP_DEG, MAX_ANGLE_SP_DEG); CMD_SERIAL.println(F("ACK:PITCH")); return; }
  if (c.startsWith("YAW:"))   { yaw_sp   = wrap180(c.substring(4).toFloat()); CMD_SERIAL.println(F("ACK:YAW")); return; }
  if (c.startsWith("TEL:"))   { telemetry_on = c.substring(4).toInt() != 0; CMD_SERIAL.println(F("ACK:TEL")); return; }
  if (c.startsWith("IMU_TEST")) { imu_test_on = !c.endsWith(":0"); CMD_SERIAL.println(F("Mode: IMU TEST")); return; }

  if (c.startsWith("PID:")) {
    int c1 = c.indexOf(',', 4);
    int c2 = (c1 > 0) ? c.indexOf(',', c1 + 1) : -1;
    int c3 = (c2 > 0) ? c.indexOf(',', c2 + 1) : -1;
    if (c3 < 0) { CMD_SERIAL.println(F("ERR: use PID:akp,rkp,rki,rkd")); return; }
    float ak = c.substring(4, c1).toFloat();
    float rk = c.substring(c1 + 1, c2).toFloat();
    float ri = c.substring(c2 + 1, c3).toFloat();
    float rd = c.substring(c3 + 1).toFloat();
    if (ak < 0 || rk < 0 || ri < 0 || rd < 0) { CMD_SERIAL.println(F("ERR: negative gain")); return; }
    ang_kp = ak; rate_kp = rk; rate_ki = ri; rate_kd = rd;
    print_gains(); return;
  }

  if (c.startsWith("SET:")) {
    int eq = c.indexOf('=');
    if (eq < 0) { CMD_SERIAL.println(F("ERR: use SET:name=value")); return; }
    String n = c.substring(4, eq); float v = c.substring(eq + 1).toFloat();
    if      (n == "YAW_DIR")    yaw_dir = (v < 0) ? -1.0f : 1.0f;
    else if (n == "YAW_TRIM")   yaw_trim_us = clampf(v, -120.0f, 120.0f);
    else if (v < 0) { CMD_SERIAL.println(F("ERR: negative gain")); return; }
    else if (n == "ANG_KP")     ang_kp = v;
    else if (n == "YAW_ANG_KP") yaw_ang_kp = v;
    else if (n == "RATE_KP")    rate_kp = v;
    else if (n == "RATE_KI")    rate_ki = v;
    else if (n == "RATE_KD")    rate_kd = v;
    else if (n == "YAWR_KP")    yawr_kp = v;
    else if (n == "YAWR_KI")    yawr_ki = v;
    else if (n == "YAWR_KD")    yawr_kd = v;
    else if (n == "SP_SLEW")    sp_slew_dps = clampf(v, 5.0f, 120.0f);
    else if (n == "M0_TRIM")    motor_trim_us[0] = (int)v;
    else { CMD_SERIAL.println(F("ERR: unknown gain")); return; }
    print_gains(); return;
  }

  if (c.startsWith("ESC_TEST:")) {
    if (state == ARMED) { CMD_SERIAL.println(F("REFUSED: disarm first")); return; }
    int p = c.indexOf(':', 9);
    if (p < 0) { CMD_SERIAL.println(F("ERR: ESC_TEST:<motor>:<us>")); return; }
    int m = c.substring(9, p).toInt();
    int us = c.substring(p + 1).toInt();
    if (m < 0 || m > 3 || us < ESC_MIN_US || us > 1200) { CMD_SERIAL.println(F("ERR: motor 0-3, us 1000-1200")); return; }
    int o[4] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
    o[m] = us; write_motors_us(o);
    esc_test_active = true; esc_test_until_ms = millis() + 2000;
    CMD_SERIAL.println(F("Mode: ESC TEST (2 s)")); return;
  }
  if (!from_wifi) {
    Serial.println(F("ERR: unknown command (HELP)"));
  }
}

static void poll_serial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line_usb.length() > 0) { handle_command(line_usb, false); line_usb = ""; }
    } else if (line_usb.length() < 80) {
      line_usb += ch;
    }
  }
  while (Serial3.available()) {
    char ch = Serial3.read();
    if (ch == '\n' || ch == '\r') {
      if (line_wifi.length() > 0) {
        Serial.print(F("[WIFI] ")); Serial.println(line_wifi);
        handle_command(line_wifi, true);
        line_wifi = "";
      }
    } else if (line_wifi.length() < 80) {
      line_wifi += ch;
    }
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  Serial.begin(CMD_BAUD);
  Serial3.begin(115200);
  for (int i = 0; i < 4; i++) { esc[i].attach(motor_pin[i], ESC_MIN_US, ESC_MAX_US); }
  motors_off();

  Wire.begin();
  Wire.setClock(100000);
  print_i2c_scan();
  imu_ok = init_imu();
  Wire.setWireTimeout(25000, true);
  if (imu_ok) read_imu();

  CMD_SERIAL.println(F("=== QUAD_FC READY (DISARMED, USB+WIFI Serial3) ==="));
  CMD_SERIAL.println(F("=== WIFI HOVER TEST READY ==="));
  if (!imu_ok) CMD_SERIAL.println(F("WARNING: IMU not detected on I2C (pins 20=SDA, 21=SCL). Check IMU power/I2C jumpers & send SCAN."));
  CMD_SERIAL.println(F("Type HELP for commands. Props OFF for bench tests!"));
  last_loop_us = micros();
}

void loop() {
  poll_serial();

  unsigned long now_us = micros();
  if (now_us - last_loop_us >= LOOP_US) {
    float dt = (now_us - last_loop_us) * 1e-6f;
    last_loop_us = now_us;

    if (imu_ok) read_imu();

    if (state == ARMED) {
      if (!imu_ok) disarm(F("IMU failure"));
      else if (imu_type == IMU_BNO085 && (millis() - imu_last_event_ms > 500UL)) disarm(F("IMU frozen/lost"));
      else if (fabs(roll) > MAX_TILT_CUT_DEG || fabs(pitch) > MAX_TILT_CUT_DEG) disarm(F("tilt limit"));
      else if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) disarm(F("command timeout (send PING)"));
      else control_step(dt);
    } else {
      if (esc_test_active && millis() > esc_test_until_ms) {
        motors_off(); esc_test_active = false; CMD_SERIAL.println(F("ESC_TEST done"));
      }
      // Auto-recover frozen BNO085 stream while DISARMED (max once per 3s)
      if (imu_type == IMU_BNO085 && (millis() - imu_last_event_ms > 1500UL) && (millis() - imu_last_recover_ms > 3000UL)) {
        CMD_SERIAL.println(F("IMU: frozen stream, recovering BNO085..."));
        i2c_bus_recover();
        imu_ok = init_imu(true);
      }
    }
  }

  unsigned long ms = millis();
  if (telemetry_on && ms - last_tel_ms >= TELEMETRY_MS) { last_tel_ms = ms; send_telemetry(); }
  if (imu_test_on && ms - last_tel_ms >= 100) {
    last_tel_ms = ms;
    CMD_SERIAL.print(F("Roll: ")); CMD_SERIAL.print(roll, 1);
    CMD_SERIAL.print(F("  Pitch: ")); CMD_SERIAL.print(pitch, 1);
    CMD_SERIAL.print(F("  Yaw: ")); CMD_SERIAL.println(yaw, 1);
  }
}
