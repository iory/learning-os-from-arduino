// ArduinoQuad — 50 Hz RL walking controller for the arduino_os_quad_robot.
//
//   Board : Arduino Uno R4 WiFi (Renesas RA4M1, 48 MHz Cortex-M4)
//   Servos: 8x Feetech STS3215 on one half-duplex TTL bus (Serial1, 1 Mbaud)
//   Policy: mjlab/rsl_rl PPO, exported by scripts/export_quad_policy.sh
//           (ArduinoQuad-Robust fine-tune, NO ERFI -- see docs; 0.148 m/s,
//            4-foot diagonal trot; gait clock 0.30 s, stance height 0.11 m,
//            command ceiling 0.15 m/s)
//
// WHAT IS VERIFIED AND WHAT IS NOT
//   Verified on the host, automatically, every time a policy is exported:
//     * the weights in arduino_quad_policy.h reproduce the PyTorch actor
//       (scripts/export_quad_policy.py, also compiles the generated C and
//       re-checks it)
//     * quad_control.h builds the same observation vector and the same joint
//       targets as the mjlab environment, step for step, over a full rollout
//       (scripts/check_quad_control.sh)
//   NOT verified — this file has never been compiled for, or run on, the real
//   board or the real servos. The bus wiring, the count<->radian calibration in
//   JOINTS[] below, and the loop timing all have to be checked on hardware.
//   Do the bring-up in the order in the "BRING-UP" section before letting it
//   run the policy.
//
// Requires the Feetech SCServo Arduino library (class SMS_STS), and a bus
// adapter that handles half-duplex direction switching (e.g. a TTLinker board).

#include <SCServo.h>

#include "quad_control.h"

// ---------------------------------------------------------------- config --
static const uint32_t CONTROL_HZ = 50;                  // must equal 1/QUAD_CONTROL_DT
static const uint32_t CONTROL_US = 1000000UL / CONTROL_HZ;
static const uint32_t BUS_BAUD = 1000000UL;
static const uint32_t BUS_TIMEOUT_MS = 4;

// STS3215: 4096 counts over a full turn.
static const float RAD_PER_COUNT = 6.28318530718f / 4096.0f;

// Policy output index -> servo. The ORDER IS NOT NEGOTIABLE: it is printed in
// arduino_quad_policy.h ("Joint order for the 8 outputs") and comes from the
// trained checkpoint. Re-export a policy and re-check this list.
//
//   id     : Feetech bus ID of that joint's servo
//   sign   : +1 / -1, whichever makes the servo turn the way the URDF joint
//            does. Find it with the BRING-UP step 2 below; do not guess.
//   zero   : the servo count that corresponds to a joint angle of 0 rad
//            (the URDF zero pose: legs straight down, see docs).
struct JointMap {
  uint8_t id;
  int8_t sign;
  int16_t zero;
};

// TODO(hardware): fill in real IDs, signs and zeros. The placeholder zeros
// assume a centred horn (2048) which is almost certainly not how it is built.
static const JointMap JOINTS[QUAD_NJ] = {
    {1, +1, 2048},  // FL_hip_joint    front left
    {2, +1, 2048},  // FL_knee_joint
    {3, +1, 2048},  // RL_hip_joint    rear  left
    {4, +1, 2048},  // RL_knee_joint
    {5, +1, 2048},  // RR_hip_joint    rear  right
    {6, +1, 2048},  // RR_knee_joint
    {7, +1, 2048},  // FR_hip_joint    front right
    {8, +1, 2048},  // FR_knee_joint
};

// The URDF allows +-2 rad; clamp a bit inside that so a bad policy step cannot
// drive a servo into its mechanical stop.
static const float JOINT_MIN = -1.8f;
static const float JOINT_MAX = 1.8f;

// ----------------------------------------------------------------- state --
SMS_STS st;
static quad_state_t g_ctrl;
static long g_step = 0;
static bool g_running = false;      // policy on/off ('g' / 's' over USB serial)
static float g_cmd[3] = {0.0f, 0.0f, 0.0f};
static float g_q[QUAD_NJ], g_qd[QUAD_NJ], g_q_prev[QUAD_NJ], g_target[QUAD_NJ];
static uint32_t g_next_us = 0;
static uint32_t g_last_read_us = 0;
static long g_skipped = 0;          // control periods lost to a slow bus
static long g_read_fail = 0;        // steps where a servo did not answer

static inline float counts_to_rad(int j, int counts) {
  return (float)JOINTS[j].sign * (float)(counts - JOINTS[j].zero) * RAD_PER_COUNT;
}

static inline int rad_to_counts(int j, float rad) {
  float c = rad / RAD_PER_COUNT * (float)JOINTS[j].sign + (float)JOINTS[j].zero;
  if (c < 0.0f) c = 0.0f;
  if (c > 4095.0f) c = 4095.0f;
  return (int)(c + 0.5f);
}

// Returns false if any servo did not answer; the caller keeps the last target
// rather than feeding the policy a stale-but-plausible reading.
static bool read_joints(float dt) {
  if (dt < 1e-4f) dt = QUAD_CONTROL_DT;  // guard the divide below
  bool ok = true;
  for (int j = 0; j < QUAD_NJ; ++j) {
    int pos = st.ReadPos(JOINTS[j].id);
    if (pos == -1) {
      ok = false;
      continue;
    }
    float q = counts_to_rad(j, pos);
    // Finite difference. The STS3215 also reports a present speed, but its
    // units and its filtering are undocumented enough that differencing the
    // position is the more predictable of the two. One-pole low pass at ~12 Hz
    // to keep encoder quantisation (0.0015 rad) out of a 50 Hz derivative.
    float qd_raw = (q - g_q_prev[j]) / dt;
    g_qd[j] = 0.6f * g_qd[j] + 0.4f * qd_raw;
    g_q_prev[j] = q;
    g_q[j] = q;
  }
  return ok;
}

static void write_targets(void) {
  uint8_t ids[QUAD_NJ];
  int16_t pos[QUAD_NJ];
  uint16_t speed[QUAD_NJ];
  uint8_t acc[QUAD_NJ];
  for (int j = 0; j < QUAD_NJ; ++j) {
    float t = g_target[j];
    if (t < JOINT_MIN) t = JOINT_MIN;
    if (t > JOINT_MAX) t = JOINT_MAX;
    ids[j] = JOINTS[j].id;
    pos[j] = (int16_t)rad_to_counts(j, t);
    speed[j] = 0;  // 0 = go as fast as the servo can; the sim models a plain
                   // position target, not a speed-limited move
    acc[j] = 0;
  }
  st.SyncWritePosEx(ids, QUAD_NJ, pos, speed, acc);
}

// Move from wherever the legs are to the policy's default stance, slowly.
// Skipping this and jumping straight to the home pose is how you snap a bracket.
static void ramp_to_home(uint32_t millis_total) {
  float start[QUAD_NJ];
  for (int j = 0; j < QUAD_NJ; ++j) {
    int p = st.ReadPos(JOINTS[j].id);
    start[j] = (p == -1) ? QUAD_DEFAULT_JOINT_POS[j] : counts_to_rad(j, p);
  }
  uint32_t t0 = millis();
  for (;;) {
    uint32_t el = millis() - t0;
    if (el > millis_total) break;
    float a = (float)el / (float)millis_total;
    for (int j = 0; j < QUAD_NJ; ++j) {
      g_target[j] = start[j] + a * (QUAD_DEFAULT_JOINT_POS[j] - start[j]);
    }
    write_targets();
    delay(20);
  }
  for (int j = 0; j < QUAD_NJ; ++j) g_target[j] = QUAD_DEFAULT_JOINT_POS[j];
  write_targets();
  delay(300);
}

static void handle_serial(void) {
  // "v <vx> <wz>" sets the command, 'g' starts the policy, 's' stops it and
  // holds the stance, 'r' releases torque.
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'g') {
      g_step = 0;
      quad_init(&g_ctrl);
      g_running = true;
      Serial.println("run");
    } else if (c == 's') {
      g_running = false;
      for (int j = 0; j < QUAD_NJ; ++j) g_target[j] = QUAD_DEFAULT_JOINT_POS[j];
      write_targets();
      Serial.println("stop");
    } else if (c == 'r') {
      g_running = false;
      for (int j = 0; j < QUAD_NJ; ++j) st.EnableTorque(JOINTS[j].id, 0);
      Serial.println("release");
    } else if (c == 'v') {
      g_cmd[0] = Serial.parseFloat();
      g_cmd[2] = Serial.parseFloat();
      g_cmd[1] = 0.0f;  // this robot has no lateral DoF; never make it nonzero
      Serial.print("cmd ");
      Serial.print(g_cmd[0]);
      Serial.print(" ");
      Serial.println(g_cmd[2]);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BUS_BAUD);
  st.pSerial = &Serial1;
  // The library defaults to 100 ms, which is five whole control periods: one
  // silent servo would cost more than the entire loop budget. A 1 Mbaud
  // round trip is well under 1 ms, so a few ms is generous and still cheap.
  st.IOTimeOut = BUS_TIMEOUT_MS;
  delay(500);

  for (int j = 0; j < QUAD_NJ; ++j) {
    st.EnableTorque(JOINTS[j].id, 1);
    g_qd[j] = 0.0f;
    int p = st.ReadPos(JOINTS[j].id);
    g_q_prev[j] = (p == -1) ? QUAD_DEFAULT_JOINT_POS[j] : counts_to_rad(j, p);
  }
  // Print what we are actually ramping to. The home pose is baked into the
  // exported header by the stance height the policy was trained at, so this is
  // the only trustworthy statement of it -- check the legs against these.
  Serial.print("home rad:");
  for (int j = 0; j < QUAD_NJ; ++j) {
    Serial.print(' ');
    Serial.print(QUAD_DEFAULT_JOINT_POS[j], 4);
  }
  Serial.println();
  ramp_to_home(2000);
  quad_init(&g_ctrl);
  Serial.println("ready: 'g' run, 's' stop, 'r' release, 'v <vx> <wz>' command");
  g_next_us = micros();
}

void loop() {
  handle_serial();

  uint32_t now = micros();
  if ((int32_t)(now - g_next_us) < 0) return;

  // If the bus stalled (SCServo waits IOTimeOut ms for a servo that does not
  // answer), running the missed periods back to back does not recover the
  // time, and the gait clock -- which is step_i * dt inside quad_control.h --
  // would end up lagging wall time by however long the stall was. A gait clock
  // that no longer matches reality is enough on its own to stop the robot
  // walking. So drop the missed periods and step the counter over them.
  int32_t behind = (int32_t)(now - g_next_us);
  if (behind > (int32_t)CONTROL_US) {
    long lost = behind / (long)CONTROL_US;
    g_step += lost;
    g_skipped += lost;
    g_next_us += (uint32_t)lost * CONTROL_US;
  }
  g_next_us += CONTROL_US;

  // Difference over the time that actually elapsed, not the nominal period --
  // they differ exactly when the loop is in trouble and qd matters most.
  float dt = (g_last_read_us == 0) ? QUAD_CONTROL_DT
                                   : (float)(now - g_last_read_us) * 1e-6f;
  g_last_read_us = now;
  bool ok = read_joints(dt);
  if (!ok) g_read_fail++;

  if (g_running && ok) {
    uint32_t t0 = micros();
    quad_step(&g_ctrl, g_q, g_qd, g_cmd, NULL, g_step, g_target, NULL);
    uint32_t t1 = micros();
    write_targets();
    g_step++;
    // Print the inference time once a second. If it is anywhere near the 20 ms
    // budget the loop is not really running at 50 Hz and the gait clock in
    // quad_control.h no longer matches wall time -- that alone will stop the
    // robot walking, so watch this number during bring-up.
    if ((g_step % CONTROL_HZ) == 0) {
      Serial.print("infer_us=");
      Serial.print(t1 - t0);
      Serial.print(" late_us=");
      Serial.print((int32_t)(micros() - g_next_us));
      Serial.print(" skipped=");
      Serial.print(g_skipped);
      Serial.print(" read_fail=");
      Serial.println(g_read_fail);
    }
  } else if (!ok) {
    // A dropped servo reply means the observation would be wrong. Hold.
    write_targets();
  }
}

// ------------------------------------------------------------- BRING-UP ---
//  1. Bus: power the servos from the battery (NOT from the R4's 5 V), set every
//     servo to a unique ID 1..8 with the Feetech tool, and confirm ReadPos()
//     answers for all eight.
//  2. Signs and zeros: with the robot held off the ground, send 'r' to release
//     torque, move one joint by hand to the pose the URDF calls zero (all four
//     legs straight down, the q=0 pose), and record ReadPos() as that joint's
//     `zero`. Then push the joint the way the URDF's positive direction goes
//     (positive = the toe swings BACKWARD, for both hip and knee, on all four
//     legs) and check whether the count went up or down: up -> sign +1,
//     down -> sign -1.
//
//     "FRONT" is the end the robot leads with, which is NOT the end with the
//     solid section of the tray -- that end is the tail. A scripted trot walks
//     the machine at 0.156 m/s leading with the open end versus 0.089 m/s the
//     other way round, so the convention is not arbitrary.
//  3. Stance: with the robot still off the ground, power up and watch it ramp
//     to the home stance. setup() prints the exact angles it is ramping to
//     (they come from the checkpoint, so do not trust a number written in
//     a comment -- re-exporting at a different stance height changes them).
//     Every leg must look the
//     same. If one leg mirrors the others, its sign is wrong -- fix it before
//     going further.
//  4. Standing: put it on the floor with torque on but the policy stopped. It
//     should hold 0.11 m of body height using about 0.27 Nm per joint, which is
//     9 % of the servo's stall torque -- they should barely notice.
//  5. Policy: hold the robot so the feet just touch, send 'v 0.1 0' then 'g',
//     and check infer_us and late_us. Only then let it take its own weight.
