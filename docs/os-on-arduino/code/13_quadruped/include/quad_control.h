// Observation assembly and action decoding for the ArduinoQuad policy.
//
// Pure C99, no Arduino and no servo dependency, so it can be compiled and
// tested on the host against the mjlab environment -- which is the point: the
// bug that kills a sim2real deployment is almost never the network, it is the
// observation vector being assembled in a different order, with a different
// history convention, or from differently-scaled units than the one the policy
// was trained on. scripts/check_quad_control.py replays a trained policy in
// mjlab and asserts this file reproduces the environment's own observation
// vector step for step.
//
// Everything layout-related comes from arduino_quad_policy.h, which is
// generated from the checkpoint, so re-exporting a policy with a different
// observation set cannot silently desync this code -- the constants move with
// it.
//
// Units are the simulation's: radians, radians/second, seconds, metres/second.
// Converting servo counts <-> radians is the caller's job (see the .ino).

#pragma once

#include <math.h>
#include <string.h>

#include "arduino_quad_policy.h"

#define QUAD_NJ QUAD_ACT_DIM

// M_PI is not in strict C99 (and the Arduino core does not always define it).
#define QUAD_TWO_PI 6.28318530717958647692f

// What the policy needs from an IMU. Pass NULL to quad_step when the policy was
// trained without one (QUAD_HAS_ANG_VEL and QUAD_HAS_GRAVITY both 0).
//
//   ang_vel  body-frame angular velocity [rad/s], x forward, y left, z up --
//            a raw rate gyro, no filtering or attitude estimate needed.
//   gravity  the gravity direction expressed in the body frame, unit length.
//            Level and upright is {0, 0, -1}. This one DOES need an attitude
//            estimate; it is not the raw accelerometer reading.
typedef struct {
  float ang_vel[3];
  float gravity[3];
} quad_imu_t;

typedef struct {
  // Per-term history ring buffers, oldest-to-newest when flattened out.
#if QUAD_HAS_ANG_VEL
  float angvel[QUAD_HISTORY][3];
#endif
#if QUAD_HAS_GRAVITY
  float grav[QUAD_HISTORY][3];
#endif
  float cmd[QUAD_HISTORY][3];
  float phase[QUAD_HISTORY][2];
  float qpos[QUAD_HISTORY][QUAD_NJ];
  float qvel[QUAD_HISTORY][QUAD_NJ];
  float act[QUAD_HISTORY][QUAD_NJ];
  int head;          // index of the newest entry
  int filled;        // how many slots hold real data
  float t;           // seconds since the policy started (gait clock)
  float last_act[QUAD_NJ];
} quad_state_t;

// Start (or restart) the controller. The history is primed with the standing
// pose and a zero action, which is what a reset looks like in mjlab too.
static inline void quad_init(quad_state_t *s) {
  memset(s, 0, sizeof(*s));
  s->head = QUAD_HISTORY - 1;
  s->filled = 0;
}

static inline void quad__push(quad_state_t *s, const float cmd[3],
                              const float qpos_rel[QUAD_NJ],
                              const float qvel[QUAD_NJ],
                              const float last_act[QUAD_NJ],
                              const float phase[2],
                              const quad_imu_t *imu) {
  s->head = (s->head + 1) % QUAD_HISTORY;
#if QUAD_HAS_ANG_VEL
  memcpy(s->angvel[s->head], imu->ang_vel, sizeof(float) * 3);
#endif
#if QUAD_HAS_GRAVITY
  memcpy(s->grav[s->head], imu->gravity, sizeof(float) * 3);
#endif
  (void)imu;
  memcpy(s->cmd[s->head], cmd, sizeof(float) * 3);
  memcpy(s->phase[s->head], phase, sizeof(float) * 2);
  memcpy(s->qpos[s->head], qpos_rel, sizeof(float) * QUAD_NJ);
  memcpy(s->qvel[s->head], qvel, sizeof(float) * QUAD_NJ);
  memcpy(s->act[s->head], last_act, sizeof(float) * QUAD_NJ);
  if (s->filled < QUAD_HISTORY) {
    // Before the buffer is full, mjlab's CircularBuffer repeats the first
    // sample into every slot, so match that instead of leaving zeros.
    for (int k = 0; k < QUAD_HISTORY; ++k) {
      if (k == s->head) continue;
      if (s->filled == 0) {
        memcpy(s->cmd[k], s->cmd[s->head], sizeof(float) * 3);
        memcpy(s->phase[k], s->phase[s->head], sizeof(float) * 2);
        memcpy(s->qpos[k], s->qpos[s->head], sizeof(float) * QUAD_NJ);
        memcpy(s->qvel[k], s->qvel[s->head], sizeof(float) * QUAD_NJ);
        memcpy(s->act[k], s->act[s->head], sizeof(float) * QUAD_NJ);
      }
    }
    s->filled++;
  }
}

// Flatten one term's history oldest-to-newest into obs[off..].
static inline void quad__flat(const float *ring, int n, int head, int stride,
                              float *out) {
  for (int k = 0; k < QUAD_HISTORY; ++k) {
    int idx = (head + 1 + k) % QUAD_HISTORY;  // oldest first
    memcpy(out + (size_t)k * n, ring + (size_t)idx * stride, sizeof(float) * n);
  }
}

// The gait clock. Must match rl/slow_robot.py::phase: a global sin/cos ramp of
// period QUAD_GAIT_PERIOD, forced to zero whenever |command| is below
// QUAD_PHASE_CMD_MIN (that is how the policy is told "stand still"). The
// exporter writes the threshold the policy was trained with; headers from
// before it did were all trained with 0.1.
#ifndef QUAD_PHASE_CMD_MIN
#define QUAD_PHASE_CMD_MIN 0.1f
#endif
static inline void quad_phase(float t, const float cmd[3], float out[2]) {
  float norm = sqrtf(cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2]);
  if (norm < QUAD_PHASE_CMD_MIN) {
    out[0] = 0.0f;
    out[1] = 0.0f;
    return;
  }
  float ph = fmodf(t, QUAD_GAIT_PERIOD) / QUAD_GAIT_PERIOD;
  out[0] = sinf(ph * QUAD_TWO_PI);
  out[1] = cosf(ph * QUAD_TWO_PI);
}

// One control step.
//   q       : measured joint angles [rad], in QUAD_ACT_DIM order (see the header)
//   qd      : measured joint velocities [rad/s], same order
//   cmd     : [vx, vy, wz]; vy must be 0 -- this robot has no lateral DoF
//   imu     : body-frame rates / gravity direction. Must be non-NULL when the
//             policy was trained with an IMU (QUAD_HAS_ANG_VEL or
//             QUAD_HAS_GRAVITY); ignored, and may be NULL, when it was not.
//   step_i  : control step counter since quad_init (drives the gait clock)
//   target  : out, joint position targets [rad]
//   obs_out : optional, the assembled QUAD_OBS_DIM observation (NULL to skip)
static inline void quad_step(quad_state_t *s, const float q[QUAD_NJ],
                             const float qd[QUAD_NJ], const float cmd[3],
                             const quad_imu_t *imu,
                             long step_i, float target[QUAD_NJ],
                             float *obs_out) {
  float qpos_rel[QUAD_NJ], phase[2], obs[QUAD_OBS_DIM], act[QUAD_ACT_DIM];
  for (int j = 0; j < QUAD_NJ; ++j) qpos_rel[j] = q[j] - QUAD_DEFAULT_JOINT_POS[j];

  s->t = (float)step_i * QUAD_CONTROL_DT;
  quad_phase(s->t, cmd, phase);
  quad__push(s, cmd, qpos_rel, qd, s->last_act, phase, imu);

  // Offsets come from arduino_quad_policy.h, which reads them out of the
  // trained checkpoint. They are NOT computed here: adding an IMU term puts a
  // term at offset 0 and shifts every other one, and code that assumed a fixed
  // order would keep filling the old slots without any error.
  int h = s->head;
#if QUAD_HAS_ANG_VEL
  quad__flat(&s->angvel[0][0], 3, h, 3, obs + QUAD_OFF_ANG_VEL);
#endif
#if QUAD_HAS_GRAVITY
  quad__flat(&s->grav[0][0], 3, h, 3, obs + QUAD_OFF_GRAVITY);
#endif
  quad__flat(&s->cmd[0][0], 3, h, 3, obs + QUAD_OFF_COMMAND);
  quad__flat(&s->phase[0][0], 2, h, 2, obs + QUAD_OFF_PHASE);
  quad__flat(&s->qpos[0][0], QUAD_NJ, h, QUAD_NJ, obs + QUAD_OFF_JOINT_POS);
  quad__flat(&s->qvel[0][0], QUAD_NJ, h, QUAD_NJ, obs + QUAD_OFF_JOINT_VEL);
  quad__flat(&s->act[0][0], QUAD_NJ, h, QUAD_NJ, obs + QUAD_OFF_ACTIONS);

  quad_policy_forward(obs, act);

  // Runtime, not #if: QUAD_ACTION_CLIP is a float literal and the preprocessor
  // cannot compare those. The compiler folds this away when the clip is 0.
  if (QUAD_ACTION_CLIP > 0.0f) {
    for (int j = 0; j < QUAD_ACT_DIM; ++j) {
      if (act[j] > QUAD_ACTION_CLIP) act[j] = QUAD_ACTION_CLIP;
      if (act[j] < -QUAD_ACTION_CLIP) act[j] = -QUAD_ACTION_CLIP;
    }
  }

  for (int j = 0; j < QUAD_NJ; ++j) {
    target[j] = QUAD_DEFAULT_JOINT_POS[j] + QUAD_ACTION_SCALE[j] * act[j];
  }
  memcpy(s->last_act, act, sizeof(act));
  if (obs_out) memcpy(obs_out, obs, sizeof(obs));
}
