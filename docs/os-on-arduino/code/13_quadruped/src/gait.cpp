// naive 歩行: 状態機械 + trot gait (本文 13.5)
//
// ポリシーを使わずに歩かせる側。平地では動くが、この機体は前後が非対称なので
// 向きによって性能が倍以上違う (0.156 m/s と 0.089 m/s)。係数を手で回して
// どちらかには合わせられるが、両方には合わせられない。そこが RL へ行く動機。
#include <Arduino.h>
#include <math.h>
#include "leg_ik.h"
#include "quad_calib.h"

#define GAIT_PERIOD_MS  320      // 学習したポリシーと同じ歩容周期
#define LEG_FL 0
#define LEG_RL 1
#define LEG_RR 2
#define LEG_FR 3

static float STEP_X   = 0.06f;   // 前進ストライド長 [m]
static float STEP_H   = 0.030f;  // 遊脚の持ち上げ [m]
static float STANCE_R = 0.1205f; // 接地時の股軸からの下向き距離 [m]

static float g_phase = 0.0f;     // 0..1

// 各脚の位相オフセット。対角の 2 本が同じ値になるのが trot。
// 並びは quad_calib.h の関節順 (FL, RL, RR, FR) に対応する。
static const float PHASE_OFFSET[4] = {0.0f, 0.5f, 0.0f, 0.5f};

// 遊脚は前へ出しながら山なりに持ち上がり、接地脚は体の下を後ろへ流れる。
// 外転が無いので横方向は無く、2 変数で足りる (本文 13.5.2)。
static void compute_swing_foot(float phi, float *fx, float *fr)
{
    if (phi < 0.5f) {
        float t = phi * 2.0f;
        *fx = (t - 0.5f) * STEP_X;
        *fr = STANCE_R - STEP_H * sinf((float)M_PI * t);
    } else {
        float t = (phi - 0.5f) * 2.0f;
        *fx = (0.5f - t) * STEP_X;
        *fr = STANCE_R;
    }
}

void gait_reset(void) { g_phase = 0.0f; }

void gait_set_stride(float step_x) { STEP_X = step_x; }

// dt [s] ぶん位相を進めて、8 関節の目標角を書く。
void gait_step(float dt, float *q_out)
{
    g_phase = fmodf(g_phase + dt * 1000.0f / GAIT_PERIOD_MS, 1.0f);
    for (int leg = 0; leg < 4; ++leg) {
        float phi = fmodf(g_phase + PHASE_OFFSET[leg], 1.0f);
        float fx, fr;
        compute_swing_foot(phi, &fx, &fr);
        leg_ik(fx, fr, &q_out[leg * 2 + 0], &q_out[leg * 2 + 1]);
    }
}
