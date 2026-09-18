// 第13章 四脚ロボット — 50 Hz の制御ループ
//
//   iktest        FK/IK の往復テスト (脚は動かない)
//   stand         home 姿勢へ 2 秒かけて立ち上がり、保持する
//   trot          naive 歩行 (本文 13.5)
//   rl <vx> <wz>  学習済みポリシーで歩く (本文 13.6, 13.7)
//   stop          ポリシーを切り離して home 姿勢を保持
//   free          トルクを切る
//
// 安全: サーボの電源はバッテリから取ること。最初の trot と rl は必ず機体を
// 吊るすか支えた状態で。
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "leg_ik.h"
#include "servo_bus.h"
#include "quad_calib.h"
#include "quad_control.h"

#define NJ           QUAD_NUM_JOINTS
#define CONTROL_US   ((unsigned long)(QUAD_CONTROL_DT * 1e6f))

extern void gait_step(float dt, float *q_out);
extern void gait_reset(void);

enum Mode { MODE_FREE, MODE_HOLD, MODE_TROT, MODE_RL };

static Mode          g_mode = MODE_HOLD;
static quad_state_t  g_policy;
static float         g_q[NJ], g_q_prev[NJ], g_qd[NJ], g_target[NJ];
static float         g_cmd[3] = {0.0f, 0.0f, 0.0f};
static unsigned long g_step = 0, g_skipped = 0, g_read_fail = 0;
static unsigned long g_last_read_us = 0;
static float         g_hold_from[NJ];
static int           g_hold_k = 0;

static void begin_hold(void)
{
    memcpy(g_hold_from, g_target, sizeof(g_hold_from));
    g_hold_k = 0;
}

// 停止はポリシーの整定に任せてはいけない。速度 0 を指令してもネットは動き続け、
// 実機ではその場で足踏みを続けることがある。ここはポリシーを迂回して、関節目標を
// home へ直接歩かせる。
static void hold_target(float *out)
{
    const int steps = 100;                       // 2 秒
    float a = (g_hold_k >= steps) ? 1.0f : (float)g_hold_k / steps;
    for (int i = 0; i < NJ; ++i)
        out[i] = g_hold_from[i] + a * (QUAD_DEFAULT_JOINT_POS[i] - g_hold_from[i]);
    if (g_hold_k < steps) g_hold_k++;
}

static void shell(char c)
{
    static char buf[32];
    static int len = 0;
    if (c != '\r' && c != '\n') {
        if (len < 31) buf[len++] = c;
        return;
    }
    buf[len] = '\0';
    int n = len;
    len = 0;
    if (n == 0) return;

    if (!strcmp(buf, "iktest")) {
        test_ik_roundtrip();
    } else if (!strcmp(buf, "stand") || !strcmp(buf, "stop")) {
        servo_bus_torque(true);
        begin_hold();
        g_mode = MODE_HOLD;
        Serial.println(F("mode=hold"));
    } else if (!strcmp(buf, "trot")) {
        servo_bus_torque(true);
        gait_reset();
        g_mode = MODE_TROT;
        Serial.println(F("mode=trot"));
    } else if (!strncmp(buf, "rl", 2)) {
        float vx = 0.0f, wz = 0.0f;
        sscanf(buf + 2, "%f %f", &vx, &wz);
        g_cmd[0] = vx; g_cmd[1] = 0.0f; g_cmd[2] = wz;
        servo_bus_torque(true);
        quad_init(&g_policy);
        g_mode = MODE_RL;
        Serial.print(F("mode=rl vx=")); Serial.print(vx, 2);
        Serial.print(F(" wz=")); Serial.println(wz, 2);
    } else if (!strcmp(buf, "free")) {
        servo_bus_torque(false);
        g_mode = MODE_FREE;
        Serial.println(F("mode=free"));
    } else {
        Serial.print(F("unknown: ")); Serial.println(buf);
    }
}

void setup()
{
    Serial.begin(115200);
    servo_bus_init();

    int found = servo_bus_ping_all();
    Serial.print(F("サーボ応答: ")); Serial.print(found);
    Serial.print(F(" / ")); Serial.println(NJ);

    // 速度レジスタは SRAM にあるので電源のたびに出荷値へ戻る。何も報告されず、
    // ロボットが遅くなるだけなので、開いたら必ず書く (補遺 13.9.1)。
    int limits = servo_bus_apply_limits();
    Serial.print(F("速度設定: ")); Serial.print(limits);
    Serial.print(F(" / ")); Serial.print(NJ * 2);
    Serial.println(limits == NJ * 2 ? F(" 適用") : F(" !! 一部失敗 (遅く歩きます)"));

    if (!servo_bus_read(g_q)) Serial.println(F("!! 起動時の関節読み取りに失敗"));
    for (int i = 0; i < NJ; ++i) {
        g_q_prev[i] = g_q[i];
        g_qd[i] = 0.0f;
        g_target[i] = g_q[i];
    }
    quad_init(&g_policy);
    servo_bus_torque(true);
    begin_hold();

    // home の値はコメントではなくこの出力が正。立ち高さを変えて再学習すると
    // 変わるので、脚の格好はここに出る数字と突き合わせること (補遺 13.8.1)。
    Serial.print(F("home rad:"));
    for (int i = 0; i < NJ; ++i) {
        Serial.print(' ');
        Serial.print(QUAD_DEFAULT_JOINT_POS[i], 4);
    }
    Serial.println();
    Serial.println(F("commands: iktest / stand / trot / rl <vx> <wz> / stop / free"));
}

void loop()
{
    static unsigned long next_us = 0;
    static unsigned long report_us = 0;
    static unsigned long loop_sum = 0, loop_max = 0, loop_n = 0, late_n = 0;

    while (Serial.available()) shell((char)Serial.read());

    if (next_us == 0) next_us = micros();
    if ((long)(micros() - next_us) < 0) return;
    unsigned long t0 = micros();

    if (g_mode == MODE_FREE) {
        next_us = micros() + CONTROL_US;
        return;
    }

    if (!servo_bus_read(g_q)) {
        g_read_fail++;
    } else {
        // 公称周期ではなく実経過時間で割る。周期が乱れているときこそ関節速度が
        // 効くのに、そこで一番大きな誤差が乗る (本文 13.4.5)。
        unsigned long now = micros();
        float dt = (g_last_read_us == 0) ? QUAD_CONTROL_DT
                                         : (float)(now - g_last_read_us) * 1e-6f;
        g_last_read_us = now;
        for (int i = 0; i < NJ; ++i) {
            g_qd[i] = 0.6f * g_qd[i] + 0.4f * ((g_q[i] - g_q_prev[i]) / dt);
            g_q_prev[i] = g_q[i];
        }

        if (g_mode == MODE_HOLD)      hold_target(g_target);
        else if (g_mode == MODE_TROT) gait_step(QUAD_CONTROL_DT, g_target);
        else                          quad_step(&g_policy, g_q, g_qd, g_cmd,
                                                NULL, (long)g_step, g_target, NULL);
        servo_bus_write(g_target);
    }
    g_step++;

    unsigned long spent = micros() - t0;
    loop_sum += spent; loop_n++;
    if (spent > loop_max) loop_max = spent;
    if (spent > CONTROL_US) late_n++;

    if (micros() - report_us > 1000000UL) {
        report_us = micros();
        Serial.print(F("loop avg ")); Serial.print(loop_n ? loop_sum / loop_n : 0);
        Serial.print(F(" us  max ")); Serial.print(loop_max);
        Serial.print(F(" us  late ")); Serial.print(late_n);
        Serial.print('/'); Serial.print(loop_n);
        Serial.print(F("  skipped ")); Serial.print(g_skipped);
        Serial.print(F("  read_fail ")); Serial.println(g_read_fail);
        loop_sum = loop_n = loop_max = late_n = 0;
    }

    next_us += CONTROL_US;
    long behind = (long)(micros() - next_us);
    if (behind > 0) {
        // 落とした周期を連続実行しても時間は戻らず、歩容クロックだけが遅れる。
        // 歩容の位相は step_i * dt で作られているので、遅れを後から詰めようと
        // すると位相が実時間から離れていく。落とした分は捨てて、step カウンタを
        // 進めて実時間に合わせ直す (本文 13.4.4)。
        long lost = behind / (long)CONTROL_US + 1;
        g_step += lost;
        g_skipped += lost;
        next_us += (unsigned long)lost * CONTROL_US;
    }
}
