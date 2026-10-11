// 第13章 四脚ロボット — 50 Hz の制御ループ
//
//   iktest        FK/IK の往復テスト (脚は動かない)
//   stand         home 姿勢へ 2 秒かけて立ち上がり、保持する
//   trot          naive 歩行 (本文 13.5)
//   rl <vx> <wz>  学習済みポリシーで歩く (本文 13.6, 13.7)
//   drive <vx> <wz>
//                 rl と同じだが、300 ms 途切れると home 保持に戻る。
//                 PC のキーボード操作 (host/quad_host.py serial) が送り続ける
//   stop          ポリシーを切り離して home 姿勢を保持。IMU を使う方策では
//                 ポリシーを速度 0 で回し続け、押されても IMU を見て踏ん張る
//   zero          全関節を 0 度 (脚をまっすぐ) へ移して保持。吊るして使う
//   free          トルクを切る
//   raw           (RCB-4 版だけ) 関節ごとのサーボ番号とパルスを出す。校正に使う
//   imu           (IMU を使う方策だけ) 胴体座標の角速度と重力の向き、関節の目標角を出す
//
// 安全: サーボの電源はバッテリから取ること。最初の trot と rl は必ず機体を
// 吊るすか支えた状態で。
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "leg_ik.h"
#include "servo_bus.h"
#include "quad_calib.h"
#include "quad_control.h"
#include "quad_imu.h"

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
#if QUAD_USES_IMU
// 方策が IMU を観測に使うときだけ読む。IMU が無い (起動時に見つからない・校正に
// 失敗した) ときと読めなかった周期は、水平に止まっている値を渡して歩かせる。
// そのときは傾いても方策には伝わらないので、毎秒の行に imu=level と出す。
static quad_imu_t    g_imu;
static bool          g_imu_ok = false;
static unsigned long g_imu_fail = 0;
#endif
static float         g_hold_from[NJ];
static const float  *g_hold_goal = QUAD_DEFAULT_JOINT_POS;
static int           g_hold_k = 0;

// zero で向かう姿勢。サーボの 0 点 (2048) を合わせた、脚がまっすぐの姿勢。
static const float   ZERO_POSE[NJ] = {0};

// drive はこの間隔より長く途切れたら止まる。PC 側が固まったりケーブルが
// 抜けたりしても、最後の速度で歩き続けないように。
#define DRIVE_TIMEOUT_MS 300UL
static bool          g_drive_armed = false;
static unsigned long g_last_drive_ms = 0;

static void begin_hold(const float *goal)
{
    memcpy(g_hold_from, g_target, sizeof(g_hold_from));
    g_hold_goal = goal;
    g_hold_k = 0;
}

// 停止はポリシーの整定に任せてはいけない。速度 0 を指令してもネットは動き続け、
// 実機ではその場で足踏みを続けることがある。ここはポリシーを迂回して、関節目標を
// home (または 0 度) へ直接歩かせる。
static void hold_target(float *out)
{
    const int steps = 100;                       // 2 秒
    float a = (g_hold_k >= steps) ? 1.0f : (float)g_hold_k / steps;
    for (int i = 0; i < NJ; ++i)
        out[i] = g_hold_from[i] + a * (g_hold_goal[i] - g_hold_from[i]);
    if (g_hold_k < steps) g_hold_k++;
}

// rl と drive の共通部分。歩いている最中なら速度だけを差し替え、ポリシーの
// 内部状態 (直前の行動と歩容の位相) は初期化しない。キーを押すたびに初期化
// すると、そのたびに歩容が頭から始まってつまずく。
// 歩き始めたときは true を返す。
static bool set_walk_command(const char *args, const __FlashStringHelper *name)
{
    // sscanf の %f は使わない。Arduino の newlib-nano は -u _scanf_float を
    // リンクしないと浮動小数を読まず、黙って 0 のまま返す。実機で
    // "rl 0.15 0" が vx=0.00 になり、ポリシーは速度 0 で回り続けた。
    // strtof はリンク指定に関係なく読める。
    char *rest;
    float vx = strtof(args, &rest);
    float wz = strtof(rest, NULL);
    g_cmd[0] = vx; g_cmd[1] = 0.0f; g_cmd[2] = wz;
    if (g_mode != MODE_RL) {
        servo_bus_torque(true);
        quad_init(&g_policy);
        g_mode = MODE_RL;
        Serial.print(F("mode=rl vx=")); Serial.print(vx, 2);
        Serial.print(F(" wz=")); Serial.print(wz, 2);
        Serial.print(F(" (")); Serial.print(name); Serial.println(')');
        return true;
    }
    return false;
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
#if QUAD_USES_IMU
    } else if (!strcmp(buf, "stop") && g_imu_ok) {
        // 歩いた後の停止も IMU のフィードバックを切らない。速度 0 の指令は
        // 位相のしきい値 (QUAD_PHASE_CMD_MIN) を下回るので、ポリシーは足踏みを
        // やめて立つ。歩行中なら内部状態はそのままで、指令だけを 0 にする。
        // 見張りは外すので、PC が送るのをやめても立ったまま踏ん張り続ける。
        g_drive_armed = false;
        g_cmd[0] = g_cmd[1] = g_cmd[2] = 0.0f;
        if (g_mode != MODE_RL) {
            servo_bus_torque(true);
            quad_init(&g_policy);
            g_mode = MODE_RL;
        }
        Serial.println(F("mode=balance (IMU で姿勢を保つ。固定するなら stand)"));
#endif
    } else if (!strcmp(buf, "stand") || !strcmp(buf, "stop")) {
        g_drive_armed = false;
        servo_bus_torque(true);
        begin_hold(QUAD_DEFAULT_JOINT_POS);
        g_mode = MODE_HOLD;
        Serial.println(F("mode=hold"));
    } else if (!strcmp(buf, "zero")) {
        g_drive_armed = false;
        servo_bus_torque(true);
        begin_hold(ZERO_POSE);
        g_mode = MODE_HOLD;
        Serial.println(F("mode=zero"));
    } else if (!strcmp(buf, "trot")) {
        g_drive_armed = false;
        servo_bus_torque(true);
        gait_reset();
        g_mode = MODE_TROT;
        Serial.println(F("mode=trot"));
    } else if (!strncmp(buf, "drive", 5)) {
        // 送り続けられる前提なので、歩き始めた 1 回だけ返事をする。
        set_walk_command(buf + 5, F("drive"));
        g_drive_armed = true;
        g_last_drive_ms = millis();
    } else if (!strncmp(buf, "rl", 2)) {
        // 手で打つ前提なので見張りは掛けない。
        g_drive_armed = false;
        if (!set_walk_command(buf + 2, F("rl"))) {
            Serial.print(F("cmd vx=")); Serial.print(g_cmd[0], 2);
            Serial.print(F(" wz=")); Serial.println(g_cmd[2], 2);
        }
#if QUAD_USES_IMU
    } else if (!strcmp(buf, "imu")) {
        if (!g_imu_ok) Serial.print(F("(IMU 無し。水平の値を渡している) "));
        quad_imu_print(&g_imu);
        // IMU が方策に届いているかは、balance 中に傾けてこの値が変わるかで見る。
        Serial.print(F("target rad:"));
        for (int i = 0; i < NJ; ++i) {
            Serial.print(' ');
            Serial.print(g_target[i], 3);
        }
        Serial.println();
#endif
#ifdef QUAD_SERVO_RCB4
    } else if (!strcmp(buf, "raw")) {
        servo_bus_print_raw();
#endif
    } else if (!strcmp(buf, "free")) {
        g_drive_armed = false;
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

#ifdef QUAD_SERVO_RCB4
    // RCB-4 には速度上限のレジスタが無い。速度は書き込みごとの補間フレーム数で決まる。
    servo_bus_apply_limits();
    Serial.println(F("速度設定: RCB-4 には無い (補間 1 フレームで送る)"));
#else
    // 速度レジスタは SRAM にあるので電源のたびに出荷値へ戻る。何も報告されず、
    // ロボットが遅くなるだけなので、開いたら必ず書く (補遺 13.9.1)。
    int limits = servo_bus_apply_limits();
    Serial.print(F("速度設定: ")); Serial.print(limits);
    Serial.print(F(" / ")); Serial.print(NJ * 2);
    Serial.println(limits == NJ * 2 ? F(" 適用") : F(" !! 一部失敗 (遅く歩きます)"));
#endif

#if QUAD_USES_IMU
    // ジャイロのゼロ点を取るので、トルクを入れる (脚が動く) 前に、静止した状態で。
    Serial.println(F("IMU: 起動中。約 10 秒、ロボットを動かさないこと"));
    quad_imu_level(&g_imu);
    if (!quad_imu_begin()) {
        Serial.println(F("!! IMU (QMI8658, 0x6B) が見つかりません。Qwiic の配線を確認"));
    } else if (!quad_imu_calibrate(1000)) {
        Serial.println(F("!! IMU の校正に失敗 (動いていた?)。静止させて再起動"));
    } else {
        g_imu_ok = true;
        quad_imu_update(QUAD_CONTROL_DT, &g_imu);
        Serial.print(F("IMU: "));
        quad_imu_print(&g_imu);
    }
    if (!g_imu_ok)
        Serial.println(F("!! IMU 無しで動かす: 方策には水平に止まっている値を渡す。"
                         "傾いても押されても方策には伝わらない"));
#endif

    if (!servo_bus_read(g_q)) Serial.println(F("!! 起動時の関節読み取りに失敗"));
    for (int i = 0; i < NJ; ++i) {
        g_q_prev[i] = g_q[i];
        g_qd[i] = 0.0f;
        g_target[i] = g_q[i];
    }
    quad_init(&g_policy);
    servo_bus_torque(true);
    begin_hold(QUAD_DEFAULT_JOINT_POS);

    // home の値はコメントではなくこの出力が正。立ち高さを変えて再学習すると
    // 変わるので、脚の格好はここに出る数字と突き合わせること (補遺 13.8.1)。
    Serial.print(F("home rad:"));
    for (int i = 0; i < NJ; ++i) {
        Serial.print(' ');
        Serial.print(QUAD_DEFAULT_JOINT_POS[i], 4);
    }
    Serial.println();
    Serial.println(F("commands: iktest / stand / trot / rl <vx> <wz> / drive <vx> <wz> / stop / zero / free"));
#if QUAD_USES_IMU
    Serial.println(F("          imu"));
#endif
}

void loop()
{
    static unsigned long next_us = 0;
    static unsigned long report_us = 0;
    static unsigned long loop_sum = 0, loop_max = 0, loop_n = 0, late_n = 0;

    while (Serial.available()) shell((char)Serial.read());

    if (g_drive_armed && g_mode == MODE_RL &&
        millis() - g_last_drive_ms > DRIVE_TIMEOUT_MS) {
        g_drive_armed = false;
        begin_hold(QUAD_DEFAULT_JOINT_POS);
        g_mode = MODE_HOLD;
        Serial.println(F("mode=hold (drive が途切れた)"));
    }

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
#if QUAD_USES_IMU
        // 重力の向きのフィルタは止めずに回し続ける (歩き出した瞬間に古い値を使わない)。
        if (g_imu_ok && !quad_imu_update(dt, &g_imu)) {
            g_imu_fail++;
            quad_imu_level(&g_imu);
        }
        const quad_imu_t *imu = &g_imu;
#else
        const quad_imu_t *imu = NULL;
#endif

        if (g_mode == MODE_HOLD)      hold_target(g_target);
        else if (g_mode == MODE_TROT) gait_step(QUAD_CONTROL_DT, g_target);
        else                          quad_step(&g_policy, g_q, g_qd, g_cmd,
                                                imu, (long)g_step, g_target, NULL);
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
        Serial.print(F("  read_fail ")); Serial.print(g_read_fail);
#if QUAD_USES_IMU
        Serial.print(F("  imu_fail ")); Serial.print(g_imu_fail);
        if (!g_imu_ok) Serial.print(F("  imu=level"));
#endif
#ifdef QUAD_SERVO_RCB4
        Serial.print(F("  write_fail ")); Serial.print(servo_bus_write_fail());
#endif
        Serial.println();
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
