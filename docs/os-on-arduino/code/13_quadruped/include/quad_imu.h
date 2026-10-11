// 胴体前面のセンサ基板の IMU (QMI8658) を読み、方策が学習した形
// (胴体座標の角速度と重力の向き) にして返す。
//
// つなぎ方: UNO R4 WiFi の Qwiic コネクタ (Wire1, 3.3 V)。
// 方策が IMU を使わないとき (QUAD_HAS_ANG_VEL も QUAD_HAS_GRAVITY も 0) は
// main.cpp から呼ばれない。
#ifndef QUAD_IMU_H
#define QUAD_IMU_H

#include "quad_control.h"

#define QUAD_USES_IMU (QUAD_HAS_ANG_VEL || QUAD_HAS_GRAVITY)

// チップを探して設定する。見つからない・応答が違うときは false。
// ジャイロが落ち着くまで 5 秒待つので、その間は動かさないこと。
bool quad_imu_begin(void);

// 静止させた状態で n 回読み、ジャイロのゼロ点と重力の向きの初期値を取る。
// 動いていたら (角速度のばらつきが大きいと) false。
bool quad_imu_calibrate(int n);

// 1 制御周期ぶん読んで out を埋める。dt [s] は前回からの実経過時間。
// 読めなかったら false (out は前回の値のまま)。
bool quad_imu_update(float dt, quad_imu_t *out);

// 水平に止まっているときの値 (角速度 0、重力 {0, 0, -1}) で out を埋める。
// IMU が無い・読めないときに方策へ渡す。押されたり傾いたりしても方策には伝わらない。
void quad_imu_level(quad_imu_t *out);

// 生の値 (チップ座標の加速度 [g]・角速度 [rad/s]) と胴体座標の値を 1 行で出す。
void quad_imu_print(const quad_imu_t *body);

#endif
