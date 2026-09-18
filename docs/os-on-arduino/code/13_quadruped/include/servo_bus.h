// シリアルバスサーボの最小ドライバ (本文 13.2.2)
//
// PWM と違って返事が返る。返事が返るということは、返らなかったときにどうする
// かを決めなければならないということで、そこから 20 ms の締切の話が始まる。
#pragma once

#include <stdbool.h>

void  servo_bus_init(void);

// 応答した個数。起動時に 8 個そろわなければ先へ進まないこと。
int   servo_bus_ping_all(void);

// 速度制限のレジスタを書いて読み返す。戻り値は成立した書き込みの数 (最大 16)。
// SRAM にあるので電源のたびに消える。詳しくは本文 13.9.1。
int   servo_bus_apply_limits(void);

void  servo_bus_torque(bool on);

// 全関節を 1 回の往復で読む。1 個でも黙っていたら false。
bool  servo_bus_read(float *q_rad);

// 8 個ぶんの目標角を 1 パケットで送る (sync write)。返事は無い。
void  servo_bus_write(const float *q_rad);
