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

#ifdef QUAD_SERVO_RCB4
// RCB-4 mini 版 (servo_bus_rcb4.cpp) だけにある。RCB-4 は書き込みにも ACK を
// 返すので、返らなかった回数を数えられる。apply_limits は書くものが無いので 0。
unsigned long servo_bus_write_fail(void);

// 関節ごとの ID・RCB-4 のサーボ番号・パルス・角度を出す (コマンド raw)。
// quad_calib_rcb4.h のゼロ点と向きはこれで測る。
void servo_bus_print_raw(void);
#endif
