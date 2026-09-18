// 1 脚 2 自由度の順運動学・逆運動学 (本文 13.3)
//
// この機体の脚は矢状面の中だけで動く (外転軸が無い)。3 次元へ広げる必要が
// ないので、平面 2 リンクの式がそのまま脚の全部になる。
#pragma once

// x は前が正、r は股の軸から下向きが正 [m]。
// t2 は股、t3 は膝の関節角 [rad]。符号は URDF に合わせてある。
void leg_fk(float t2, float t3, float *fx, float *fr);
void leg_ik(float fx, float fr, float *t2, float *t3);

// FK と IK を突き合わせて実装を確かめる (本文 13.3.7)
void test_ik_roundtrip(void);
