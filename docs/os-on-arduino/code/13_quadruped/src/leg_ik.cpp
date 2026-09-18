#include <math.h>
#include "leg_ik.h"

// CAD から測った値。股軸→膝軸、膝軸→足先球の中心。
static const float L2 = 0.0883f;   // 大腿  88.3 mm
static const float L3 = 0.1087f;   // 下腿 108.7 mm

// 下腿ブラケットの折れ (本文 13.3.5)。膝角 0 でも 4.36 度だけ前に折れている。
// 無視しても脚は動くが、足先が狙いから 8 mm 前へずれる。しかもそのずれは
// ログには出ない -- IK が返した角度にサーボは行き、読み戻せば一致するので、
// 辻褄は完璧に合ったまま足の位置だけが違う。
static const float KNEE_OFFSET = -0.0761f;

// x は前を正、r は股軸から下向きを正。URDF の関節符号は「正で足が後ろへ」
// なので、前後を入れ替えて平面 2 リンクの式へ渡す。ここを合わせておかないと、
// IK は正しい角度を返すのに脚は逆へ振れる。
void leg_fk(float t2, float t3, float *fx, float *fr)
{
    float a = t2;
    float b = t2 + t3 + KNEE_OFFSET;
    *fx = -(L2 * sinf(a) + L3 * sinf(b));
    *fr =   L2 * cosf(a) + L3 * cosf(b);
}

void leg_ik(float fx, float fr, float *t2, float *t3)
{
    float u = -fx;                       // 平面 2 リンクの向きへ直す
    float d2 = u * u + fr * fr;

    // 余弦定理で膝の曲がり角 (本文 13.3.3 手順その1)
    float c = (d2 - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);
    if (c >  1.0f) c =  1.0f;            // 届かないときの保険 (本文 13.3.8)
    if (c < -1.0f) c = -1.0f;

    // acosf が返すのは 0..pi の側だが、この機体の膝は後ろへ折れる。立ち姿勢の
    // 膝角が -101 度である以上、負のほうの解を採るのが正しい (本文 13.3.4)。
    float gamma = -acosf(c);

    // 引き算で股の角度 (本文 13.3.3 手順その2)
    float a = atan2f(u, fr);
    float b = atan2f(L3 * sinf(gamma), L2 + L3 * cosf(gamma));
    *t2 = a - b;
    *t3 = gamma - KNEE_OFFSET;
}
