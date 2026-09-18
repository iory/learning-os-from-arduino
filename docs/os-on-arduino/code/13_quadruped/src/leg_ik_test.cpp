// FK と IK は互いに逆向きの計算なので、通して元に戻るかで実装を確かめられる
// (本文 13.3.7)。実機へ載せる前に必ず通しておくこと。
#include <Arduino.h>
#include <math.h>
#include "leg_ik.h"

void test_ik_roundtrip(void)
{
    // (x, r) [m]。x は前が正、r は股軸から下向き
    const float targets[][2] = {
        { 0.000f, 0.1600f},   // ほぼ真下、脚を伸ばし気味
        { 0.040f, 0.1400f},   // 前へ
        {-0.040f, 0.1400f},   // 後ろへ
        { 0.000f, 0.1205f},   // 立ち姿勢 (home) の高さ
        { 0.000f, 0.2500f},   // 届かない位置。切り詰めが効くか
    };

    for (unsigned i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
        float t2, t3, gx, gr;
        leg_ik(targets[i][0], targets[i][1], &t2, &t3);
        leg_fk(t2, t3, &gx, &gr);

        float err = fabsf(gx - targets[i][0]) + fabsf(gr - targets[i][1]);

        Serial.print(F("target=(")); Serial.print(targets[i][0], 3);
        Serial.print(F(", "));       Serial.print(targets[i][1], 4);
        Serial.print(F(")  hip="));  Serial.print(t2 * 57.2958f, 2);
        Serial.print(F("  knee=")); Serial.print(t3 * 57.2958f, 2);
        Serial.print(F("  err="));  Serial.println(err, 6);
    }
    Serial.println(F("err が 0.00001 を下回れば式は正しい。"
                     "最後の 1 行だけは届かない位置なので 0.05 前後になる。"));
}
