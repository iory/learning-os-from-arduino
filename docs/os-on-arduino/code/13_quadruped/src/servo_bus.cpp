#include <Arduino.h>
#include "servo_bus.h"
#include "quad_calib.h"

#define BUS          Serial1        // D0(RX) / D1(TX)
#define BUS_BAUD     1000000        // 本文 13.2.2: 50 Hz 制御にはこれが要る
#define INST_PING    0x01
#define INST_READ    0x02
#define INST_WRITE   0x03
#define INST_SYNC    0x83
#define ADDR_TORQUE  40
#define ADDR_ACC     41             // 加速度 (1) の直後に目標位置 (2)・時間 (2)・速度 (2)
#define ADDR_POS     56
#define SYNC_LEN     7              // ADDR_ACC から 7 バイト = 41..47

// データシートに無い SRAM の速度上限。出荷値は 50 と 1 で、無負荷速度の 21%
// しか出ない (実測 1.00 rad/s に対しポリシーが要求するのは 5.34 rad/s)。SRAM な
// ので電源を切ると戻る。何も報告されず、ロボットが遅くなるだけ。補遺 13.9.1。
#define ADDR_MAX_ACC   85
#define ADDR_ACC_MULT  86
#define VAL_MAX_ACC   254
#define VAL_ACC_MULT    0

#define NJ QUAD_NUM_JOINTS
#define RAD_PER_COUNT (6.28318530717958647692f / 4096.0f)

static uint8_t checksum(const uint8_t *b, int n)
{
    int s = 0;
    for (int i = 0; i < n; ++i) s += b[i];
    return (uint8_t)(~s);
}

// 1 往復。戻り値は受け取ったパラメータ数、負なら失敗。
//
// timeout_us は制御周期より必ず短くすること。既定 100 ms のライブラリをその
// まま使うと、サーボ 1 個が黙るだけで制御周期 5 回分を失う (本文 13.4.3)。
static int txrx(uint8_t id, uint8_t inst, const uint8_t *par, int npar,
                uint8_t *out, int outmax, uint32_t timeout_us)
{
    uint8_t pkt[16];
    int n = 0;
    pkt[n++] = 0xFF; pkt[n++] = 0xFF;
    pkt[n++] = id; pkt[n++] = (uint8_t)(npar + 2); pkt[n++] = inst;
    for (int i = 0; i < npar; ++i) pkt[n++] = par[i];
    pkt[n] = checksum(pkt + 2, n - 2); n++;

    // このバスは半二重なので、アダプタによっては自分の送信がそのまま受信
    // バッファに現れる。ヘッダを探し直すことでそれを読み飛ばす (本文 13.2.2)。
    while (BUS.available()) BUS.read();
    BUS.write(pkt, n);
    BUS.flush();

    uint32_t t0 = micros();
    int prev = -1, cur = -1;
    while (micros() - t0 < timeout_us) {
        if (!BUS.available()) continue;
        prev = cur; cur = BUS.read();
        if (prev == 0xFF && cur == 0xFF) break;
    }
    if (!(prev == 0xFF && cur == 0xFF)) return -1;

    uint8_t head[2];
    for (int i = 0; i < 2; ++i) {
        while (!BUS.available()) if (micros() - t0 > timeout_us) return -2;
        head[i] = BUS.read();
    }
    int len = head[1];
    if (len < 2 || len > 32) return -3;
    uint8_t body[34];
    for (int i = 0; i < len; ++i) {
        while (!BUS.available()) if (micros() - t0 > timeout_us) return -2;
        body[i] = BUS.read();
    }
    uint8_t sum[36];
    sum[0] = head[0]; sum[1] = head[1];
    for (int i = 0; i < len; ++i) sum[2 + i] = body[i];
    if (checksum(sum, len + 1) != body[len - 1]) return -4;
    if (head[0] != id) return -5;

    int np = len - 2;
    for (int i = 0; i < np && i < outmax; ++i) out[i] = body[1 + i];
    return np;
}

void servo_bus_init(void)
{
    BUS.begin(BUS_BAUD);
    delay(200);
}

int servo_bus_ping_all(void)
{
    int found = 0;
    for (int i = 0; i < NJ; ++i) {
        uint8_t o[4];
        if (txrx(QUAD_SERVO_ID[i], INST_PING, NULL, 0, o, 4, 3000) >= 0) found++;
    }
    return found;
}

// 書いて読み返す。応答が無かった書き込みと、届かなかった書き込みは区別が
// つかない -- 黙って適用されない速度上限こそが、この関数の存在理由。
static bool write_verify(uint8_t id, uint8_t addr, uint8_t value, int tries)
{
    for (int t = 0; t < tries; ++t) {
        uint8_t p[2] = {addr, value}, o[4];
        txrx(id, INST_WRITE, p, 2, o, 4, 3000);
        uint8_t r[2] = {addr, 1};
        if (txrx(id, INST_READ, r, 2, o, 1, 3000) == 1 && o[0] == value) return true;
        delay(20);
    }
    return false;
}

int servo_bus_apply_limits(void)
{
    int ok = 0;
    for (int i = 0; i < NJ; ++i) {
        if (write_verify(QUAD_SERVO_ID[i], ADDR_MAX_ACC,  VAL_MAX_ACC,  6)) ok++;
        if (write_verify(QUAD_SERVO_ID[i], ADDR_ACC_MULT, VAL_ACC_MULT, 6)) ok++;
    }
    return ok;
}

void servo_bus_torque(bool on)
{
    for (int i = 0; i < NJ; ++i) {
        uint8_t p[2] = {ADDR_TORQUE, (uint8_t)(on ? 1 : 0)}, o[4];
        txrx(QUAD_SERVO_ID[i], INST_WRITE, p, 2, o, 2, 3000);
    }
}

bool servo_bus_read(float *q_rad)
{
    bool ok = true;
    for (int i = 0; i < NJ; ++i) {
        uint8_t p[2] = {ADDR_POS, 2}, o[2];
        if (txrx(QUAD_SERVO_ID[i], INST_READ, p, 2, o, 2, 3000) != 2) {
            ok = false;
            continue;
        }
        float counts = (float)(o[0] | (o[1] << 8));
        q_rad[i] = (counts - QUAD_SERVO_ZERO[i]) * RAD_PER_COUNT * QUAD_SERVO_SIGN[i];
    }
    return ok;
}

void servo_bus_write(const float *q_rad)
{
    // 8 個ぶんを 1 パケットで送る。1 個ずつ書くと往復が 8 倍になり、
    // 20 ms の予算が崩れる (本文 13.4.2)。
    uint8_t pkt[8 + NJ * (SYNC_LEN + 1)];
    int k = 0;
    pkt[k++] = 0xFF; pkt[k++] = 0xFF;
    pkt[k++] = 0xFE;                                   // broadcast
    pkt[k++] = (uint8_t)((SYNC_LEN + 1) * NJ + 4);
    pkt[k++] = INST_SYNC;
    // 加速度 (41) から書き始める。以前は 42 (目標位置) から 7 バイト書き、最後の
    // 「加速度」の 0 が実際には 48 番地 = トルク上限の下位バイトに入っていた。
    // 出荷値 1000 (0x03E8) が 768 (0x0300) に落ち、荷重のかかる脚が沈んで転ぶ。
    // 加速度は一度も書かれず、サーボに残っていた値のまま動いていた。
    pkt[k++] = ADDR_ACC;
    pkt[k++] = SYNC_LEN;
    for (int i = 0; i < NJ; ++i) {
        float counts = q_rad[i] / RAD_PER_COUNT * QUAD_SERVO_SIGN[i]
                     + QUAD_SERVO_ZERO[i];
        if (counts < 0.0f) counts = 0.0f;
        if (counts > 4095.0f) counts = 4095.0f;
        int c = (int)(counts + 0.5f);
        pkt[k++] = QUAD_SERVO_ID[i];
        pkt[k++] = 0;                                  // 加速度 0 = 上限なし
        pkt[k++] = (uint8_t)(c & 0xFF);                // 目標位置
        pkt[k++] = (uint8_t)((c >> 8) & 0xFF);
        pkt[k++] = 0; pkt[k++] = 0;                    // 時間
        pkt[k++] = 0; pkt[k++] = 0;                    // 速度 0 = 上限なし
    }
    pkt[k] = checksum(pkt + 2, k - 2); k++;
    BUS.write(pkt, k);
    BUS.flush();
}
