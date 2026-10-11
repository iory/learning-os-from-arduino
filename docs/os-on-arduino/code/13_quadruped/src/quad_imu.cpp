// 胴体前面のセンサ基板の IMU のドライバと、方策が学習した観測への変換。
//
// 基板の IMU は部品表では ICM-42688-P だが、中身は QST の QMI8658 で、
// QMI8658 のレジスタで動かす (I2C アドレス 0x6B、WHO_AM_I = 0x05)。
// レジスタの番地と意味は QMI8658A データシート (Rev D) の Table 23 / 28:
//   https://www.qstcorp.com/upload/pdf/202512/13-52-25%20QMI8658A%20Datasheet%20Rev%20D.pdf
// 設定値 (レンジ・ODR・LPF 無効・ウォームアップ) は、このチップで実際に
// 動かして確かめたものに合わせている。データシートどおりの設定 (LPF 有効) では
// ノイズが桁違いに大きくなった。
//
// 方策が見るのは学習時の imu_in_body サイトの値、つまり
//   ang_vel  胴体座標 (x 前, y 左, z 上) の角速度 [rad/s]
//   gravity  胴体座標で見た重力の向き (単位ベクトル。水平に立てば {0, 0, -1})
// チップの軸は胴体の軸と向きが違うので、取り付けの向きの回転をかける。
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "quad_imu.h"

#if QUAD_USES_IMU

#define IMU_WIRE      Wire1     // UNO R4 WiFi の Qwiic コネクタ

// レジスタ
#define REG_WHO_AM_I  0x00      // 0x05 が返る
#define REG_CTRL1     0x02
#define REG_CTRL2     0x03      // 加速度: フルスケールと ODR
#define REG_CTRL3     0x04      // ジャイロ: フルスケールと ODR
#define REG_CTRL5     0x06      // ローパスフィルタ
#define REG_CTRL7     0x08      // センサの有効化
#define REG_AX_L      0x35      // AX_L から GZ_H まで 12 バイト連続
#define REG_RESET     0x60      // 0xB0 を書くとソフトリセット (最大 15 ms)

#define WHO_AM_I_VAL  0x05

// CTRL1: bit6 ADDR_AI (アドレス自動インクリメント) = 1、bit5 BE = 0 (リトルエンディアン)
#define CTRL1_VAL     0x40
// CTRL2: aFS = 010 (±8 g)、aODR = 0011 (6DOF で 896.8 Hz)
#define CTRL2_VAL     0x23
// CTRL3: gFS = 111 (±2048 dps)、gODR = 0011 (896.8 Hz)
#define CTRL3_VAL     0x73
// CTRL5: LPF は加速度・ジャイロとも無効
#define CTRL5_VAL     0x00
// CTRL7: gEN = 1、aEN = 1 (bit7 の SyncSample は立てない)
#define CTRL7_VAL     0x03

#define ACC_LSB_PER_G     4096.0f   // ±8 g
#define GYR_LSB_PER_DPS   16.0f     // ±2048 dps
#define DEG2RAD           0.017453292519943295f

// 電源を入れてからジャイロのゼロ点が落ち着くまでの待ち [ms]
#define WARMUP_MS         5000

// 基板では SA0 が Low なので 0x6B。
static const uint8_t s_addr = 0x6B;

// チップ座標 -> 胴体座標の回転 (v_body = R * v_chip)。胴体前面の 45° の座面に、
// 4 ピン I2C コネクタを下に向けて取り付けたときのもの
// (arduino_os_quad_robot/scripts/normalize_urdf.py の SENSOR_BOARD_RPY と IMU_RPY)。
//   チップ x = 胴体の右 (-y)
//   チップ y = 前上 45°
//   チップ z = 後ろ上 45°
static const float R_BODY_CHIP[3][3] = {
    { 0.0f,  0.70710678f, -0.70710678f},
    {-1.0f,  0.0f,         0.0f       },
    { 0.0f,  0.70710678f,  0.70710678f},
};

// 重力の向きを出す相補フィルタの時定数 [s]。ジャイロで回した向きを、
// 加速度の向きへこの時間でゆっくり寄せる。短いと歩行中の加速度に引きずられ、
// 長いとジャイロのずれが残る。
#define GRAVITY_TAU_S     0.5f
// 加速度の大きさが 1 g からこれ以上離れているとき (着地の衝撃など) は寄せない。
#define ACC_TRUST_BAND_G  0.15f

static float s_gyro_bias[3];   // チップ座標 [rad/s]
static float s_grav[3];        // 胴体座標、単位ベクトル
static float s_acc_chip[3], s_gyr_chip[3];

static bool write_reg(uint8_t reg, uint8_t val)
{
    IMU_WIRE.beginTransmission(s_addr);
    IMU_WIRE.write(reg);
    IMU_WIRE.write(val);
    return IMU_WIRE.endTransmission() == 0;
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t n)
{
    IMU_WIRE.beginTransmission(s_addr);
    IMU_WIRE.write(reg);
    if (IMU_WIRE.endTransmission(false) != 0) return false;
    if (IMU_WIRE.requestFrom(s_addr, (uint8_t)n) != n) return false;
    for (size_t i = 0; i < n; ++i) buf[i] = (uint8_t)IMU_WIRE.read();
    return true;
}

static void rotate(const float m[3][3], const float v[3], float out[3])
{
    for (int i = 0; i < 3; ++i)
        out[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
}

static void normalize(float v[3])
{
    float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-6f) { v[0] /= n; v[1] /= n; v[2] /= n; }
}

// 加速度 [g] と角速度 [rad/s] をチップ座標で読む (ゼロ点は引かない)。
static bool read_raw(float acc[3], float gyr[3])
{
    uint8_t b[12];
    if (!read_regs(REG_AX_L, b, sizeof(b))) return false;
    for (int i = 0; i < 3; ++i) {
        int16_t a = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        int16_t g = (int16_t)(b[6 + 2 * i] | (b[6 + 2 * i + 1] << 8));
        acc[i] = a / ACC_LSB_PER_G;
        gyr[i] = g / GYR_LSB_PER_DPS * DEG2RAD;
    }
    return true;
}

// 加速度計は重力と逆向き (静止で上向き +1 g) を返すので、反転したものが重力の向き。
static void gravity_from_acc(const float acc_chip[3], float g_body[3])
{
    float down_chip[3] = {-acc_chip[0], -acc_chip[1], -acc_chip[2]};
    rotate(R_BODY_CHIP, down_chip, g_body);
    normalize(g_body);
}

bool quad_imu_begin(void)
{
    IMU_WIRE.begin();
    IMU_WIRE.setClock(400000);      // 3.3 V なら 400 kHz で読める
    uint8_t id = 0;
    if (!read_regs(REG_WHO_AM_I, &id, 1) || id != WHO_AM_I_VAL) return false;

    (void)write_reg(REG_RESET, 0xB0);
    delay(15);
    // センサを止めてから設定し、最後に有効にする。
    if (!(write_reg(REG_CTRL7, 0x00))) return false;
    delay(5);
    if (!(write_reg(REG_CTRL1, CTRL1_VAL) && write_reg(REG_CTRL2, CTRL2_VAL) &&
          write_reg(REG_CTRL3, CTRL3_VAL) && write_reg(REG_CTRL5, CTRL5_VAL) &&
          write_reg(REG_CTRL7, CTRL7_VAL)))
        return false;
    delay(WARMUP_MS);
    return true;
}

bool quad_imu_calibrate(int n)
{
    float sum_g[3] = {0}, sum_g2[3] = {0}, sum_a[3] = {0};
    int got = 0;
    for (int k = 0; k < n; ++k) {
        float a[3], g[3];
        delayMicroseconds(4000);
        if (!read_raw(a, g)) continue;
        for (int i = 0; i < 3; ++i) {
            sum_g[i] += g[i];
            sum_g2[i] += g[i] * g[i];
            sum_a[i] += a[i];
        }
        got++;
    }
    if (got < n / 2) return false;
    // 角速度の標準偏差が 0.05 rad/s (約 3°/s) を超えたら、動いていたとみなす。
    for (int i = 0; i < 3; ++i) {
        float mean = sum_g[i] / got;
        float var = sum_g2[i] / got - mean * mean;
        if (var > 0.05f * 0.05f) return false;
        s_gyro_bias[i] = mean;
        sum_a[i] /= got;
    }
    gravity_from_acc(sum_a, s_grav);
    return true;
}

bool quad_imu_update(float dt, quad_imu_t *out)
{
    float acc[3], gyr[3];
    if (!read_raw(acc, gyr)) return false;
    for (int i = 0; i < 3; ++i) {
        s_acc_chip[i] = acc[i];
        s_gyr_chip[i] = gyr[i];
        gyr[i] -= s_gyro_bias[i];
    }

    float w[3];
    rotate(R_BODY_CHIP, gyr, w);

    // 胴体座標で見た重力 (世界に固定したベクトル) は、胴体が w で回ると
    // dg/dt = -w x g で動く。まずジャイロでこれを進める。
    float *g = s_grav;
    float wxg[3] = {w[1] * g[2] - w[2] * g[1],
                    w[2] * g[0] - w[0] * g[2],
                    w[0] * g[1] - w[1] * g[0]};
    for (int i = 0; i < 3; ++i) g[i] -= wxg[i] * dt;

    // 加速度の大きさが 1 g に近いときだけ、加速度の向きへ寄せる。
    float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (fabsf(an - 1.0f) < ACC_TRUST_BAND_G) {
        float ga[3];
        gravity_from_acc(acc, ga);
        float k = dt / (GRAVITY_TAU_S + dt);
        for (int i = 0; i < 3; ++i) g[i] += k * (ga[i] - g[i]);
    }
    normalize(g);

    for (int i = 0; i < 3; ++i) {
        out->ang_vel[i] = w[i];
        out->gravity[i] = g[i];
    }
    return true;
}

void quad_imu_level(quad_imu_t *out)
{
    for (int i = 0; i < 3; ++i) {
        out->ang_vel[i] = 0.0f;
        out->gravity[i] = 0.0f;
    }
    out->gravity[2] = -1.0f;
}

void quad_imu_print(const quad_imu_t *body)
{
    Serial.print(F("chip acc[g]"));
    for (int i = 0; i < 3; ++i) { Serial.print(' '); Serial.print(s_acc_chip[i], 3); }
    Serial.print(F("  body w[rad/s]"));
    for (int i = 0; i < 3; ++i) { Serial.print(' '); Serial.print(body->ang_vel[i], 3); }
    Serial.print(F("  body g"));
    for (int i = 0; i < 3; ++i) { Serial.print(' '); Serial.print(body->gravity[i], 3); }
    Serial.println();
}

#endif  // QUAD_USES_IMU
