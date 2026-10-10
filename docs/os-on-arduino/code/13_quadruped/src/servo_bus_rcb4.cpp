// servo_bus.h の RCB-4 mini 版 (env:uno_r4_wifi_rcb4)。
//
// Feetech 版 (servo_bus.cpp) はサーボと直接話すが、こちらは間に RCB-4 が入る。
// RCB-4 が自分の周期で ICS サーボを読み書きし、こちらはその RAM を読み、
// 目標角を渡す。関数の約束は servo_bus.h のとおり。
#include <Arduino.h>
#include <math.h>
#include "servo_bus.h"
#include "quad_calib.h"
#include "quad_calib_rcb4.h"
#include "rcb4_link.h"

#define NJ QUAD_NUM_JOINTS

// RCB-4 の RAM 上のサーボ情報。1 個あたり 20 バイトで、その +4 にサーボが
// 返した現在位置 (2 バイト) がある。Kondo の RCB-4 Library for Python の
// DeviceAddrOffset と同じ。
#define SERVO_RAM_ADDRESS   0x0090
#define SERVO_RECORD_SIZE   20
#define MOTOR_POS_OFFSET    0x04
#define REF_POS_OFFSET      0x06    // 目標位置 (2 バイト)
#define SERVO_SLOTS         35

// ICS の割り当て表。RAM の $0044 から ICS 番号ごとに 2 バイトで、$FFFF は
// 割り当てなし、それ以外はその番号のサーボ情報の RAM アドレス。RCB-4 は
// 割り当てのある番号のサーボとしか話さない (RCB-4 ファームウェア
// リファレンス、システムレジスタの表)。起動時に ROM のプロジェクトから
// 読み込まれるので、ここで書いても電源を切れば元に戻る。
#define ICS_ASSIGN_ADDRESS  0x0044
#define ICS_UNASSIGNED      0xFFFF
// 割り当ててから、RCB-4 がサーボと 1 往復して現在位置を書くまで待つ時間。
// 1 フレームは約 16 ms。
#define ASSIGN_SETTLE_MS    100

#define CMD_MOVE            0x00
#define SUB_RAM_TO_COM      0x20
#define SUB_COM_TO_RAM      0x02
#define CMD_MULTI_SERVO     0x10    // 複数サーボ・速度共通
#define CMD_ACK             0xFE
#define ACK                 0x06

#define PULSE_HOLD          0x7FFF  // 今の位置で保持
#define PULSE_FREE          0x8000  // 脱力
#define PULSE_MIN           3500
#define PULSE_MAX           11500
#define PULSE_PER_RAD       ((4000.0f / 135.0f) * (180.0f / 3.14159265358979f))

// 1 往復の締切。RCB-4 の返事は 1 往復に約 0.6 ms かかる (実測)。
#define REPLY_TIMEOUT_US    3000
// RCB-4 は自分のサーボ周期の最中だと読み出しに答えないことがある。
#define READ_ATTEMPTS       4
// 目標角までの補間時間。単位は RCB-4 のフレーム (この基板で実測約 16 ms)。
// 1 が最短。50 Hz で目標を送り続けるので、補間は短いほどよい。
#define WRITE_FRAMES        1

#define NO_SLOT             0xFF

static const char *const JOINT_NAME[NJ] = {
    "FL_hip", "FL_knee", "RL_hip", "RL_knee",
    "RR_hip", "RR_knee", "FR_hip", "FR_knee"};

static bool          g_link = false;    // RCB-4 と話せる
static uint8_t       g_slot[NJ];        // 関節ごとの RCB-4 のサーボ番号。NO_SLOT は見つからなかった
static unsigned long g_write_fail = 0;

static uint8_t sum8(const uint8_t *b, int n)
{
    uint8_t s = 0;
    for (int i = 0; i < n; ++i) s += b[i];
    return s;
}

// RCB-4 の RAM に書く。返事は [4][0x00][ACK 0x06 か NAK 0x15][和]。
static bool write_ram(uint16_t addr, const uint8_t *data, int n)
{
    uint8_t cmd[16];
    cmd[0] = (uint8_t)(7 + n);
    cmd[1] = CMD_MOVE;
    cmd[2] = SUB_COM_TO_RAM;
    cmd[3] = (uint8_t)(addr & 0xFF);
    cmd[4] = (uint8_t)(addr >> 8);
    cmd[5] = 0x00;
    memcpy(cmd + 6, data, n);
    cmd[6 + n] = sum8(cmd, 6 + n);
    uint8_t r[8];
    return rcb4_transact(cmd, 7 + n, r, sizeof(r), REPLY_TIMEOUT_US) == 4
        && r[2] == ACK;
}

static bool write_word(uint16_t addr, uint16_t v)
{
    uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
    return write_ram(addr, d, 2);
}

static bool read_word(uint16_t addr, uint16_t *v)
{
    uint8_t cmd[10] = {0x0A, CMD_MOVE, SUB_RAM_TO_COM, 0x00, 0x00, 0x00,
                       (uint8_t)(addr & 0xFF), (uint8_t)(addr >> 8), 2, 0};
    cmd[9] = sum8(cmd, 9);
    for (int t = 0; t < READ_ATTEMPTS; ++t) {
        uint8_t r[8];
        if (rcb4_transact(cmd, sizeof(cmd), r, sizeof(r), REPLY_TIMEOUT_US) == 5) {
            *v = (uint16_t)(r[2] | (r[3] << 8));
            return true;
        }
    }
    return false;
}

static uint16_t record_of(uint8_t slot)
{
    return (uint16_t)(SERVO_RAM_ADDRESS + SERVO_RECORD_SIZE * slot);
}

// サーボ 1 個の現在位置 [パルス]。サーボがつながっていない番号は 0 が返る。
static bool read_pulse(uint8_t slot, uint16_t *pulse)
{
    return read_word(record_of(slot) + MOTOR_POS_OFFSET, pulse);
}

// 割り当てのある番号でサーボが現在位置を返しているか。
static bool slot_alive(uint8_t slot)
{
    uint16_t assigned, p;
    return read_word(ICS_ASSIGN_ADDRESS + 2 * slot, &assigned)
        && assigned == record_of(slot)
        && read_pulse(slot, &p) && p != 0;
}

// 割り当てのない番号を割り当てて、サーボが答えるか見る。答えなければ割り当てを
// 外して元に戻す。割り当てた瞬間に RCB-4 はサーボ情報の目標位置をサーボへ送る
// ので、先に目標位置を脱力 (0x8000) にしておく。前の値 (多くは 7500 = 中央) が
// 残ったまま割り当てると、脚がそこへ跳ぶ。
static bool try_assign(uint8_t slot)
{
    uint16_t assigned;
    if (!read_word(ICS_ASSIGN_ADDRESS + 2 * slot, &assigned)) return false;
    if (assigned != ICS_UNASSIGNED) return false;      // プロジェクトのものには触らない
    if (!write_word(record_of(slot) + REF_POS_OFFSET, PULSE_FREE)) return false;
    if (!write_word(ICS_ASSIGN_ADDRESS + 2 * slot, record_of(slot))) return false;
    delay(ASSIGN_SETTLE_MS);
    uint16_t p;
    if (read_pulse(slot, &p) && p != 0) return true;
    write_word(ICS_ASSIGN_ADDRESS + 2 * slot, ICS_UNASSIGNED);
    return false;
}

// 見つかった関節ぶんのパルスを 1 コマンドで送る。pulse は関節の順。
// RCB-4 はサーボ番号のビットマップの昇順に位置を読むので、番号の昇順で
// 並べ直して詰める。並べ直さずに送ると、どれも黙って別のサーボへ行く。
static bool send_pulses(const uint16_t *pulse, uint8_t frames)
{
    uint8_t cmd[9 + 2 * NJ];
    int n = 0;
    memset(cmd + 2, 0, 5);
    for (int slot = 0; slot < SERVO_SLOTS; ++slot) {
        for (int i = 0; i < NJ; ++i) {
            if (g_slot[i] != slot) continue;
            cmd[2 + (slot >> 3)] |= (uint8_t)(1u << (slot & 7));
            cmd[8 + 2 * n] = (uint8_t)(pulse[i] & 0xFF);
            cmd[9 + 2 * n] = (uint8_t)(pulse[i] >> 8);
            n++;
        }
    }
    if (n == 0) return false;
    int len = 9 + 2 * n;
    cmd[0] = (uint8_t)len;
    cmd[1] = CMD_MULTI_SERVO;
    cmd[7] = frames;
    cmd[len - 1] = sum8(cmd, len - 1);

    // 返事は [4][0x10][ACK 0x06 か NAK 0x15][和]。返事は必ず待つ。待たずに次の
    // 読み出しを始めると、遅れて来た ACK を読み出しの返事として受け取ってしまう。
    uint8_t r[8];
    return rcb4_transact(cmd, len, r, sizeof(r), REPLY_TIMEOUT_US) == 4
        && r[2] == ACK;
}

static bool may_drive(void)
{
    // 未校正のうちは読むだけ。向きもゼロ点も分からないまま目標角を送らない。
    return g_link && QUAD_RCB4_CALIBRATED;
}

static void send_all(uint16_t value)
{
    uint16_t p[NJ];
    for (int i = 0; i < NJ; ++i) p[i] = value;
    if (!send_pulses(p, WRITE_FRAMES)) g_write_fail++;
}

void servo_bus_init(void)
{
    if (!rcb4_link_begin()) {
        Serial.print(F("!! RCB-4 との通信を準備できない: "));
        Serial.println(rcb4_link_error());
        return;
    }
    g_link = true;

    // 右の SIO ポートなら ID * 2、左なら ID * 2 + 1。両方を調べ、サーボが
    // 答えるほうを採る。RCB-4 に書き込まれたプロジェクトがこの機体のサーボを
    // 登録しているとは限らないので (実機では 8 個中 2 個だった)、どちらにも
    // 答えがなければ、割り当てのない番号を割り当てて試す。両方にいたら、
    // どちらを動かすべきか決められないので使わない。
    for (int i = 0; i < NJ; ++i) {
        uint8_t right = (uint8_t)(QUAD_SERVO_ID[i] * 2);
        bool r = slot_alive(right);
        bool l = slot_alive(right + 1);
        if (!r && !l) {
            r = try_assign(right);
            if (!r) l = try_assign(right + 1);
        }
        g_slot[i] = (r && !l) ? right : (l && !r) ? (uint8_t)(right + 1) : NO_SLOT;
        Serial.print(JOINT_NAME[i]);
        Serial.print(F(" id ")); Serial.print(QUAD_SERVO_ID[i]);
        if (r && l)      Serial.println(F(": !! 右と左の両方にいる。使わない"));
        else if (r)      { Serial.print(F(" -> 右 #")); Serial.println(right); }
        else if (l)      { Serial.print(F(" -> 左 #")); Serial.println(right + 1); }
        else             Serial.println(F(": 見つからない"));
    }
    if (!QUAD_RCB4_CALIBRATED) {
        Serial.println(F("!! quad_calib_rcb4.h が未校正 (QUAD_RCB4_CALIBRATED 0)。"
                         "読むだけで、目標角は送らない。raw で測ること"));
    }
}

int servo_bus_ping_all(void)
{
    if (!g_link) return 0;
    static const uint8_t ack[4] = {0x04, CMD_ACK, ACK, 0x08};
    uint8_t r[8];
    if (rcb4_transact(ack, sizeof(ack), r, sizeof(r), REPLY_TIMEOUT_US) != 4) return 0;

    // RCB-4 はサーボがつながっていない番号も 0 で答えるので、0 は「いない」と数える
    int found = 0;
    for (int i = 0; i < NJ; ++i) {
        uint16_t p;
        if (g_slot[i] != NO_SLOT && read_pulse(g_slot[i], &p) && p != 0) found++;
    }
    return found;
}

// Feetech 版が書く速度上限のレジスタに当たるものは RCB-4 には無い。
// 速度は書き込みのたびに補間フレーム数 (WRITE_FRAMES) で決まる。
int servo_bus_apply_limits(void)
{
    return 0;
}

void servo_bus_torque(bool on)
{
    // 脱力は目標角を送らないので、未校正でも通す。プロジェクトや前の指令で
    // 力が入ったままのサーボを、校正の前に手で動かせるようにするため。
    if (on ? !may_drive() : !g_link) return;
    // 脱力 (0x8000) したサーボは、位置を送るだけでは力が入らない。先に保持
    // (0x7FFF) を送って入れ直す (kxreus の AtomS3 版で実測)。
    send_all(on ? PULSE_HOLD : PULSE_FREE);
}

bool servo_bus_read(float *q_rad)
{
    if (!g_link) return false;
    bool ok = true;
    for (int i = 0; i < NJ; ++i) {
        uint16_t p;
        if (g_slot[i] == NO_SLOT || !read_pulse(g_slot[i], &p) || p == 0) {
            ok = false;
            continue;
        }
        q_rad[i] = ((float)p - QUAD_RCB4_ZERO[i]) / PULSE_PER_RAD * QUAD_RCB4_SIGN[i];
    }
    return ok;
}

void servo_bus_write(const float *q_rad)
{
    if (!may_drive()) return;
    uint16_t p[NJ];
    for (int i = 0; i < NJ; ++i) {
        float pulse = q_rad[i] * PULSE_PER_RAD * QUAD_RCB4_SIGN[i] + QUAD_RCB4_ZERO[i];
        if (pulse < PULSE_MIN) pulse = PULSE_MIN;
        if (pulse > PULSE_MAX) pulse = PULSE_MAX;
        p[i] = (uint16_t)(pulse + 0.5f);
    }
    if (!send_pulses(p, WRITE_FRAMES)) g_write_fail++;
}

unsigned long servo_bus_write_fail(void)
{
    return g_write_fail;
}

void servo_bus_print_raw(void)
{
    for (int i = 0; i < NJ; ++i) {
        Serial.print(JOINT_NAME[i]);
        Serial.print(F("  id ")); Serial.print(QUAD_SERVO_ID[i]);
        if (g_slot[i] == NO_SLOT) {
            Serial.println(F("  見つからない"));
            continue;
        }
        Serial.print(g_slot[i] & 1 ? F("  左 #") : F("  右 #"));
        Serial.print(g_slot[i]);
        uint16_t p;
        if (!read_pulse(g_slot[i], &p)) {
            Serial.println(F("  読めない"));
            continue;
        }
        Serial.print(F("  pulse ")); Serial.print(p);
        Serial.print(F("  rad "));
        Serial.println(((float)p - QUAD_RCB4_ZERO[i]) / PULSE_PER_RAD * QUAD_RCB4_SIGN[i], 4);
    }
}
