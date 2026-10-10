// 第13章 四脚ロボット — WiFi 版 (env:uno_r4_wifi_udp / uno_r4_wifi_rcb4_udp)
//
// real-quad-rl/deploy/arduino/quad_wifi を移したもの。PC は速度だけを UDP で
// 送り、50 Hz のループとポリシーはマイコンで回す。違いは 2 つだけ:
//   - サーボとのやり取りを servo_bus.h に任せる。Feetech 版と RCB-4 版の
//     どちらでも同じこのファイルが走る
//   - Feetech の速度上限レジスタを起動時に書く (main.cpp と同じ。補遺 13.9.1)
//
// ループを PC 側ではなくマイコンに置くのは、制御周期ごとに WiFi の往復が
// 入ると 20 ms に収まらないから。ループがマイコンで閉じていれば、パケットが
// 落ちても指令が古くなるだけで、それは見張り (300 ms) が停止に変える。
//
// PC 側: host/quad_host.py wifi
// 接続先: include/arduino_secrets.h (arduino_secrets.example.h を写して埋める)
#include <Arduino.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <math.h>
#include <string.h>

#if __has_include("arduino_secrets.h")
#include "arduino_secrets.h"
#else
#error "include/arduino_secrets.example.h を include/arduino_secrets.h に写して、接続先を埋めること"
#endif

#include "servo_bus.h"
#include "quad_calib.h"
#include "quad_control.h"

#define NJ           QUAD_NJ

// 指令がこれより古ければ、通信が切れたとみなして home 保持に入る。
// 最後の速度で歩き続けると、ロボットはそのまま歩き去ってしまう。
#define COMMAND_TIMEOUT_MS 300
// 保持姿勢へ移るときの関節速度の上限 [rad/s]。
#define HOLD_MAX_RATE 1.0f
// 「home 姿勢を保持」を表すモードのバイト。host/quad_link.py と同じ。
#define MODE_HOLD_HOME_BYTE 1
// 1 回の制御周期で読み捨てるパケットの上限。この WiFi スタックは返信に数十 ms
// かかるので、届いたパケットに全部答えるとループが止まる。
#define MAX_PACKETS_PER_STEP 8
// 状態を PC へ返すか。RA4M1 と ESP32-S3 は 115200 baud 固定の UART で
// つながっていて、そこがこの基板の詰まりどころ。返信は数 ms で済むが、
// 受信側が 25〜130 ms かかる (実測)。PC はロボットを見つけるのにも返信を使う
// (発見はブロードキャストして返事を待つ) ので、間引いて残す。
//   https://forum.arduino.cc/t/max-transfer-rate-of-udp-data-with-uno-r4-wifi/1456024
//   https://github.com/arduino/uno-r4-wifi-usb-bridge/issues/55
#define REPLY_ENABLED 1
#define REPLY_INTERVAL_MS 500
// WiFi を見に行く間隔 [制御周期]。無線に触るたびに平均 18 ms、最悪 105 ms
// かかる (ESP32-S3 との往復)。速度指令はキーを押したときしか変わらないので
// 50 Hz で見る必要はない。5 周期 (100 ms) ごとにして、300 ms の見張りに
// 間に合わせる。送信レートと周期超過の関係 (実測):
//   20 Hz -> loop avg 18.4 ms, 19% の周期が 20 ms 超え
//    5 Hz -> loop avg  9.2 ms, 10%
//    1 Hz -> loop avg  4.8 ms,  2%
#define POLL_EVERY_N_STEPS 5

static quad_state_t policy;
static WiFiUDP udp;

static float q[NJ], qd[NJ], q_prev[NJ], target[NJ], last_target[NJ];
static float cmd[3] = {0.0f, 0.0f, 0.0f};
static unsigned long last_cmd_ms = 0;
static bool holding = true;
static float hold_from[NJ], hold_goal[NJ];
static int hold_k = 0, hold_steps = 1;
// 0 = 歩行、1 = home 保持、2 = 全関節 0 度で保持
static uint8_t hold_mode = 1;
static unsigned long step_i = 0;
static unsigned long loop_max_us = 0, loop_sum_us = 0, loop_n = 0, skipped = 0;
static unsigned long poll_max_us = 0, poll_rx_us = 0, poll_rx_max_us = 0;
// 制御周期を超えた回数。歩容のクロックがまだ意味を持つかはこれで決まる。
static unsigned long late_steps = 0, total_steps = 0;

// --- 保持 -------------------------------------------------------------------

// mode 1 は home、2 は全関節 0 度。0 度はサーボの校正の基準姿勢で、特異姿勢に
// 近い (膝 176 度) ので、確かめる用であって立つ用ではない。
static void begin_hold(const float *from, uint8_t mode)
{
    hold_mode = mode;
    for (int i = 0; i < NJ; ++i)
        hold_goal[i] = (mode == 2) ? 0.0f : QUAD_DEFAULT_JOINT_POS[i];
    float span = 0.0f;
    for (int i = 0; i < NJ; ++i) {
        float d = fabsf(hold_goal[i] - from[i]);
        if (d > span) span = d;
        hold_from[i] = from[i];
    }
    int need = (int)(span / (HOLD_MAX_RATE * QUAD_CONTROL_DT) + 0.5f);
    hold_steps = need < 20 ? 20 : need;
    hold_k = 0;
    holding = true;
}

static void hold_target(float *out)
{
    float a = hold_steps <= 0 ? 1.0f : (float)hold_k / (float)hold_steps;
    if (a > 1.0f) a = 1.0f;
    for (int i = 0; i < NJ; ++i)
        out[i] = hold_from[i] + a * (hold_goal[i] - hold_from[i]);
    if (hold_k <= hold_steps) hold_k++;
}

// --- WiFi -------------------------------------------------------------------

static void start_wifi(void)
{
    if (QUAD_WIFI_AP) {
        Serial.print(F("AP を開始: "));
        Serial.println(QUAD_WIFI_SSID);
        if (WiFi.beginAP(QUAD_WIFI_SSID, QUAD_WIFI_PASS) != WL_AP_LISTENING) {
            Serial.println(F("!! AP を開始できません"));
            return;
        }
        delay(2000);
    } else {
        Serial.print(F("接続中: "));
        Serial.println(QUAD_WIFI_SSID);
        WiFi.begin(QUAD_WIFI_SSID, QUAD_WIFI_PASS);
        for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; ++i) delay(500);
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println(F("!! WiFi に接続できません。指令なしで待機します"));
            return;
        }
    }
    IPAddress ip = WiFi.localIP();
    Serial.print(F("IP: ")); Serial.print(ip);
    Serial.print(F("   UDP ポート: ")); Serial.println(QUAD_UDP_PORT);
    udp.begin(QUAD_UDP_PORT);
}

// 指令パケット: 'Q','C', float vx, float wz, uint8 mode。リトルエンディアン
// (RA4M1 も PC もそう)。
static unsigned long last_reply_ms = 0;

static void send_state(IPAddress ip, uint16_t port)
{
    uint8_t out[64];
    int k = 0;
    out[k++] = 'Q'; out[k++] = 'S';
    for (int i = 0; i < NJ; ++i) {
        int16_t deg = (int16_t)(q[i] * 5729.58f / 10.0f);   // 0.1 度単位
        memcpy(out + k, &deg, 2); k += 2;
    }
    uint16_t loop_us = (uint16_t)(loop_n ? loop_sum_us / loop_n : 0);
    memcpy(out + k, &loop_us, 2); k += 2;
    uint16_t mx = (uint16_t)loop_max_us;
    memcpy(out + k, &mx, 2); k += 2;
    uint16_t sk = (uint16_t)skipped;
    memcpy(out + k, &sk, 2); k += 2;
    out[k++] = holding ? hold_mode : 0;
    udp.beginPacket(ip, port);
    udp.write(out, k);
    udp.endPacket();
}

static void poll_command(void)
{
    unsigned long rx0 = micros();
    bool got = false;
    float vx = 0.0f, wz = 0.0f;
    uint8_t mode = MODE_HOLD_HOME_BYTE;
    IPAddress from;
    uint16_t port = 0;

    // 数パケットだけ読み、いちばん新しいものだけを使う。古い速度に価値は無く、
    // 上限なしで読み捨てていたことがループを止めた原因だった。
    for (int i = 0; i < MAX_PACKETS_PER_STEP; ++i) {
        int n = udp.parsePacket();
        if (n <= 0) break;
        uint8_t buf[16];
        int nread = udp.read(buf, sizeof(buf));
        if (nread >= 11 && buf[0] == 'Q' && buf[1] == 'C') {
            memcpy(&vx, buf + 2, 4);
            memcpy(&wz, buf + 6, 4);
            mode = buf[10];
            from = udp.remoteIP();
            port = udp.remotePort();
            got = true;
        }
    }
    poll_rx_us = micros() - rx0;
    if (!got) return;

    last_cmd_ms = millis();
    if (mode) {
        if (!holding || hold_mode != mode) begin_hold(last_target, mode);
        cmd[0] = cmd[2] = 0.0f;
    } else {
        if (holding) {
            holding = false;
            quad_init(&policy);
        }
        cmd[0] = vx; cmd[2] = wz;
    }

    // 返信は高くつくので、指令ごとではなく自分のタイマーで送る。PC は表示に使うだけ。
#if REPLY_ENABLED
    if (millis() - last_reply_ms >= REPLY_INTERVAL_MS) {
        last_reply_ms = millis();
        send_state(from, port);
    }
#else
    (void)from; (void)port;
#endif
}

// --- setup / loop -----------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    servo_bus_init();

    int found = servo_bus_ping_all();
    Serial.print(F("サーボ応答: ")); Serial.print(found);
    Serial.print(F(" / ")); Serial.println(NJ);

#ifdef QUAD_SERVO_RCB4
    servo_bus_apply_limits();
    Serial.println(F("速度設定: RCB-4 には無い (補間 1 フレームで送る)"));
#else
    // 速度レジスタは SRAM にあるので電源のたびに出荷値へ戻る (補遺 13.9.1)
    int limits = servo_bus_apply_limits();
    Serial.print(F("速度設定: ")); Serial.print(limits);
    Serial.print(F(" / ")); Serial.print(NJ * 2);
    Serial.println(limits == NJ * 2 ? F(" 適用") : F(" !! 一部失敗 (遅く歩きます)"));
#endif

    if (!servo_bus_read(q)) Serial.println(F("!! 起動時の関節読み取りに失敗"));
    for (int i = 0; i < NJ; ++i) { q_prev[i] = q[i]; qd[i] = 0.0f; last_target[i] = q[i]; }

    quad_init(&policy);
    servo_bus_torque(true);
    begin_hold(q, 1);               // home へ立ち上がって待つ
    start_wifi();
    Serial.println(F("準備完了"));
}

void loop()
{
    static unsigned long next_us = 0;
    if (next_us == 0) next_us = micros();
    next_us += (unsigned long)(QUAD_CONTROL_DT * 1e6f);

    unsigned long t0 = micros();
    if (step_i % POLL_EVERY_N_STEPS == 0) {
        unsigned long p0 = micros();
        poll_command();
        unsigned long pd = micros() - p0;
        if (pd > poll_max_us) poll_max_us = pd;
        if (poll_rx_us > poll_rx_max_us) poll_rx_max_us = poll_rx_us;
    }

    if (!holding && millis() - last_cmd_ms > COMMAND_TIMEOUT_MS) {
        // 通信が途切れた。古い指令で歩き続けず、立って待つ。
        begin_hold(last_target, 1);
        cmd[0] = cmd[2] = 0.0f;
    }

    if (!servo_bus_read(q)) {
        skipped++;
    } else {
        for (int i = 0; i < NJ; ++i) {
            qd[i] = 0.6f * qd[i] + 0.4f * ((q[i] - q_prev[i]) / QUAD_CONTROL_DT);
            q_prev[i] = q[i];
        }
        if (holding) hold_target(target);
        else quad_step(&policy, q, qd, cmd, NULL, (long)step_i, target, NULL);
        for (int i = 0; i < NJ; ++i) last_target[i] = target[i];
        servo_bus_write(target);
    }
    step_i++;

    unsigned long dt = micros() - t0;
    total_steps++;
    if (dt > (unsigned long)(QUAD_CONTROL_DT * 1e6f)) late_steps++;
    loop_sum_us += dt; loop_n++;
    if (dt > loop_max_us) loop_max_us = dt;
    if (loop_n >= 50) {
        Serial.print(F("loop avg ")); Serial.print(loop_sum_us / loop_n);
        Serial.print(F(" us  max ")); Serial.print(loop_max_us);
        Serial.print(F(" us  skipped ")); Serial.print(skipped);
        Serial.print(F("  ")); Serial.print(holding ? F("hold") : F("walk"));
        // 途切れるならどこか: 受け取った指令、ポリシーの出した目標、関節の実際
        Serial.print(F("  late=")); Serial.print(late_steps);
        Serial.print('/'); Serial.print(total_steps);
        Serial.print(F("  poll_max=")); Serial.print(poll_max_us);
        Serial.print(F(" rx_max=")); Serial.print(poll_rx_max_us);
        Serial.print(F("  cmd_vx=")); Serial.print(cmd[0], 3);
        Serial.print(F(" tgt0=")); Serial.print(target[0] - QUAD_DEFAULT_JOINT_POS[0], 3);
        Serial.print(F(" q0=")); Serial.print(q[0] - QUAD_DEFAULT_JOINT_POS[0], 3);
        Serial.print(F(" age=")); Serial.print(millis() - last_cmd_ms);
#ifdef QUAD_SERVO_RCB4
        Serial.print(F("  write_fail=")); Serial.print(servo_bus_write_fail());
#endif
        Serial.println();
        loop_sum_us = 0; loop_n = 0; loop_max_us = 0; poll_max_us = 0; poll_rx_max_us = 0;
    }

    long sleep_us = (long)(next_us - micros());
    if (sleep_us > 0) delayMicroseconds(sleep_us);
    else next_us = micros();
}
