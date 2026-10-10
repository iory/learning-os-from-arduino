// RCB-4 mini の COM ポート (反転 UART 1.25 Mbps 8E1) を GPT + DMA で叩く。
// 仕組みは rcb4_link.h の冒頭。
#include <Arduino.h>
#include "FspTimer.h"
#include "r_dmac.h"
#include "r_elc.h"
#include "rcb4_link.h"

extern int attachIrq2Link(uint32_t pin, PinStatus mode);

// D0/D1 は Serial1 (SCI2) のピンなので空けておき、UART の機能を持たないピンを使う。
// 受信は ELC で GPT を起動するので、外部割り込み (IRQ) を持つピンに限られる。
#define RX_PIN          8           // D8 (P304, IRQ9) <- RCB-4 Tx
#define TX_PIN          9           // D9 (P303)       -> RCB-4 Rx
#define BAUD            1250000UL
#define BITS_PER_FRAME  11          // スタート + データ 8 + 偶数パリティ + ストップ
#define TX_MAX_BYTES    96
#define DMA_CH_RX       0           // 番号の小さいチャンネルほど優先度が高い
#define DMA_CH_TX       1

// GPT のコンペア A がこのカウントで一致するたびに DMA が 1 回動く。
#define TRIGGER_COUNT   1

// 受信のサンプル間隔 [GPT カウント]。PIDR から SRAM への DMA は 1 回あたり
// 約 12.2 カウント (PCLKD 48 MHz) が限界で、9〜12 を指定してもすべて 1 ビット
// (38.4 カウント) あたり 3.14 サンプルになった (実測)。14 にして余裕を取る。
// 1 ビットあたり約 2.7 サンプル。
#define RX_PERIOD_COUNTS  14
#define RX_SAMPLES        5000      // 1 バイト 11 ビットで約 180 バイトぶん

// スタートビットの立ち上がりから最初のサンプルまでの遅れ [GPT カウント]。
// ICU のエッジ同期 + ELC + GPT 起動 + コンペア + DMA。取り込んだ ACK の返事の
// 遷移位置から逆算すると 23.6〜27.6 に収まり、15〜30 ならどれでも解読できた。
#define RX_LATENCY_COUNTS 25

// 返事を待つあいだ DMA の残り数を読みに行く間隔。CPU が読みに行くたびに
// DMA とバスを取り合う。
#define RX_POLL_INTERVAL_US 20

typedef struct {
    transfer_info_t info;
    dmac_extended_cfg_t ext;
    transfer_cfg_t cfg;
    dmac_instance_ctrl_t ctrl;
} dma_chan_t;

static FspTimer   g_tx_tim;
static FspTimer   g_rx_tim;
static dma_chan_t g_tx_dma;
static dma_chan_t g_rx_dma;
static elc_instance_ctrl_t g_rcb4_elc_ctrl;
static const elc_cfg_t     g_rcb4_elc_cfg = {{ELC_EVENT_NONE}};

static uint32_t g_tx_buf[TX_MAX_BYTES * BITS_PER_FRAME];
static uint16_t g_rx_buf[RX_SAMPLES];
static volatile uint32_t       *g_tx_port;   // 送信ピンのポートの PCNTR3
static const volatile uint16_t *g_rx_port;   // 受信ピンのポートの PIDR
static uint32_t g_tx_mask;
static uint16_t g_rx_mask;
static uint32_t g_bit_q8;                    // 1 ビットの長さ [GPT カウント] の 256 倍
static const char *g_error = "not started";

static R_PORT0_Type *port_of(bsp_io_port_pin_t pin)
{
    return (R_PORT0_Type *)((uint32_t)(R_PORT1 - R_PORT0) * ((pin >> 8) & 0xFF)
                            + R_PORT0);
}

static elc_event_t gpt_compare_a_event(int ch)
{
    static const elc_event_t ev[] = {
        ELC_EVENT_GPT0_CAPTURE_COMPARE_A, ELC_EVENT_GPT1_CAPTURE_COMPARE_A,
        ELC_EVENT_GPT2_CAPTURE_COMPARE_A, ELC_EVENT_GPT3_CAPTURE_COMPARE_A,
        ELC_EVENT_GPT4_CAPTURE_COMPARE_A, ELC_EVENT_GPT5_CAPTURE_COMPARE_A,
        ELC_EVENT_GPT6_CAPTURE_COMPARE_A, ELC_EVENT_GPT7_CAPTURE_COMPARE_A,
    };
    return (ch >= 0 && ch < 8) ? ev[ch] : ELC_EVENT_NONE;
}

static elc_event_t icu_irq_event(int ch)
{
    static const elc_event_t ev[] = {
        ELC_EVENT_ICU_IRQ0,  ELC_EVENT_ICU_IRQ1,  ELC_EVENT_ICU_IRQ2,
        ELC_EVENT_ICU_IRQ3,  ELC_EVENT_ICU_IRQ4,  ELC_EVENT_ICU_IRQ5,
        ELC_EVENT_ICU_IRQ6,  ELC_EVENT_ICU_IRQ7,  ELC_EVENT_ICU_IRQ8,
        ELC_EVENT_ICU_IRQ9,  ELC_EVENT_ICU_IRQ10, ELC_EVENT_ICU_IRQ11,
        ELC_EVENT_ICU_IRQ12, ELC_EVENT_ICU_IRQ13, ELC_EVENT_ICU_IRQ14,
        ELC_EVENT_ICU_IRQ15,
    };
    return (ch >= 0 && ch < 16) ? ev[ch] : ELC_EVENT_NONE;
}

// 空いている GPT を 1 つ取る。start_on_elc なら ELC の GPT_A イベントで起動する。
static int open_gpt(FspTimer &tim, uint32_t period, bool start_on_elc)
{
    uint8_t type = 0;
    int8_t ch = FspTimer::get_available_timer(type);
    if (ch < 0 || type != GPT_TIMER) return -1;
    if (!tim.begin(TIMER_MODE_PERIODIC, type, ch, period, TRIGGER_COUNT,
                   TIMER_SOURCE_DIV_1)) {
        return -1;
    }
    if (start_on_elc && !tim.set_source_start(GPT_SOURCE_GPT_A)) return -1;
    if (!tim.open()) return -1;
    tim.stop();
    return ch;
}

static bool open_dma(dma_chan_t *d, uint8_t ch, elc_event_t trigger, bool to_port,
                     transfer_size_t size)
{
    memset(d, 0, sizeof(*d));
    transfer_info_t *i = &d->info;
    i->transfer_settings_word_b.dest_addr_mode =
        to_port ? TRANSFER_ADDR_MODE_FIXED : TRANSFER_ADDR_MODE_INCREMENTED;
    i->transfer_settings_word_b.repeat_area =
        to_port ? TRANSFER_REPEAT_AREA_SOURCE : TRANSFER_REPEAT_AREA_DESTINATION;
    i->transfer_settings_word_b.irq = TRANSFER_IRQ_END;
    i->transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED;
    i->transfer_settings_word_b.src_addr_mode =
        to_port ? TRANSFER_ADDR_MODE_INCREMENTED : TRANSFER_ADDR_MODE_FIXED;
    i->transfer_settings_word_b.size = size;
    i->transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL;
    i->p_src  = to_port ? (void const *)g_tx_buf : (void const *)g_rx_port;
    i->p_dest = to_port ? (void *)g_tx_port : (void *)g_rx_buf;
    i->length = 1;

    // 割り込みは使わない。終わったかどうかは DMCNT を見に行く。
    d->ext.offset = 1;
    d->ext.src_buffer_size = 1;
    d->ext.irq = FSP_INVALID_VECTOR;
    d->ext.ipl = 12;
    d->ext.channel = ch;
    d->ext.activation_source = trigger;

    d->cfg.p_info = &d->info;
    d->cfg.p_extend = &d->ext;
    return R_DMAC_Open(&d->ctrl, &d->cfg) == FSP_SUCCESS;
}

static uint32_t tx_sample(bool high)
{
    // PCNTR3 の下位 16 bit は「1 にする」、上位 16 bit は「0 にする」
    return high ? g_tx_mask : (g_tx_mask << 16);
}

static size_t build_tx(const uint8_t *cmd, size_t len)
{
    size_t n = 0;
    for (size_t i = 0; i < len; ++i) {
        uint8_t ones = 0;
        // 反転した線なので、論理 0 が HIGH、論理 1 が LOW
        g_tx_buf[n++] = tx_sample(true);                 // スタート (論理 0)
        for (int k = 0; k < 8; ++k) {
            bool bit = (cmd[i] >> k) & 1;
            ones += bit;
            g_tx_buf[n++] = tx_sample(!bit);
        }
        g_tx_buf[n++] = tx_sample(!(ones & 1));          // 偶数パリティ
        g_tx_buf[n++] = tx_sample(false);                // ストップ (論理 1)
    }
    return n;
}

// 時刻 t (GPT カウントの 256 倍、取り込みを始めたスタートビットが 0) に
// いちばん近いサンプルの番号。サンプル k は RX_LATENCY_COUNTS + k * 間隔。
static uint32_t sample_at(uint32_t t_q8)
{
    const uint32_t off = RX_LATENCY_COUNTS << 8;
    const uint32_t per = RX_PERIOD_COUNTS << 8;
    if (t_q8 < off) return 0;
    return (t_q8 - off + per / 2) / per;
}

static bool rx_high(size_t i) { return (g_rx_buf[i] & g_rx_mask) != 0; }

// 先頭 got 個のサンプルから、最後まで届いているフレームを解読する。
// パリティかストップビットが合わなかったフレームの数を *errors に返す。
static size_t decode(size_t got, uint8_t *out, size_t out_max, int *errors)
{
    size_t n = 0;
    uint32_t start = 0;            // 最初のスタートビットが取り込みを始めた
    *errors = 0;
    while (n < out_max) {
        uint32_t stop_i = sample_at(start + ((21 * g_bit_q8) >> 1));
        if (stop_i >= got) break;
        uint8_t v = 0, ones = 0;
        for (int k = 0; k < 8; ++k) {
            bool bit = !rx_high(sample_at(start + (((2 * k + 3) * g_bit_q8) >> 1)));
            v |= (uint8_t)(bit << k);
            ones += bit;
        }
        bool parity = !rx_high(sample_at(start + ((19 * g_bit_q8) >> 1)));
        bool stop = !rx_high(stop_i);
        if (((ones + parity) & 1) != 0 || !stop) (*errors)++;
        out[n++] = v;

        // 次のスタートビットの立ち上がりを探す。返事のバイトは続けて来るとは
        // 限らないので、1 バイトごとに位置を合わせ直す。
        size_t i = stop_i + 1;
        while (i < got && !rx_high(i)) ++i;
        if (i >= got) break;
        // 立ち上がりはサンプル i-1 と i の間にある
        start = (RX_LATENCY_COUNTS << 8) + i * (RX_PERIOD_COUNTS << 8)
              - ((RX_PERIOD_COUNTS << 8) >> 1);
    }
    return n;
}

bool rcb4_link_begin(void)
{
    bsp_io_port_pin_t tx_pin = g_pin_cfg[TX_PIN].pin;
    bsp_io_port_pin_t rx_pin = g_pin_cfg[RX_PIN].pin;
    g_tx_mask = 1u << (tx_pin & 0xFF);
    g_rx_mask = (uint16_t)(1u << (rx_pin & 0xFF));
    g_tx_port = &port_of(tx_pin)->PCNTR3;
    // 16 bit で読む PIDR は +0x06 にある。+0x04 は EIDR で、読むと 0 が続く。
    g_rx_port = &port_of(rx_pin)->PIDR;

    uint32_t pclkd = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKD);
    g_bit_q8 = (uint32_t)(((uint64_t)pclkd << 8) / BAUD);
    uint32_t tx_period = (g_bit_q8 + 128) >> 8;

    // ほかの何よりも先に、送信ピンを待機レベル (LOW) にしておく
    R_IOPORT_PinCfg(&g_ioport_ctrl, tx_pin,
                    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW);

    int tx_ch = open_gpt(g_tx_tim, tx_period, false);
    int rx_ch = open_gpt(g_rx_tim, RX_PERIOD_COUNTS, true);
    if (tx_ch < 0 || rx_ch < 0) {
        g_error = "no free GPT channel";
        return false;
    }
    if (!open_dma(&g_tx_dma, DMA_CH_TX, gpt_compare_a_event(tx_ch), true,
                  TRANSFER_SIZE_4_BYTE) ||
        !open_dma(&g_rx_dma, DMA_CH_RX, gpt_compare_a_event(rx_ch), false,
                  TRANSFER_SIZE_2_BYTE)) {
        g_error = "DMA open failed";
        return false;
    }

    int irq_ch = attachIrq2Link(RX_PIN, RISING);
    elc_event_t irq_ev = icu_irq_event(irq_ch);
    R_IOPORT_PinCfg(&g_ioport_ctrl, rx_pin,
                    IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_IRQ_ENABLE);
    if (irq_ev == ELC_EVENT_NONE
        || R_ELC_Open(&g_rcb4_elc_ctrl, &g_rcb4_elc_cfg) != FSP_SUCCESS
        || R_ELC_LinkSet(&g_rcb4_elc_ctrl, ELC_PERIPHERAL_GPT_A, irq_ev) != FSP_SUCCESS
        || R_ELC_Enable(&g_rcb4_elc_ctrl) != FSP_SUCCESS) {
        g_error = "ELC link from the RX pin failed";
        return false;
    }
    g_error = NULL;
    return true;
}

const char *rcb4_link_error(void) { return g_error; }

int rcb4_transact(const uint8_t *cmd, size_t len, uint8_t *reply,
                  size_t reply_max, uint32_t timeout_us)
{
    if (len > TX_MAX_BYTES) return RCB4_TX_TOO_LONG;

    // 先に受信を構える。RCB-4 のスタートビットが ELC 経由で受信の GPT を起動する。
    g_rx_tim.stop();
    g_rx_tim.reset();
    R_DMAC_Reset(&g_rx_dma.ctrl, (void const *)g_rx_port, g_rx_buf, RX_SAMPLES);

    size_t n_tx = build_tx(cmd, len);
    R_DMAC_Reset(&g_tx_dma.ctrl, g_tx_buf, (void *)g_tx_port, n_tx);
    g_tx_tim.reset();
    g_tx_tim.start();
    uint32_t t0 = micros();
    while (g_tx_dma.ctrl.p_reg->DMCNT && micros() - t0 < timeout_us) {
    }
    g_tx_tim.stop();

    int result = RCB4_NO_REPLY;
    t0 = micros();
    for (;;) {
        bool full = g_rx_dma.ctrl.p_reg->DMCNT == 0;
        size_t got = full ? RX_SAMPLES
                          : RX_SAMPLES - g_rx_dma.ctrl.p_reg->DMCRA_b.DMCRAL;
        int errors = 0;
        size_t n = got ? decode(got, reply, reply_max, &errors) : 0;
        if (n >= 1 && n >= reply[0]) {
            uint8_t sum = 0;
            for (size_t i = 0; i + 1 < reply[0]; ++i) sum += reply[i];
            result = (reply[0] >= 2 && errors == 0 && sum == reply[reply[0] - 1])
                   ? reply[0] : RCB4_BAD_REPLY;
            break;
        }
        if (full) {
            result = RCB4_RX_OVERFLOW;
            break;
        }
        if (micros() - t0 > timeout_us) {
            result = n ? RCB4_BAD_REPLY : RCB4_NO_REPLY;
            break;
        }
        delayMicroseconds(RX_POLL_INTERVAL_US);
    }
    g_rx_tim.stop();
    R_DMAC_Disable(&g_rx_dma.ctrl);
    return result;
}
