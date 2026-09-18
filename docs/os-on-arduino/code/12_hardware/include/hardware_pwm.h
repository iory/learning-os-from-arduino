// hardware_pwm.h
// GPTタイマによるハードウェアPWM（サーボ制御用）

#ifndef HARDWARE_PWM_H
#define HARDWARE_PWM_H

#include <Arduino.h>

// RA4M1 GPTレジスタ定義
// Renesas FSP ヘッダから利用可能
#include <Arduino.h>   // R_GPT0 / R_ADC0 などのレジスタ定義はここから来る

class HardwarePWM {
private:
    R_GPT0_Type *gpt;    // GPTチャネルのレジスタブロックへのポインタ
    int pin;
    int ch_num;          // GPTチャネル番号 (0〜7)
    int channel;         // 0 = GTIOCA 出力, 1 = GTIOCB 出力
    bool active;

public:
    HardwarePWM() : gpt(nullptr), pin(-1), ch_num(-1), channel(-1), active(false) {}

    /**
     * GPTチャネルを使ってPWM出力を開始する
     *
     * Arduino UNO R4 WiFi のGPTピン対応:
     *   D3  → GPT1 (GTIOC1A)
     *   D5  → GPT0 (GTIOC0A)
     *   D6  → GPT3 (GTIOC3A)
     *   D9  → GPT7 (GTIOC7B)
     *   D10 → GPT2 (GTIOC2A)
     *   D11 → GPT6 (GTIOC6A)
     */
    bool attach(int p) {
        pin = p;

        // ピン番号からGPTチャネルを決定
        switch (pin) {
            case 3:  gpt = R_GPT1; ch_num = 1; channel = 0; break;  // GTIOC1A
            case 5:  gpt = R_GPT0; ch_num = 0; channel = 0; break;  // GTIOC0A
            case 6:  gpt = R_GPT3; ch_num = 3; channel = 0; break;  // GTIOC3A
            case 9:  gpt = R_GPT7; ch_num = 7; channel = 1; break;  // GTIOC7B
            case 10: gpt = R_GPT2; ch_num = 2; channel = 0; break;  // GTIOC2A
            case 11: gpt = R_GPT6; ch_num = 6; channel = 0; break;  // GTIOC6A
            default:
                Serial.println("[PWM] Error: unsupported pin");
                return false;
        }

        // ---- GPT モジュールの電源ON ----
        // AVRと違いARMの周辺モジュールはデフォルトで電源OFFの場合がある
        // Module Stop Control Register でGPTモジュールを有効化
        // RA4M1 は ch0〜1 (GPT320/321) が MSTPD5、ch2〜7 (GPT162〜167) が MSTPD6
        if (ch_num <= 1) {
            R_MSTP->MSTPCRD_b.MSTPD5 = 0;
        } else {
            R_MSTP->MSTPCRD_b.MSTPD6 = 0;
        }

        // ---- ピンのマルチプレクサ設定 ----
        // AVRではピンの多機能割り当ては限られていたが、
        // ARMでは PFS (Pin Function Select) レジスタで明示的に設定する
        // ここではArduinoコアの pinPeripheral() でピンを GPT 出力へ切り替える
        // (実際のレジスタは R_PFS->PORT[port].PIN[pin].PmnPFS)
        pinPeripheral((uint32_t)pin,
                      (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));

        // ---- GPTタイマの設定 ----
        // Step 1: タイマ停止
        // GTSTP/GTSTR は全チャネル共有のビットマップ。自分のビットだけ立てる
        gpt->GTSTP = (1u << ch_num);

        // Step 2: 動作モード設定
        // Saw-wave (片斜面) PWMモード
        // プリスケーラ: PCLKD/64 → 48MHz / 64 = 750kHz
        gpt->GTCR = (0x0 << 16)    // MD[2:0] = 000 : ノコギリ波PWMモード
                   | (0x3 << 24);   // TPCS[2:0] = 011 : PCLKD/64

        // Step 3: 周期設定
        // 50Hz (20ms) のPWM周期
        // 750kHz × 20ms = 15000 カウント
        // → AVRで「プリスケーラ1024、OCR0A = 155」としていたのと同じ考え方
        gpt->GTPR = 15000 - 1;     // 周期 = (GTPR + 1) / 750kHz = 20ms

        // Step 4: 出力制御
        // 周期先頭でHigh、比較一致でLowに設定
        if (channel == 0) {
            // GTIOCA: 周期先頭でHigh、比較一致でLow (初期値はLow)
            gpt->GTIOR_b.GTIOA = 0x09;
            gpt->GTIOR_b.OAE = 1;       // GTIOCA出力有効
        } else {
            // GTIOCB: 同様
            gpt->GTIOR_b.GTIOB = 0x09;
            gpt->GTIOR_b.OBE = 1;       // GTIOCB出力有効
        }

        active = true;

        // Step 5: 初期デューティ比（90° = 中央）
        writeMicroseconds(1500);

        // Step 6: カウンタクリア & タイマ開始
        gpt->GTCNT = 0;
        gpt->GTSSR_b.CSTRT = 1;       // ソフトウェアスタートを起動要因として許可
        gpt->GTSTR = (1u << ch_num);  // カウント開始

        return true;
    }

    void detach() {
        if (gpt) {
            gpt->GTSTP = (1u << ch_num);  // タイマ停止
        }
        active = false;
    }

    /**
     * パルス幅をマイクロ秒で指定
     * 750kHz → 1カウント = 1.333μs
     * us マイクロ秒 → us × 750000 / 1000000 = us × 0.75 カウント
     */
    void writeMicroseconds(int us) {
        if (!active) return;
        uint32_t counts = (uint32_t)us * 3 / 4;  // μs → カウント変換

        if (channel == 0) {
            gpt->GTCCR[0] = counts;  // GTCCRA更新 → デューティ比変更
        } else {
            gpt->GTCCR[1] = counts;  // GTCCRB更新
        }
        // ★ ここがポイント: レジスタに値を書くだけ
        // CPUはすぐ次の処理に進める
        // 波形生成はハードウェアが自動で行う
    }

    /**
     * 角度（0〜180°）で指定
     */
    void write(int angle) {
        angle = constrain(angle, 0, 180);
        int us = map(angle, 0, 180, 500, 2500);
        writeMicroseconds(us);
    }

    int read() {
        if (!active) return 0;
        uint32_t counts;
        if (channel == 0) {
            counts = gpt->GTCCR[0];
        } else {
            counts = gpt->GTCCR[1];
        }
        int us = counts * 4 / 3;
        return map(us, 500, 2500, 0, 180);
    }
};

#endif // HARDWARE_PWM_H
