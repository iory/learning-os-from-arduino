// raw_adc.h
// レジスタ操作によるA/D変換

#ifndef RAW_ADC_H
#define RAW_ADC_H

#include <Arduino.h>
#include <Arduino.h>   // R_GPT0 / R_ADC0 などのレジスタ定義はここから来る

/**
 * ADCの初期化
 * AVRでの ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
 * に相当する処理
 */
void raw_adc_init(void) {
    // ---- ADCモジュールの電源ON ----
    R_MSTP->MSTPCRD_b.MSTPD16 = 0;  // ADC140 モジュールストップ解除

    // ---- ADC設定 ----
    // Step 1: ADCを停止（設定変更前に必要）
    R_ADC0->ADCSR = 0;

    // Step 2: 変換精度の設定
    // ADPRC (ADCER のビット2:1) = 11 → 14ビット分解能
    // (RA4M1 で有効なのは 00=12bit と 11=14bit のみ)
    // AVRでは固定10ビットだったが、RA4M1では選択可能
    R_ADC0->ADCER = 0x0006;  // ADPRC=11: 14bit, 右詰め

    // Step 3: サンプリング時間の設定
    // AVRのプリスケーラ（ADPS）に相当する概念
    // サンプリングステート数を設定
    for (int i = 0; i < 16; i++) {
        R_ADC0->ADSSTR[i] = 0x0D;  // 13ステート（デフォルト）
    }
}

/**
 * 指定チャネルのA/D変換を実行（ポーリング方式）
 *
 * AVRでの以下のコードに相当:
 *   ADMUX = (ADMUX & 0xF0) | channel;
 *   ADCSRA |= _BV(ADSC);
 *   loop_until_bit_is_clear(ADCSRA, ADSC);
 *   return ADC;
 */
uint16_t raw_analog_read(uint8_t channel) {
    // Step 1: チャネル選択
    // AVR: ADMUX = (ADMUX & 0xF0) | channel;
    R_ADC0->ADANSA[0] = (1 << channel);  // 指定チャネルのみ有効

    // Step 2: 変換開始
    // AVR: ADCSRA |= _BV(ADSC);
    R_ADC0->ADCSR |= R_ADC0_ADCSR_ADST_Msk;  // ADST = 1 で変換開始

    // Step 3: 変換完了を待機（ポーリング）
    // AVR: loop_until_bit_is_clear(ADCSRA, ADSC);
    while (R_ADC0->ADCSR & R_ADC0_ADCSR_ADST_Msk) {
        // ADSTが自動クリアされるまで待機
        // AVRの ADSC と同じ動作
    }

    // Step 4: 結果を読む
    // AVR: return ADC;  (ADCH:ADCLの16bit結合値)
    return R_ADC0->ADDR[channel];  // 14ビット値（0〜16383）
}

/**
 * 14ビット値を10ビット値（0〜1023）に変換
 * Arduino標準のanalogRead()と同じ範囲にする
 */
uint16_t raw_analog_read_10bit(uint8_t channel) {
    return raw_analog_read(channel) >> 4;  // 14bit → 10bit
}

#endif // RAW_ADC_H
