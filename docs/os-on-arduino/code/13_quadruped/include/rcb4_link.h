// RCB-4 mini の COM ポートを UNO R4 の D8/D9 から叩く (RCB-4 版 servo_bus 用)
//
// COM ポートは 1.25 Mbps・8E1 の「反転」UART で、待機中が LOW になる。
// RA4M1 の SCI にはピンを反転する機能が無い (SPTR の TINV/RINV は書いても
// 立たない。実測)。標準の SoftwareSerial は反転とパリティを同時に使うと
// スタート/ストップ/パリティの各ビットが正しく出ない。そこで両方向とも
// GPT と DMA でピンを直接扱う。
//
//   送信 (D9): ビット周期の GPT が、1 ビットにつき 1 回 PCNTR3 へ DMA で書く
//   受信 (D8): 最初の立ち上がり (反転したスタートビット) が ELC 経由で GPT を
//              起動し、その GPT が PIDR を DMA で 1 ビットに約 2.7 回読む。
//              ためたサンプルは後からソフトで解読する
//
// 配線: RCB-4 の COM コネクタは GND 側から GND - Rx - Tx。
//       UNO D9 -> RCB-4 Rx、UNO D8 <- RCB-4 Tx、GND 共通。
//       D0/D1 (Serial1) はほかの UART 機器のために空けてある。
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// rcb4_transact の戻り値。正の値は返事の長さ。
#define RCB4_NO_REPLY       0
#define RCB4_BAD_REPLY     (-1)   // 長さかチェックサムが合わない
#define RCB4_TX_TOO_LONG   (-2)
#define RCB4_RX_OVERFLOW   (-3)

// 失敗したら false。理由は rcb4_link_error() が返す。
bool rcb4_link_begin(void);
const char *rcb4_link_error(void);

// cmd を送り、timeout_us まで返事を待つ。RCB-4 の返事は先頭が自分の長さ、
// 末尾がそれ以外のバイトの 8 bit 和になっている。
int rcb4_transact(const uint8_t *cmd, size_t len, uint8_t *reply,
                  size_t reply_max, uint32_t timeout_us);
