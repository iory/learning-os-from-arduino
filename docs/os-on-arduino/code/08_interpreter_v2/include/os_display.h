// os_display.h

#ifndef OS_DISPLAY_H
#define OS_DISPLAY_H

#include "os_kernel.h"
#include "Arduino_LED_Matrix.h"

// 表示を初期化
void display_init(void);

// 表示を更新（定期的に呼ぶ）
void display_update(void);

// 表示タスク
void display_task(void);

// 直近の計測窓（SAMPLE_MS）の値。シェルの top がマトリクスと同じ数字を出すのに使う
int display_get_load(void);              // 全体の CPU 負荷 [%]（idle 以外）
int display_get_task_percent(int id);    // タスク id の CPU 使用率 [%]
int display_get_bar_length(int percent); // 横棒の長さ（0〜12）。マトリクスと同じ換算
uint32_t display_get_sample_count(void); // 計測するたびに 1 増える

#endif // OS_DISPLAY_H
