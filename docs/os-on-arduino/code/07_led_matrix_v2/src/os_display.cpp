// os_display.cpp — 第7章 LED マトリクス可視化 (v2)
//
// htop の CPU メーターと同じく「いまの値」を左から伸びる横棒で出す。12 個で 100%。
//   行 0-1 : 全体の CPU 負荷（idle 以外）。太い棒
//   行 2   : 空き（区切り）
//   行 3-7 : タスク 1〜5（LED, Light, Heavy, Shell, Display）の CPU 使用率
// どれも SAMPLE_MS ごとに測り直す。数え方は ps の CPU% と同じ。

#include "os_display.h"
#include <Arduino.h>
#include <string.h>

#define ROWS        8
#define COLS        12
#define SAMPLE_MS   500             // 0.5 秒ごとに測り直す
#define LOAD_ROWS   2               // 全体の負荷の棒の太さ（行 0-1）
#define TASK_ROW0   3               // タスクの棒の最初の行
#define FIRST_TASK  1               // 棒を出す最初のタスク ID (0 は idle)
#define BAR_TASKS   (ROWS - TASK_ROW0)

static ArduinoLEDMatrix matrix;
static uint8_t frame[ROWS][COLS];

// 直近の計測窓の全体の CPU 負荷 [%]
static uint8_t load_percent = 0;

// タスクごとの直近 SAMPLE_MS の CPU 使用率 [%]（全タスク分）
static uint8_t task_percent[MAX_TASKS];
static volatile uint32_t sample_count = 0;

void display_init(void)
{
    matrix.begin();
    memset(frame, 0, sizeof(frame));
    load_percent = 0;
    memset(task_percent, 0, sizeof(task_percent));
}

// 使用率 [%] を横棒の長さ (0〜12) に。少しでも動いていれば 1 個は点ける。
static int percent_to_length(int percent)
{
    int len = (percent * COLS + 99) / 100;
    return (len > COLS) ? COLS : len;
}

// 前回からの差分で、全体の負荷とタスクごとの使用率を測る。
// os_get_cpu_usage() は ps の計測窓を動かしてしまうので使わず、
// 生のカウンタ (ミリ秒) から自前で差分を取る。
static void sample(void)
{
    static uint32_t last_total = 0;
    static uint32_t last_ticks[MAX_TASKS];

    uint32_t total = os_get_tick();
    uint32_t d_total = total - last_total;
    last_total = total;

    uint32_t d_ticks[MAX_TASKS];
    int n = os_get_task_count();
    for (int i = 0; i < n; i++) {
        uint32_t t = os_get_task_cpu_ticks(i);
        d_ticks[i] = t - last_ticks[i];
        last_ticks[i] = t;
    }
    if (d_total == 0) return;

    // 全体 = idle 以外
    load_percent = (uint8_t)(100u - (d_ticks[0] * 100u) / d_total);

    for (int i = 0; i < n; i++) {
        task_percent[i] = (uint8_t)((d_ticks[i] * 100u) / d_total);
    }
    sample_count++;
}

static void draw_bar(int row, int percent)
{
    int len = percent_to_length(percent);
    for (int col = 0; col < len; col++) {
        frame[row][col] = 1;
    }
}

static void draw(void)
{
    memset(frame, 0, sizeof(frame));
    for (int r = 0; r < LOAD_ROWS; r++) {
        draw_bar(r, load_percent);
    }
    for (int b = 0; b < BAR_TASKS; b++) {
        draw_bar(TASK_ROW0 + b, task_percent[FIRST_TASK + b]);
    }
    matrix.renderBitmap(frame, ROWS, COLS);
}

int display_get_load(void) { return load_percent; }

int display_get_task_percent(int id)
{
    return (id >= 0 && id < MAX_TASKS) ? task_percent[id] : 0;
}

int display_get_bar_length(int percent) { return percent_to_length(percent); }

uint32_t display_get_sample_count(void) { return sample_count; }

void display_update(void)
{
    sample();
    draw();
}

void display_task(void)
{
    display_init();
    while (1) {
        display_update();
        os_sleep(SAMPLE_MS);
    }
}
