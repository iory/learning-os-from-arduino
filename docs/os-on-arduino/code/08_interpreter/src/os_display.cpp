// os_display.cpp

#include "os_display.h"
#include <Arduino.h>

// LEDマトリクスオブジェクト
static ArduinoLEDMatrix matrix;

// フレームバッファ（8行×12列）
static uint8_t frame[8][12];

// CPU負荷履歴（12サンプル）
static uint8_t cpu_history[12];
static int history_index = 0;

// タスク実行履歴（4タスク × 8サンプル）
#define TASK_HISTORY_LEN 8
static int task_history[MAX_TASKS][TASK_HISTORY_LEN];
static int task_history_index = 0;

void display_init(void)
{
    matrix.begin();

    // フレームバッファをクリア
    memset(frame, 0, sizeof(frame));
    memset(cpu_history, 0, sizeof(cpu_history));
    memset(task_history, -1, sizeof(task_history));
}

// CPU負荷をパーセントから高さ（0-4）に変換
static int cpu_to_height(int cpu_percent)
{
    if (cpu_percent >= 80) return 4;
    if (cpu_percent >= 60) return 3;
    if (cpu_percent >= 40) return 2;
    if (cpu_percent >= 20) return 1;
    return 0;
}

// フレームバッファにCPU負荷グラフを描画
static void draw_cpu_graph(void)
{
    // 上半分（行0-3）にCPU負荷を描画
    for (int col = 0; col < 12; col++) {
        int height = cpu_to_height(cpu_history[col]);

        for (int row = 0; row < 4; row++) {
            // 下から上に向かって描画（行3が底、行0が頂上）
            frame[3 - row][col] = (row < height) ? 1 : 0;
        }
    }
}

// フレームバッファにタスク実行状態を描画
static void draw_task_status(void)
{
    int task_count = os_get_task_count();

    // 最大4タスク表示（行4-7）
    int display_tasks = (task_count > 4) ? 4 : task_count;

    for (int t = 0; t < display_tasks; t++) {
        int row = 4 + t;

        // 履歴を描画
        for (int i = 0; i < TASK_HISTORY_LEN; i++) {
            int hist_idx = (task_history_index + i + 1) % TASK_HISTORY_LEN;
            int col = i;

            if (task_history[t][hist_idx] == t) {
                // このタスクが実行中だった
                frame[row][col] = 1;
            } else {
                frame[row][col] = 0;
            }
        }

        // 現在の状態を示すインジケータ（右端）
        TaskState state = os_get_task_state(t);
        switch (state) {
            case TASK_RUNNING:
                frame[row][10] = 1;
                frame[row][11] = 1;
                break;
            case TASK_READY:
                frame[row][10] = 1;
                frame[row][11] = 0;
                break;
            case TASK_BLOCKED:
                frame[row][10] = 0;
                frame[row][11] = 1;
                break;
            default:
                frame[row][10] = 0;
                frame[row][11] = 0;
                break;
        }
    }
}

// 区切り線を描画
static void draw_separator(void)
{
    // CPU負荷領域とタスク状態領域の間に区切り線
    // 列8, 9を区切りとして使用
    for (int row = 0; row < 8; row++) {
        frame[row][8] = 0;
        frame[row][9] = (row == 3) ? 1 : 0;  // 行3だけ点灯
    }
}

void display_update(void)
{
    // CPU 使用率を更新（アイドル以外の割合）。
    // os_get_cpu_usage() は呼ぶたびに基準時刻を更新するので、ここから呼ぶと
    // シェルの ps の計測窓を壊してしまう。生カウンタから自前で差分を取る。
    static uint32_t last_total = 0;
    static uint32_t last_idle  = 0;

    uint32_t total = os_get_tick();
    uint32_t idle  = os_get_task_cpu_ticks(0);
    uint32_t d_total = total - last_total;
    uint32_t d_idle  = idle  - last_idle;
    last_total = total;
    last_idle  = idle;

    int total_cpu = (d_total == 0) ? 0
                  : (int)(100u - (d_idle * 100u) / d_total);

    // 履歴に追加
    cpu_history[history_index] = total_cpu;
    history_index = (history_index + 1) % 12;

    // 現在実行中のタスクを記録
    int current = os_get_current_task();
    if (current >= 0 && current < MAX_TASKS) {
        task_history[current][task_history_index] = current;
    }
    task_history_index = (task_history_index + 1) % TASK_HISTORY_LEN;

    // フレームバッファをクリア
    memset(frame, 0, sizeof(frame));

    // 各要素を描画
    draw_cpu_graph();
    draw_task_status();

    // マトリクスに表示
    matrix.renderBitmap(frame, 8, 12);
}

// 表示更新タスク
void display_task(void)
{
    display_init();

    while (1) {
        display_update();
        os_sleep(100);  // 100msごとに更新
    }
}
