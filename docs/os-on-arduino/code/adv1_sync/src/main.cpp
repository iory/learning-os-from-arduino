/**
 * 応用編 第1章: ロックと同期
 *
 * 2 つのデモを続けて走らせる。
 *
 *  1) counter++ の race condition と、mutex で直る様子（1.1 / 1.3）
 *  2) 食事する哲学者 — 順序付けでデッドロックを避ける版（1.6.3）
 *
 * デッドロックする素朴版を体験したい場合は PHIL_NAIVE を 1 にする。
 * 数秒から数十秒で全員 hungry のまま止まる。
 */

#include <Arduino.h>
#include "os_kernel.h"
#include "os_sync.h"
#include "os_atomic.h"

#define PHIL_NAIVE 0     // 1 にするとデッドロックする素朴版になる
#define N_PHIL     5
#define N_INC      20000 // カウンタを増やす回数（1 タスクあたり）

// ---- デモ1: race condition ----
static volatile uint32_t g_counter_raw = 0;   // 保護なし
static volatile uint32_t g_counter_safe = 0;  // mutex で保護
static mutex_t g_counter_mtx;
static volatile int g_inc_done = 0;

static void task_inc(void)
{
    for (uint32_t i = 0; i < N_INC; i++) {
        g_counter_raw++;                 // 壊れる方
        mutex_lock(&g_counter_mtx);
        g_counter_safe++;                // 守られている方
        mutex_unlock(&g_counter_mtx);
    }
    __disable_irq();
    g_inc_done++;
    __enable_irq();
    while (1) os_sleep(1000);
}

// ---- デモ2: 食事する哲学者 ----
static mutex_t g_forks[N_PHIL];
static volatile uint32_t g_meals[N_PHIL];

static void philosopher(int id)
{
    int left  = id;
    int right = (id + 1) % N_PHIL;

#if !PHIL_NAIVE
    // 番号の小さいフォークから取る。これで循環待機が崩れる
    int lo = (left < right) ? left : right;
    int hi = (left < right) ? right : left;
    left = lo;
    right = hi;
#endif

    while (1) {
        os_sleep(50 + id * 7);        // 考える
        mutex_lock(&g_forks[left]);
        mutex_lock(&g_forks[right]);
        g_meals[id]++;                // 食べる
        os_sleep(30);
        mutex_unlock(&g_forks[right]);
        mutex_unlock(&g_forks[left]);
    }
}

static void phil0(void) { philosopher(0); }
static void phil1(void) { philosopher(1); }
static void phil2(void) { philosopher(2); }
static void phil3(void) { philosopher(3); }
static void phil4(void) { philosopher(4); }

// ---- 進捗の表示 ----
static void task_report(void)
{
    bool printed_counter = false;

    while (1) {
        if (!printed_counter && g_inc_done >= 2) {
            printed_counter = true;
            Serial.println();
            Serial.println(F("--- counter demo ---"));
            Serial.print(F("expected      : ")); Serial.println(N_INC * 2);
            Serial.print(F("without mutex : ")); Serial.println(g_counter_raw);
            Serial.print(F("with mutex    : ")); Serial.println(g_counter_safe);
            Serial.println(F("without mutex の方が小さければ race condition が起きている"));
            Serial.println();
        }

        Serial.print(F("meals:"));
        for (int i = 0; i < N_PHIL; i++) {
            Serial.print(F(" P")); Serial.print(i);
            Serial.print(F("=")); Serial.print(g_meals[i]);
        }
        Serial.println();
        os_sleep(2000);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println(F("=== Advanced 1: Locks and Synchronization ==="));
#if PHIL_NAIVE
    Serial.println(F("philosophers: NAIVE (deadlock expected)"));
#else
    Serial.println(F("philosophers: resource ordering (deadlock free)"));
#endif

    os_init();

    mutex_init(&g_counter_mtx);
    for (int i = 0; i < N_PHIL; i++) {
        mutex_init(&g_forks[i]);
        g_meals[i] = 0;
    }

    // os_create_task は MAX_TASKS を超えると -1 を返す。黙って落ちると
    // 「デモが動かない」原因が分かりにくいので、その場で知らせる。
    struct { void (*fn)(void); const char *name; } demo[] = {
        {task_inc, "Inc1"}, {task_inc, "Inc2"},
        {phil0, "P0"}, {phil1, "P1"}, {phil2, "P2"}, {phil3, "P3"}, {phil4, "P4"},
        {task_report, "Report"},
    };
    for (unsigned i = 0; i < sizeof(demo) / sizeof(demo[0]); i++) {
        if (os_create_task(demo[i].fn, demo[i].name, 512) < 0) {
            Serial.print(F("[Error] cannot create task: "));
            Serial.println(demo[i].name);
        }
    }

    os_start();
}

void loop() { }
