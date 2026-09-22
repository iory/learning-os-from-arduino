// 07_interpreter.ino
// 簡易インタプリタのデモ

#include "os_kernel.h"
#include "shell.h"
#include "interpreter.h"
#include "os_display.h"

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED_PIN LED_BUILTIN

void task_led(void)
{
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        os_sleep(500);
    }
}

// ---- v2 で足したタスク（書籍の第8章には無い）。マトリクスの棒を動かすための仕事で、
// 07_led_matrix_v2 の Light / Heavy と同じもの。

// 自分の CPU 時間を ms ぶん使い切るまで計算する。壁時計ではなく自分の CPU 時間で
// 数えるので、ほかのタスクと取り合っても仕事の量は変わらない。
static void burn_cpu_ms(uint32_t ms)
{
    int self = os_get_current_task();
    uint32_t start = os_get_task_cpu_ticks(self);
    volatile long sum = 0;
    while (os_get_task_cpu_ticks(self) - start < ms) {
        sum += 1;
    }
}

// 軽い計算（CPU を 20 ms 使っては 60 ms 眠る）
void task_light(void)
{
    while (1) {
        burn_cpu_ms(20);
        os_sleep(60);
    }
}

// 重い計算（眠らずに回り続ける）
void task_heavy(void)
{
    while (1) {
        volatile long sum = 0;
        for (long i = 0; i < 500000; i++) {
            sum += i;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== Motion Interpreter Demo ===");
    Serial.println("Type 'run' to enter script mode.");
    Serial.println("Type 'run <command>' for one-line execution.\n");

    Serial.println("Available commands:");
    Serial.println("  FORWARD <dist>      - Move forward");
    Serial.println("  BACKWARD <dist>      - Move backward");
    Serial.println("  TURN <angle>     - Turn");
    Serial.println("  SERVO <ch> <deg>  - Set servo angle");
    Serial.println("  LED <pin> <0/1> - Control LED");
    Serial.println("  DELAY <ms>        - Delay");
    Serial.println("  PRINT <text>      - Print message");
    Serial.println("  LOOP <n>        - Start loop");
    Serial.println("  END             - End loop");
    Serial.println();

    Serial.println("LED matrix (12 dots = 100%):");
    Serial.println("  Rows 0-1: total CPU load");
    Serial.println("  Rows 3-7: CPU% of LED, Shell, Display, Light, Heavy");
    Serial.println("  Type 'top' to see the same numbers here.\n");

    interp_init();
    os_init();

    os_create_task(task_led, "LED", 512);
    os_create_task(shell_task, "Shell", 1024);
    os_create_task(display_task, "Display", 512);
    // 書籍の 3 つの後ろに足すので、LED (1) / Shell (2) / Display (3) の ID は変わらない
    os_create_task(task_light, "Light", 512);
    os_create_task(task_heavy, "Heavy", 512);

    os_start();
}

void loop() { }
