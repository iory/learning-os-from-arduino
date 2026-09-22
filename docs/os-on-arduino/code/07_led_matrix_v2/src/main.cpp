// 06_led_matrix.ino
// LEDマトリクス可視化のデモ

#include "os_kernel.h"
#include "os_display.h"
#include "shell.h"

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED_PIN LED_BUILTIN

// タスク1: LED点滅（軽い）
void task_led(void)
{
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        os_sleep(200);
    }
}

// 自分の CPU 時間を ms ぶん使い切るまで計算する。
// 壁時計ではなく自分の CPU 時間で数えるので、ほかのタスクと取り合っても
// 仕事の量は変わらない。
static void burn_cpu_ms(uint32_t ms)
{
    int self = os_get_current_task();
    uint32_t start = os_get_task_cpu_ticks(self);
    volatile long sum = 0;
    while (os_get_task_cpu_ticks(self) - start < ms) {
        sum += 1;
    }
}

// タスク2: 軽い計算（CPU を 20 ms 使っては 60 ms 眠る）
void task_light(void)
{
    while (1) {
        burn_cpu_ms(20);
        os_sleep(60);
    }
}

// タスク3: 重い計算（CPU負荷高い）
void task_heavy(void)
{
    while (1) {
        volatile long sum = 0;
        for (long i = 0; i < 500000; i++) {
            sum += i;
        }
        // os_sleep なし：CPUをずっと使う
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== LED Matrix Visualization Demo ===");
    Serial.println("Watch the LED matrix to see OS activity!\n");

    os_init();

    os_create_task(task_led, "LED", 512);
    os_create_task(task_light, "Light", 512);
    os_create_task(task_heavy, "Heavy", 512);
    os_create_task(shell_task, "Shell", 1024);
    os_create_task(display_task, "Display", 512);

    Serial.println("Legend (12 dots = 100%):");
    Serial.println("  Rows 0-1: total CPU load");
    Serial.println("  Rows 3-7: CPU% of LED, Light, Heavy, Shell, Display");
    Serial.println("  Type 'top' to see the same numbers here.\n");

    Serial.println("Try 'kill 3' to stop Heavy task and watch CPU load drop!\n");

    os_start();
}

void loop() { }
