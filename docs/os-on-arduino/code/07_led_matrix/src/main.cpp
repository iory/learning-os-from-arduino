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

// タスク2: 軽い計算
void task_light(void)
{
    while (1) {
        volatile int sum = 0;
        for (int i = 0; i < 10000; i++) {
            sum += i;
        }
        os_sleep(50);
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

    Serial.println("Legend:");
    Serial.println("  Top half: CPU load graph (left=old, right=new)");
    Serial.println("  Bottom half: Task execution (1 row per task)");
    Serial.println("  Right indicators: RUNNING=11, READY=10, BLOCKED=01\n");

    Serial.println("Try 'kill 3' to stop Heavy task and watch CPU load drop!\n");

    os_start();
}

void loop() { }
