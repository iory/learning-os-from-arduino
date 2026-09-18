/**
 * Chapter 5: Interactive Shell Demo
 *
 * Features:
 * - ps: List all tasks with state and CPU usage
 * - kill N: Suspend task N
 * - exec N: Resume suspended task N
 * - info: Show system information
 * - reboot: Reboot the system
 */

#include <Arduino.h>
#include "os_kernel.h"
#include "shell.h"
#include "Arduino_LED_Matrix.h"

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED1_PIN LED_BUILTIN

// 2個目の LED も外付けせず、基板内蔵の 12×8 LED マトリクスで代用します。
// 左半分を LED2 として丸ごと点灯・消灯します。
// マトリクス制御の仕組みは第7章で学びます。ここではライブラリに任せます。
static ArduinoLEDMatrix matrix;
static uint8_t matrix_frame[8][12];

static void matrix_led(int led, bool on)
{
    int col_start = (led == 3) ? 6 : 0;
    for (int row = 0; row < 8; row++) {
        for (int col = col_start; col < col_start + 6; col++) {
            matrix_frame[row][col] = on ? 1 : 0;
        }
    }
    matrix.renderBitmap(matrix_frame, 8, 12);
}

void task_led1(void)
{
    pinMode(LED1_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED1_PIN, HIGH);
        os_sleep(200);
        digitalWrite(LED1_PIN, LOW);
        os_sleep(200);
    }
}

void task_led2(void)
{
    while (1) {
        matrix_led(2, true);
        os_sleep(500);
        matrix_led(2, false);
        os_sleep(500);
    }
}

void task_heartbeat(void)
{
    int count = 0;
    while (1) {
        // Silent heartbeat - just keeps running
        count++;
        os_sleep(1000);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== Interactive Shell Demo ===");
    Serial.println();

    // LEDマトリクスを初期化（LED2 の代用）
    matrix.begin();

    os_init();

    os_create_task(task_led1, "LED1", 512);
    os_create_task(task_led2, "LED2", 512);
    os_create_task(task_heartbeat, "Heartbeat", 256);
    os_create_task(shell_task, "Shell", 1024);

    Serial.println("Tasks created. Starting OS...");
    Serial.println();

    os_start();
}

void loop()
{
    // Never reached
}
