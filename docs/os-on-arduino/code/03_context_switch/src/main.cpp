/**
 * Chapter 3: Context Switch Demo
 *
 * Demonstrates manual context switching using os_yield().
 * Three tasks run concurrently, each yielding control manually.
 */

#include <Arduino.h>
#include <stddef.h>
#include "os_kernel.h"
#include "Arduino_LED_Matrix.h"

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED1_PIN LED_BUILTIN

// 2個目の LED も外付けせず、基板内蔵の 12×8 LED マトリクスで代用します。
// この章では左半分を LED2 として丸ごと点灯・消灯します。
// マトリクス制御の仕組みは第7章で学びます。ここではライブラリに任せます。
static ArduinoLEDMatrix matrix;
static uint8_t matrix_frame[8][12];

static void matrix_led(int led, bool on)
{
    int col_start = (led == 3) ? 6 : 0;   // 3 なら右半分、それ以外は左半分
    for (int row = 0; row < 8; row++) {
        for (int col = col_start; col < col_start + 6; col++) {
            matrix_frame[row][col] = on ? 1 : 0;
        }
    }
    matrix.renderBitmap(matrix_frame, 8, 12);
}

// Task 1: Blink LED1
void task1(void)
{
    Serial.println("Task1 started");
    Serial.flush();
    pinMode(LED1_PIN, OUTPUT);
    int count = 0;
    while (1) {
        digitalWrite(LED1_PIN, HIGH);
        for (volatile int i = 0; i < 100000; i++);  // Simple delay
        digitalWrite(LED1_PIN, LOW);
        for (volatile int i = 0; i < 100000; i++);

        Serial.print("Task1 yield #");
        Serial.println(count++);
        Serial.flush();
        os_yield();  // Explicitly yield CPU
    }
}

// Task 2: Blink LED2 (matrix left half) at different speed
void task2(void)
{
    Serial.println("Task2 started");
    Serial.flush();
    int count = 0;
    while (1) {
        matrix_led(2, true);
        for (volatile int i = 0; i < 50000; i++);
        matrix_led(2, false);
        for (volatile int i = 0; i < 50000; i++);

        Serial.print("Task2 yield #");
        Serial.println(count++);
        Serial.flush();
        os_yield();
    }
}

// Task 3: Serial output
void task3(void)
{
    int count = 0;
    while (1) {
        Serial.print("Task3 count: ");
        Serial.println(count++);
        Serial.flush();

        for (volatile int i = 0; i < 200000; i++);
        Serial.println("Task3 yielding...");
        Serial.flush();
        os_yield();
        Serial.println("Task3 resumed");
        Serial.flush();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection

    // Blink built-in LED to show we're alive
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }

    // LEDマトリクスを初期化（LED2 の代用）
    matrix.begin();

    Serial.println();
    Serial.println("=== Context Switch Demo ===");
    Serial.println();
    Serial.println("This demonstrates cooperative context switching.");
    Serial.println("Each task must call os_yield() to switch to another task.");
    Serial.println();

    // Initialize OS
    Serial.println("Initializing OS...");
    Serial.flush();
    os_init();
    Serial.println("OS initialized.");
    Serial.flush();

    // Debug: print TCB size and offsets
    Serial.print("TCB size: ");
    Serial.println(sizeof(TCB));
    Serial.print("  sp offset: ");
    Serial.println(offsetof(TCB, sp));
    Serial.print("  stack_base offset: ");
    Serial.println(offsetof(TCB, stack_base));
    Serial.print("  state offset: ");
    Serial.println(offsetof(TCB, state));
    Serial.print("  name offset: ");
    Serial.println(offsetof(TCB, name));
    Serial.print("  entry offset: ");
    Serial.println(offsetof(TCB, entry));
    Serial.flush();

    // Create tasks
    Serial.println("Creating tasks...");
    Serial.flush();
    os_create_task(task1, "LED1", 512);
    os_create_task(task2, "LED2", 512);
    os_create_task(task3, "Serial", 1024);

    Serial.println("Tasks created:");
    Serial.println("  - idle (ID 0)");
    Serial.println("  - LED1 (ID 1)");
    Serial.println("  - LED2 (ID 2)");
    Serial.println("  - Serial (ID 3)");
    Serial.println();
    Serial.print("Task count: ");
    Serial.println(g_task_count);
    Serial.println();
    // Debug: print task stack pointers
    Serial.println("Task stack pointers:");
    for (int i = 0; i < g_task_count; i++) {
        Serial.print("  Task ");
        Serial.print(i);
        Serial.print(" sp: 0x");
        Serial.println((uint32_t)g_tasks[i].sp, HEX);
    }
    Serial.flush();

    Serial.print("sizeof(TCB) = ");
    Serial.println(sizeof(TCB));
    Serial.println("Starting OS...");
    Serial.flush();
    delay(100);

    // Start OS (never returns)
    os_start();
}

void loop()
{
    // Never reached
}
