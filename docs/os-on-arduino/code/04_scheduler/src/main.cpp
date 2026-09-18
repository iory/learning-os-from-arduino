/**
 * Chapter 4: Preemptive Scheduler Demo
 *
 * Tasks are automatically switched by SysTick timer interrupt.
 * No need to call os_yield() - the scheduler runs every TIME_SLICE_MS.
 */

#include <Arduino.h>
#include "os_kernel.h"
#include "Arduino_LED_Matrix.h"

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED1_PIN LED_BUILTIN

// 2個目以降の LED は外付けせず、基板内蔵の 12×8 LED マトリクスで代用します。
// 左半分を LED2、右半分を LED3 として丸ごと点灯・消灯します。
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

// Task 1: Fast LED blink
void task_led1(void)
{
    pinMode(LED1_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED1_PIN, HIGH);
        os_sleep(100);  // Sleep 100ms
        digitalWrite(LED1_PIN, LOW);
        os_sleep(100);
    }
}

// Task 2: Slow blink on LED2 (matrix left half)
void task_led2(void)
{
    while (1) {
        matrix_led(2, true);
        os_sleep(500);  // Sleep 500ms
        matrix_led(2, false);
        os_sleep(500);
    }
}

// Task 3: Serial output with system info
void task_serial(void)
{
    int count = 0;
    while (1) {
        Serial.print("[");
        Serial.print(os_get_tick());
        Serial.print("ms] Count: ");
        Serial.print(count++);
        Serial.print(" | Current task: ");
        Serial.println(os_get_current_task());

        os_sleep(1000);  // Sleep 1 second
    }
}

// Task 4: Heavy computation (demonstrates preemption)
void task_heavy(void)
{
    volatile uint32_t sum = 0;
    while (1) {
        // Heavy computation - will be preempted
        for (volatile int i = 0; i < 1000000; i++) {
            sum += i;
        }
        Serial.print("Heavy task completed iteration, sum=");
        Serial.println(sum);
        sum = 0;

        // 計算完了をマトリクス右半分（LED3 相当）のトグルで示す
        static bool led3_on = false;
        led3_on = !led3_on;
        matrix_led(3, led3_on);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    // LEDマトリクスを初期化（LED2 / LED3 の代用）
    matrix.begin();

    Serial.println("=== Preemptive Scheduler Demo ===");
    Serial.println();
    Serial.println("Tasks are automatically switched every 10ms by SysTick.");
    Serial.println("The heavy_task demonstrates that even long-running");
    Serial.println("computations don't block other tasks.");
    Serial.println();

    // Initialize OS
    os_init();

    // Create tasks
    os_create_task(task_led1, "LED1", 512);
    os_create_task(task_led2, "LED2", 512);
    os_create_task(task_serial, "Serial", 1024);
    os_create_task(task_heavy, "Heavy", 512);

    Serial.println("Tasks created:");
    for (int i = 0; i < os_get_task_count(); i++) {
        Serial.print("  ID ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(os_get_task_name(i));
    }
    Serial.println();
    Serial.println("Starting preemptive scheduler...");
    Serial.flush();
    delay(100);

    // Start OS
    os_start();
}

void loop()
{
    // Never reached
}
