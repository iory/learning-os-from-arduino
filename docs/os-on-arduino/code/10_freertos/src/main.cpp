/**
 * Chapter 10: FreeRTOS Implementation
 *
 * Demonstrates how to implement the same functionality using FreeRTOS.
 * Shows the mapping between our custom OS and FreeRTOS APIs.
 *
 * Note: Arduino Uno R4 WiFi has FreeRTOS support through Arduino_FreeRTOS library.
 * This example shows the concepts - actual FreeRTOS requires the library.
 */

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"
#include "servo_pwm.h"

// This is a simplified demonstration showing FreeRTOS concepts
// For actual FreeRTOS, include <Arduino_FreeRTOS.h>

ArduinoLEDMatrix matrix;
ServoPWM servo;

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED1_PIN LED_BUILTIN
#define LED2_PIN 3
#define SERVO_PIN 9

// Simulated FreeRTOS-like API
// In real FreeRTOS, these would be the actual FreeRTOS functions

typedef void (*TaskFunction_t)(void*);

struct FakeTask {
    TaskFunction_t func;
    const char* name;
    void* param;
    unsigned long delay_until;
    bool suspended;
};

#define MAX_FAKE_TASKS 8
FakeTask fake_tasks[MAX_FAKE_TASKS];
int fake_task_count = 0;

// xTaskCreate equivalent
void xTaskCreate(TaskFunction_t func, const char* name, int stack, void* param, int priority, void* handle)
{
    if (fake_task_count < MAX_FAKE_TASKS) {
        fake_tasks[fake_task_count].func = func;
        fake_tasks[fake_task_count].name = name;
        fake_tasks[fake_task_count].param = param;
        fake_tasks[fake_task_count].delay_until = 0;
        fake_tasks[fake_task_count].suspended = false;
        fake_task_count++;
        Serial.print("[FreeRTOS] Task created: ");
        Serial.println(name);
    }
}

// vTaskDelay equivalent (simulated)
unsigned long vTaskDelay_ms = 0;
void vTaskDelay(unsigned long ticks)
{
    vTaskDelay_ms = millis() + ticks;
}

// vTaskDelayUntil equivalent
void vTaskDelayUntil(unsigned long* lastWake, unsigned long period)
{
    *lastWake += period;
    while (millis() < *lastWake) {
        // In real FreeRTOS, this would yield to scheduler
        delay(1);
    }
}

// ============================================
// FreeRTOS-style Task Implementations
// ============================================

// In real FreeRTOS, tasks are functions that never return
// and use vTaskDelay() or vTaskDelayUntil() for timing

void vTaskLED1(void* param)
{
    // Real FreeRTOS task would have:
    // while(1) {
    //     digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));
    //     vTaskDelay(pdMS_TO_TICKS(200));
    // }
    (void)param;
    static bool state = false;
    state = !state;
    digitalWrite(LED1_PIN, state);
}

void vTaskLED2(void* param)
{
    (void)param;
    static bool state = false;
    state = !state;
    digitalWrite(LED2_PIN, state);
}

void vTaskServo(void* param)
{
    (void)param;
    static int angle = 0;
    angle = (angle + 10) % 180;
    servo.write(angle);
}

void vTaskSerial(void* param)
{
    (void)param;
    static int count = 0;
    Serial.print("[FreeRTOS Task] Count: ");
    Serial.println(count++);
}

void vTaskDisplay(void* param)
{
    (void)param;
    static uint8_t frame[8][12];
    static int pos = 0;

    memset(frame, 0, sizeof(frame));

    // Moving dot animation
    frame[3][pos] = 1;
    frame[4][pos] = 1;
    pos = (pos + 1) % 12;

    matrix.renderBitmap(frame, 8, 12);
}

// ============================================
// Comparison: Our OS vs FreeRTOS
// ============================================

void show_api_comparison(void)
{
    Serial.println();
    Serial.println("============================================");
    Serial.println("       Our OS  vs  FreeRTOS  Comparison");
    Serial.println("============================================");
    Serial.println();
    Serial.println("Task Creation:");
    Serial.println("  Our OS:   os_create_task(func, name, stack)");
    Serial.println("  FreeRTOS: xTaskCreate(func, name, stack, param, priority, handle)");
    Serial.println();
    Serial.println("Delay:");
    Serial.println("  Our OS:   os_sleep(ms)");
    Serial.println("  FreeRTOS: vTaskDelay(pdMS_TO_TICKS(ms))");
    Serial.println();
    Serial.println("Task Control:");
    Serial.println("  Our OS:   os_suspend_task(id) / os_resume_task(id)");
    Serial.println("  FreeRTOS: vTaskSuspend(handle) / vTaskResume(handle)");
    Serial.println();
    Serial.println("Semaphores (FreeRTOS only):");
    Serial.println("  xSemaphoreCreateBinary()");
    Serial.println("  xSemaphoreTake(sem, timeout)");
    Serial.println("  xSemaphoreGive(sem)");
    Serial.println();
    Serial.println("Queues (FreeRTOS only):");
    Serial.println("  xQueueCreate(length, itemSize)");
    Serial.println("  xQueueSend(queue, &item, timeout)");
    Serial.println("  xQueueReceive(queue, &item, timeout)");
    Serial.println();
    Serial.println("============================================");
    Serial.println();
}

// Simulated scheduler loop
unsigned long task_timers[MAX_FAKE_TASKS] = {0};
unsigned long task_periods[] = {200, 500, 100, 1000, 150};

void fake_scheduler_run(void)
{
    unsigned long now = millis();
    for (int i = 0; i < fake_task_count && i < 5; i++) {
        if (!fake_tasks[i].suspended && now - task_timers[i] >= task_periods[i]) {
            task_timers[i] = now;
            fake_tasks[i].func(fake_tasks[i].param);
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println();
    Serial.println("=== FreeRTOS Concepts Demo ===");
    Serial.println();
    Serial.println("This demonstrates FreeRTOS-style task creation.");
    Serial.println("Note: For actual FreeRTOS, use Arduino_FreeRTOS library.");
    Serial.println();

    // Initialize hardware
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    servo.attach(SERVO_PIN);
    matrix.begin();

    // Create tasks FreeRTOS-style
    xTaskCreate(vTaskLED1, "LED1", 128, NULL, 1, NULL);
    xTaskCreate(vTaskLED2, "LED2", 128, NULL, 1, NULL);
    xTaskCreate(vTaskServo, "Servo", 256, NULL, 2, NULL);
    xTaskCreate(vTaskSerial, "Serial", 512, NULL, 1, NULL);
    xTaskCreate(vTaskDisplay, "Display", 256, NULL, 1, NULL);

    Serial.println();

    // Show API comparison
    show_api_comparison();

    Serial.println("Starting FreeRTOS-style scheduler...");
    Serial.println();

    // In real FreeRTOS, you would call: vTaskStartScheduler();
    // The scheduler never returns in FreeRTOS
}

void loop()
{
    // Simulated FreeRTOS scheduler
    fake_scheduler_run();
}
