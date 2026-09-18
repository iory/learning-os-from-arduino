/**
 * Chapter 2: Cooperative Multitasking Scheduler
 *
 * Demonstrates cooperative (non-preemptive) multitasking.
 * Each task runs to completion before another task can run.
 *
 * Problem: If one task takes too long (like heavy_task),
 * all other tasks are delayed.
 */

#include <Arduino.h>

#define LED_PIN LED_BUILTIN
#define SERVO_PIN 9

// --- Scheduler Definition ---
typedef void (*TaskFunc)(void);

struct Task {
    TaskFunc func;
    unsigned long interval;
    unsigned long last_run;
    const char* name;
};

#define MAX_TASKS 4
Task tasks[MAX_TASKS];
int task_count = 0;

void scheduler_add_task(TaskFunc func, unsigned long interval, const char* name)
{
    if (task_count < MAX_TASKS) {
        tasks[task_count].func = func;
        tasks[task_count].interval = interval;
        tasks[task_count].last_run = 0;
        tasks[task_count].name = name;
        task_count++;
    }
}

void scheduler_run(void)
{
    unsigned long now = millis();
    for (int i = 0; i < task_count; i++) {
        if (now - tasks[i].last_run >= tasks[i].interval) {
            tasks[i].last_run = now;

            Serial.print("[");
            Serial.print(now);
            Serial.print("ms] Running: ");
            Serial.println(tasks[i].name);

            tasks[i].func();
        }
    }
}

// --- Task Definitions ---
void led_task(void)
{
    static bool state = false;
    state = !state;
    digitalWrite(LED_PIN, state);
    Serial.println("  LED toggled");
}

void servo_task(void)
{
    static int angle = 0;
    angle = (angle + 10) % 180;
    Serial.print("  Servo angle: ");
    Serial.println(angle);
}

void sensor_task(void)
{
    int value = analogRead(A0);
    Serial.print("  Sensor value: ");
    Serial.println(value);
}

void heavy_task(void)
{
    // Simulate heavy processing
    Serial.println("  Heavy task start...");
    delay(200);  // 200ms blocking!
    Serial.println("  Heavy task end");
}

// --- Setup and Loop ---
void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    pinMode(LED_PIN, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);

    Serial.println("=== Cooperative Scheduler Demo ===");
    Serial.println();
    Serial.println("This demonstrates cooperative multitasking.");
    Serial.println("Uncomment heavy_task to see the problem with blocking tasks.");
    Serial.println();

    // Register tasks
    scheduler_add_task(led_task, 500, "LED");
    scheduler_add_task(servo_task, 100, "Servo");
    scheduler_add_task(sensor_task, 200, "Sensor");
    // Uncomment the following line to see the problem:
    // scheduler_add_task(heavy_task, 1000, "Heavy");

    Serial.println("Tasks registered. Starting scheduler...");
    Serial.println();
}

void loop()
{
    scheduler_run();
}
