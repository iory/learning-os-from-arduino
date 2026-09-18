/**
 * Chapter 6: Memory Protection Demo
 *
 * Demonstrates HardFault handling:
 * - Divide by zero detection
 * - Invalid memory access detection
 * - Crashed task termination (other tasks continue)
 */

#include <Arduino.h>
#include "os_kernel.h"
#include "os_fault.h"

// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED_PIN LED_BUILTIN

// Normal task: blinks LED
void task_led(void)
{
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, HIGH);
        os_sleep(500);
        digitalWrite(LED_PIN, LOW);
        os_sleep(500);
    }
}

// Normal task: periodic output
void task_heartbeat(void)
{
    int count = 0;
    while (1) {
        Serial.print("[Heartbeat] ");
        Serial.println(count++);
        os_sleep(2000);
    }
}

// Crash task: divide by zero
void task_crash_divzero(void)
{
    Serial.println("[Crash] Dividing by zero in 3 seconds...");
    Serial.flush();
    os_sleep(3000);

    // Ensure Serial is idle before crashing
    Serial.flush();
    os_sleep(100);

    volatile int a = 100;
    volatile int b = 0;
    volatile int c = a / b;  // This will trigger HardFault!
    (void)c;
}

// Crash task: invalid memory access
void task_crash_memory(void)
{
    Serial.println("[Crash] Invalid memory access in 5 seconds...");
    os_sleep(5000);

    volatile uint32_t *bad_ptr = (uint32_t *)0xFFFFFFFF;
    *bad_ptr = 0x12345678;  // This will trigger HardFault!
}

// Shell-like task for control
void task_shell(void)
{
    Serial.println();
    Serial.println("Commands: 'd' = div by zero, 'm' = bad memory, 'p' = ps");

    while (1) {
        // Print any pending fault messages (deferred from ISR context)
        fault_print_pending();

        if (Serial.available()) {
            char c = Serial.read();

            if (c == 'd') {
                Serial.println("Creating div-by-zero crash task...");
                os_create_task(task_crash_divzero, "Crash_Div", 512);
            } else if (c == 'm') {
                Serial.println("Creating bad-memory crash task...");
                os_create_task(task_crash_memory, "Crash_Mem", 512);
            } else if (c == 'p') {
                Serial.println();
                Serial.println("ID  NAME           STATE");
                for (int i = 0; i < os_get_task_count(); i++) {
                    Serial.print(i);
                    Serial.print("   ");
                    Serial.print(os_get_task_name(i));
                    Serial.print("     ");
                    TaskState s = os_get_task_state(i);
                    Serial.println(s == TASK_READY ? "READY" :
                                   s == TASK_RUNNING ? "RUNNING" :
                                   s == TASK_BLOCKED ? "BLOCKED" :
                                   s == TASK_TERMINATED ? "TERMINATED" : "?");
                }
            }
        }
        os_sleep(100);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== Memory Protection Demo ===");
    Serial.println();
    Serial.println("This demo shows fault handling.");
    Serial.println("When a task crashes, only that task is terminated.");
    Serial.println("Other tasks continue running.");
    Serial.println();

    // Initialize fault handling
    fault_init();

    // Initialize OS
    os_init();

    // Create tasks
    os_create_task(task_led, "LED", 512);
    os_create_task(task_heartbeat, "Heartbeat", 512);
    os_create_task(task_shell, "Shell", 1024);

    Serial.println("Press 'd' for divide-by-zero crash");
    Serial.println("Press 'm' for bad memory access crash");
    Serial.println("Press 'p' to show task list");
    Serial.println();

    os_start();
}

void loop()
{
    // Never reached
}
