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

    interp_init();
    os_init();

    os_create_task(task_led, "LED", 512);
    os_create_task(shell_task, "Shell", 1024);
    os_create_task(display_task, "Display", 512);

    os_start();
}

void loop() { }
