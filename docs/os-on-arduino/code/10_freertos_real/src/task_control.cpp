// freertos_task_control.ino（第10章 10.5 の掲載例）
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

TaskHandle_t ledTaskHandle = NULL;

void task_led(void *pvParameters)
{
    pinMode(2, OUTPUT);
    for (;;) {
        digitalWrite(2, !digitalRead(2));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void task_shell(void *pvParameters)
{
    for (;;) {
        if (Serial.available()) {
            String cmd = Serial.readStringUntil('\n');
            cmd.trim();

            if (cmd == "suspend") {
                if (ledTaskHandle != NULL) {
                    vTaskSuspend(ledTaskHandle);  // タスク停止
                    Serial.println("LED task suspended");
                }
            }
            else if (cmd == "resume") {
                if (ledTaskHandle != NULL) {
                    vTaskResume(ledTaskHandle);   // タスク再開
                    Serial.println("LED task resumed");
                }
            }
            else if (cmd == "info") {
                // タスク情報表示
                Serial.print("Free heap: ");
                Serial.println(xPortGetFreeHeapSize());

                Serial.print("Tick count: ");
                Serial.println(xTaskGetTickCount());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== FreeRTOS Task Control Demo ===");
    Serial.println("Commands: suspend, resume, info");

    xTaskCreate(task_led, "LED", 128, NULL, 1, &ledTaskHandle);
    xTaskCreate(task_shell, "Shell", 256, NULL, 2, NULL);

    vTaskStartScheduler();
}

void loop() { }
