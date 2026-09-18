// freertos_preemptive.ino（第10章 10.4 の掲載例）
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

void task_led1(void *pvParameters)
{
    pinMode(2, OUTPUT);
    for (;;) {
        digitalWrite(2, !digitalRead(2));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_led2(void *pvParameters)
{
    pinMode(3, OUTPUT);
    for (;;) {
        digitalWrite(3, !digitalRead(3));
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// 重い計算タスク（yield不要！）
void task_heavy(void *pvParameters)
{
    pinMode(4, OUTPUT);
    for (;;) {
        volatile long sum = 0;
        for (long i = 0; i < 1000000; i++) {
            sum += i;
        }
        digitalWrite(4, !digitalRead(4));
        // vTaskDelayなしでもプリエンプションで切り替わる
    }
}

void setup()
{
    Serial.begin(115200);

    xTaskCreate(task_led1, "LED1", 128, NULL, 1, NULL);
    xTaskCreate(task_led2, "LED2", 128, NULL, 1, NULL);
    xTaskCreate(task_heavy, "Heavy", 256, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() { }
