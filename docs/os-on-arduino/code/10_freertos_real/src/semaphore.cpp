// 第10章 10.7「セマフォの例」の掲載例
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>   // semphr.h などもここから取り込まれる

SemaphoreHandle_t xSemaphore = NULL;
volatile int sharedData = 0;

void task_producer(void *pvParameters)
{
    for (;;) {
        sharedData++;

        // セマフォを与える（シグナル）
        xSemaphoreGive(xSemaphore);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_consumer(void *pvParameters)
{
    for (;;) {
        // セマフォを待つ
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.print("Received: ");
            Serial.println(sharedData);
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // バイナリセマフォ作成
    xSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(task_producer, "Producer", 128, NULL, 1, NULL);
    xTaskCreate(task_consumer, "Consumer", 128, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() { }
