// 第10章 10.8「キュー（メッセージパッシング）」の掲載例
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>   // queue.h などもここから取り込まれる

QueueHandle_t xQueue = NULL;

typedef struct {
    int sensorId;
    int value;
} SensorData;

void task_sensor(void *pvParameters)
{
    for (;;) {
        SensorData data;
        data.sensorId = 1;
        data.value = analogRead(A0);

        // キューに送信
        xQueueSend(xQueue, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void task_processor(void *pvParameters)
{
    SensorData received;

    for (;;) {
        // キューから受信
        if (xQueueReceive(xQueue, &received, portMAX_DELAY) == pdTRUE) {
            Serial.print("Sensor ");
            Serial.print(received.sensorId);
            Serial.print(": ");
            Serial.println(received.value);
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // キュー作成（10要素）
    xQueue = xQueueCreate(10, sizeof(SensorData));

    xTaskCreate(task_sensor, "Sensor", 128, NULL, 1, NULL);
    xTaskCreate(task_processor, "Processor", 256, NULL, 2, NULL);

    vTaskStartScheduler();
}

void loop() { }
