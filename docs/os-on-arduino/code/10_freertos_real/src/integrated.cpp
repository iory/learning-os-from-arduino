// freertos_integrated.ino（第10章 10.10 の掲載例）
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>   // semphr.h などもここから取り込まれる
#include <Servo.h>

// 基板上の「L」LED。追加部品なしで動かせるようにこれを使います
#define LED_PIN LED_BUILTIN
#define SERVO_PIN 9
#define POT_PIN A0

// 共有データ
// R4 同梱の FreeRTOS はミューテックスが無効なので、
// バイナリセマフォを「1 個の鍵」として使って排他する (10.7 参照)
volatile int g_pot_value = 0;
SemaphoreHandle_t xDataMutex = NULL;

// サーボ
Servo servo;

// タスク: センサ読み取り
void task_sensor(void *pvParameters)
{
    for (;;) {
        int value = analogRead(POT_PIN);

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            g_pot_value = value;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// タスク: サーボ制御
void task_servo(void *pvParameters)
{
    servo.attach(SERVO_PIN);

    for (;;) {
        int value;
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            value = g_pot_value;
            xSemaphoreGive(xDataMutex);
        }

        int angle = map(value, 0, 1023, 0, 180);
        servo.write(angle);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// タスク: LED点滅
void task_led(void *pvParameters)
{
    pinMode(LED_PIN, OUTPUT);

    for (;;) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// タスク: 状態表示
void task_monitor(void *pvParameters)
{
    for (;;) {
        Serial.println("=== FreeRTOS Status ===");
        Serial.print("Tick: ");
        Serial.println(xTaskGetTickCount());
        Serial.print("Free heap: ");
        Serial.println(xPortGetFreeHeapSize());
        Serial.print("Pot value: ");
        Serial.println(g_pot_value);
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== FreeRTOS Integrated Demo ===\n");

    // 排他用のバイナリセマフォを作成し、最初は「空き」にしておく
    xDataMutex = xSemaphoreCreateBinary();
    xSemaphoreGive(xDataMutex);

    // タスク作成
    xTaskCreate(task_sensor, "Sensor", 128, NULL, 2, NULL);
    xTaskCreate(task_servo, "Servo", 128, NULL, 2, NULL);
    xTaskCreate(task_led, "LED", 128, NULL, 1, NULL);
    xTaskCreate(task_monitor, "Monitor", 256, NULL, 1, NULL);

    // スケジューラ開始
    vTaskStartScheduler();
}

void loop() { }
