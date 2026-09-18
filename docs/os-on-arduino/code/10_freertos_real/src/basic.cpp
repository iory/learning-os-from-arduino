// 第10章 10.3「FreeRTOSの基本」の掲載例
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

// タスク関数
void task_led(void *pvParameters)
{
    pinMode(2, OUTPUT);

    for (;;) {  // FreeRTOSではwhile(1)の代わりにfor(;;)が慣例
        digitalWrite(2, !digitalRead(2));
        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms待機
    }
}

void setup()
{
    // タスク作成
    xTaskCreate(
        task_led,      // タスク関数
        "LED",         // タスク名
        128,           // スタックサイズ（ワード単位）
        NULL,          // パラメータ
        1,             // 優先度
        NULL           // タスクハンドル
    );

    // スケジューラ開始
    vTaskStartScheduler();
}

void loop()
{
    // FreeRTOS使用時は到達しない
}
