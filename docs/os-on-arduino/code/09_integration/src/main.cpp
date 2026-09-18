// 08_integrated_os.ino
// 統合版自作OS

#include "os_kernel.h"
#include "os_fault.h"
#include "os_display.h"
#include "shell.h"
#include "interpreter.h"
#include <Servo.h>

// ハードウェアピン定義
// Arduino UNO R4 WiFi は基板上に "L" と印字された LED を持っています。
// 追加部品なしで動かせるよう、これを主 LED として使います。
#define LED1_PIN LED_BUILTIN
#define LED2_PIN 3
#define LED3_PIN 4
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define BUTTON_PIN 7
#define POT_PIN A0

// グローバル変数（タスク間共有）
volatile int g_pot_value = 0;
volatile bool g_button_pressed = false;
Servo servo1, servo2;

// ===========================================
// タスク定義
// ===========================================

// タスク1: システムハートビート
void task_heartbeat(void)
{
    pinMode(LED1_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED1_PIN, HIGH);
        os_sleep(100);
        digitalWrite(LED1_PIN, LOW);
        os_sleep(900);
    }
}

// タスク2: センサ読み取り
// 注: ここではanalogRead()を使っているが、内部のレジスタ操作については
//     Chapter 12でADC14のレジスタを直接操作する方法を学ぶ。
void task_sensor(void)
{
    pinMode(POT_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    while (1) {
        g_pot_value = analogRead(POT_PIN);
        g_button_pressed = (digitalRead(BUTTON_PIN) == LOW);
        os_sleep(50);
    }
}

// タスク3: サーボ制御（ポテンショメータ連動）
// 注: ここではServoライブラリ（またはServoPWM）を使っているが、
//     ソフトウェアPWMはコンテキストスイッチの影響でジッターする可能性がある。
//     Chapter 12でGPTタイマを使ったハードウェアPWMに改善する。
void task_servo(void)
{
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);

    while (1) {
        int angle = map(g_pot_value, 0, 1023, 0, 180);
        servo1.write(angle);
        servo2.write(180 - angle);  // 逆方向
        os_sleep(20);
    }
}

// タスク4: LED表示（センサ値を3段階表示）
void task_led_indicator(void)
{
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);

    while (1) {
        int level = g_pot_value / 341;  // 0-2の3段階

        digitalWrite(LED2_PIN, level >= 1);
        digitalWrite(LED3_PIN, level >= 2);

        os_sleep(100);
    }
}

// タスク5: ボタン監視（押すたびにイベント発生）
void task_button(void)
{
    bool last_state = false;
    int press_count = 0;

    while (1) {
        if (g_button_pressed && !last_state) {
            // 押された瞬間
            press_count++;
            Serial.print("[Button] Pressed! Count: ");
            Serial.println(press_count);
        }
        last_state = g_button_pressed;
        os_sleep(20);
    }
}

// ===========================================
// セットアップ
// ===========================================

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    // スタートアップメッセージ
    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║     MiniOS for Arduino UNO R4 WiFi     ║");
    Serial.println("║          Version 1.0                   ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();

    // 例外設定
    fault_init();

    // OS初期化
    os_init();
    interp_init();

    Serial.println("Creating tasks...");

    // タスク作成
    os_create_task(task_heartbeat, "Heartbeat", 256);
    os_create_task(task_sensor, "Sensor", 512);
    os_create_task(task_servo, "Servo", 512);
    os_create_task(task_led_indicator, "LED_Ind", 256);
    os_create_task(task_button, "Button", 512);
    os_create_task(shell_task, "Shell", 1024);
    os_create_task(display_task, "Display", 512);

    Serial.println();
    Serial.println("Tasks created:");
    for (int i = 0; i < os_get_task_count(); i++) {
        Serial.print("  ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(os_get_task_name(i));
    }

    Serial.println();
    Serial.println("Type 'help' for available commands.");
    Serial.println("Type 'run' to execute motion scripts.");
    Serial.println();

    // OS開始
    os_start();
}

void loop()
{
    // 到達しない
}
