/**
 * Chapter 12: Hardware PWM and ADC
 *
 * 本書 12.4「OS との統合」の After 版をそのまま動かすデモ。
 *
 * - task_servo   : GPT のハードウェア PWM でサーボを駆動（レジスタ書き込みのみ）
 * - task_sensor  : ADC14 を直接叩いて 14bit で読む
 * - task_heavy   : わざと CPU を占有し、それでもサーボが揺れないことを見る
 * - shell        : ps / kill / exec でタスクを触る
 *
 * 配線:
 *   D9 -- サーボ信号線（サーボの電源は別系統から取ること）
 *   A0 -- ポテンショメータのワイパ
 *   D2 -- LED（ハートビート）
 */

#include <Arduino.h>
#include "os_kernel.h"
#include "os_fault.h"
#include "shell.h"
#include "hardware_pwm.h"
#include "raw_adc.h"

#define LED_PIN    2
#define SERVO_PIN  9
#define POT_CH     9   // A0 = P014 = AN009 (ch0 は A1)

// タスク間共有データ（本書 9.2「タスク間共有データ」を参照）
volatile uint16_t g_pot_raw = 0;   // 14bit 生値
volatile int      g_pot_10bit = 0; // 10bit 換算

static HardwarePWM servo1;

// ハートビート: OS が生きていることを示す
void task_led(void)
{
    pinMode(LED_PIN, OUTPUT);
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        os_sleep(500);
    }
}

// センサ: ADC14 を直接読む（12.3）
void task_sensor(void)
{
    raw_adc_init();
    while (1) {
        g_pot_raw   = raw_analog_read(POT_CH);
        g_pot_10bit = raw_analog_read_10bit(POT_CH);
        os_sleep(50);
    }
}

// サーボ: ハードウェア PWM で駆動（12.2）
// write() はレジスタに書くだけなので CPU を占有しない。
// refresh() も不要で、波形は GPT が自動で出し続ける。
void task_servo(void)
{
    if (!servo1.attach(SERVO_PIN)) {
        Serial.println(F("[Error] servo attach failed"));
        while (1) os_sleep(1000);
    }
    while (1) {
        int angle = map(g_pot_10bit, 0, 1023, 0, 180);
        servo1.write(angle);
        os_sleep(20);
    }
}

// 重い計算: CPU を食い続ける。
// ソフトウェア PWM ならここでサーボが暴れるが、GPT なら影響を受けない。
void task_heavy(void)
{
    while (1) {
        volatile long sum = 0;
        for (long i = 0; i < 200000; i++) {
            sum += i;
        }
        os_sleep(1);
    }
}

// 状態表示
void task_status(void)
{
    while (1) {
        Serial.print(F("pot(14bit)="));
        Serial.print(g_pot_raw);
        Serial.print(F("  pot(10bit)="));
        Serial.print(g_pot_10bit);
        Serial.print(F("  servo="));
        Serial.print(servo1.read());
        Serial.println(F("deg"));
        os_sleep(1000);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println(F("=== Chapter 12: Hardware PWM / ADC ==="));
    Serial.println(F("Turn the pot on A0. The servo on D9 should follow"));
    Serial.println(F("smoothly even while 'Heavy' is hogging the CPU."));
    Serial.println(F("Try 'kill 4' to stop Heavy and compare.\n"));

    os_init();
    fault_init();

    os_create_task(task_led,    "LED",    512);
    os_create_task(task_sensor, "Sensor", 512);
    os_create_task(task_servo,  "Servo",  512);
    os_create_task(task_heavy,  "Heavy",  512);
    os_create_task(task_status, "Status", 512);
    os_create_task(shell_task,  "Shell",  1024);

    os_start();
}

void loop() { }
