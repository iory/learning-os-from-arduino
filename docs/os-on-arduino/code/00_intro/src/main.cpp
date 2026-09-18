// 00_check_environment.ino
// 第0章「準備：開発環境の確認」のスケッチ。
// シリアル出力と LED マトリクスの動作確認を行う。

#include <Arduino.h>
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println("=== Arduino UNO R4 WiFi OS Exercise ===");
    Serial.print("CPU Frequency: ");
    Serial.print(F_CPU / 1000000);
    Serial.println(" MHz");

    matrix.begin();

    // LEDマトリクスに "OS" と表示
    uint8_t frame[8][12] = {0};
    // 'O'（左端1列空けて配置）
    frame[1][2] = 1; frame[1][3] = 1; frame[1][4] = 1;
    frame[2][1] = 1; frame[2][5] = 1;
    frame[3][1] = 1; frame[3][5] = 1;
    frame[4][1] = 1; frame[4][5] = 1;
    frame[5][2] = 1; frame[5][3] = 1; frame[5][4] = 1;
    // 'S'（1列右にずらして配置）
    frame[1][8] = 1; frame[1][9] = 1; frame[1][10] = 1;
    frame[2][7] = 1;
    frame[3][8] = 1; frame[3][9] = 1;
    frame[4][10] = 1;
    frame[5][7] = 1; frame[5][8] = 1; frame[5][9] = 1;

    matrix.renderBitmap(frame, 8, 12);

    Serial.println("Environment OK! Ready to start.");
}

void loop()
{
    static int count = 0;
    Serial.print("Loop count: ");
    Serial.println(count++);
    delay(1000);
}
