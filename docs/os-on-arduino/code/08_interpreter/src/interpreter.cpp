// interpreter.cpp

#include "interpreter.h"
#include "os_kernel.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <Servo.h>

// スクリプト格納
static char g_script[MAX_SCRIPT_LINES][MAX_LINE_LENGTH];
static int g_script_lines = 0;
static int g_current_line = 0;
static bool g_running = false;

// ループスタック
#define MAX_LOOP_DEPTH 4
static int g_loop_stack[MAX_LOOP_DEPTH];
static int g_loop_count[MAX_LOOP_DEPTH];
static int g_loop_depth = 0;

// ハードウェア
static Servo g_servos[2];
static bool g_servo_attached[2] = {false, false};

// サーボピン
#define SERVO_PIN_0 9
#define SERVO_PIN_1 10

// モーター制御（ダミー実装）
static void motor_forward(int distance)
{
    Serial.print("[Motor] Forward ");
    Serial.print(distance);
    Serial.println(" units");
    // 実際のモーター制御はここに実装
    os_sleep(distance * 10);  // 仮の待機
}

static void motor_backward(int distance)
{
    Serial.print("[Motor] Backward ");
    Serial.print(distance);
    Serial.println(" units");
    os_sleep(distance * 10);
}

static void motor_turn(int angle)
{
    Serial.print("[Motor] Turn ");
    Serial.print(angle);
    Serial.println(" degrees");
    os_sleep(abs(angle) * 10);
}

// サーボ制御
static void servo_set(int channel, int angle)
{
    if (channel < 0 || channel > 1) {
        Serial.println("[Error] Invalid servo channel");
        return;
    }

    // 必要に応じてアタッチ
    if (!g_servo_attached[channel]) {
        int pin = (channel == 0) ? SERVO_PIN_0 : SERVO_PIN_1;
        g_servos[channel].attach(pin);
        g_servo_attached[channel] = true;
    }

    // 角度を制限
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    g_servos[channel].write(angle);

    Serial.print("[Servo] Ch");
    Serial.print(channel);
    Serial.print(" = ");
    Serial.print(angle);
    Serial.println(" deg");
}

// LED制御
static void led_set(int pin, int state)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, state ? HIGH : LOW);

    Serial.print("[LED] Pin ");
    Serial.print(pin);
    Serial.print(" = ");
    Serial.println(state ? "ON" : "OFF");
}

// 空白をスキップ
static const char* skip_spaces(const char *cursor)
{
    while (*cursor == ' ' || *cursor == '\t') cursor++;
    return cursor;
}

// 数値を解析
static int parse_int(const char **cursor_ptr)
{
    const char *cursor = skip_spaces(*cursor_ptr);
    int sign = 1;
    int value = 0;

    if (*cursor == '-') {
        sign = -1;
        cursor++;
    }

    while (*cursor >= '0' && *cursor <= '9') {
        value = value * 10 + (*cursor - '0');
        cursor++;
    }

    *cursor_ptr = cursor;
    return sign * value;
}

// 1行を実行
static ExecResult execute_line(const char *line)
{
    const char *cursor = skip_spaces(line);

    // 空行やコメント
    if (*cursor == '\0' || *cursor == '#') {
        return EXEC_OK;
    }

    // コマンドを取得
    char cmd[16] = {0};
    int i = 0;
    while (*cursor && *cursor != ' ' && *cursor != '\t' && i < 15) {
        cmd[i++] = *cursor++;
    }

    // コマンド実行
    if (strcmp(cmd, "FORWARD") == 0) {
        int dist = parse_int(&cursor);
        motor_forward(dist);
    }
    else if (strcmp(cmd, "BACKWARD") == 0) {
        int dist = parse_int(&cursor);
        motor_backward(dist);
    }
    else if (strcmp(cmd, "TURN") == 0) {
        int angle = parse_int(&cursor);
        motor_turn(angle);
    }
    else if (strcmp(cmd, "SERVO") == 0) {
        int ch = parse_int(&cursor);
        int angle = parse_int(&cursor);
        servo_set(ch, angle);
    }
    else if (strcmp(cmd, "LED") == 0) {
        int pin = parse_int(&cursor);
        int state = parse_int(&cursor);
        led_set(pin, state);
    }
    else if (strcmp(cmd, "DELAY") == 0) {
        int ms = parse_int(&cursor);
        Serial.print("[Delay] ");
        Serial.print(ms);
        Serial.println(" ms");
        os_sleep(ms);
    }
    else if (strcmp(cmd, "PRINT") == 0) {
        cursor = skip_spaces(cursor);
        Serial.print("[Print] ");
        Serial.println(cursor);
    }
    else if (strcmp(cmd, "LOOP") == 0) {
        if (g_loop_depth >= MAX_LOOP_DEPTH) {
            Serial.println("[Error] Loop too deep");
            return EXEC_ERROR;
        }
        int count = parse_int(&cursor);
        g_loop_stack[g_loop_depth] = g_current_line;
        g_loop_count[g_loop_depth] = count;
        g_loop_depth++;
    }
    else if (strcmp(cmd, "END") == 0) {
        if (g_loop_depth <= 0) {
            Serial.println("[Error] END without LOOP");
            return EXEC_ERROR;
        }
        g_loop_count[g_loop_depth - 1]--;
        if (g_loop_count[g_loop_depth - 1] > 0) {
            // ループ継続
            g_current_line = g_loop_stack[g_loop_depth - 1];
        } else {
            // ループ終了
            g_loop_depth--;
        }
    }
    else {
        Serial.print("[Error] Unknown command: ");
        Serial.println(cmd);
        return EXEC_ERROR;
    }

    return EXEC_OK;
}

void interp_init(void)
{
    g_script_lines = 0;
    g_current_line = 0;
    g_running = false;
    g_loop_depth = 0;
}

int interp_load(const char *script)
{
    interp_init();

    const char *cursor = script;
    int line = 0;

    while (*cursor && line < MAX_SCRIPT_LINES) {
        int i = 0;
        while (*cursor && *cursor != '\n' && i < MAX_LINE_LENGTH - 1) {
            g_script[line][i++] = *cursor++;
        }
        g_script[line][i] = '\0';

        // 空行でなければカウント
        if (i > 0) {
            line++;
        }

        if (*cursor == '\n') cursor++;
    }

    g_script_lines = line;
    g_current_line = 0;

    Serial.print("[Interpreter] Loaded ");
    Serial.print(g_script_lines);
    Serial.println(" lines");

    return g_script_lines;
}

ExecResult interp_step(void)
{
    if (g_current_line >= g_script_lines) {
        g_running = false;
        return EXEC_END;
    }

    g_running = true;
    ExecResult result = execute_line(g_script[g_current_line]);
    g_current_line++;

    return result;
}

int interp_run(void)
{
    Serial.println("\n[Interpreter] Starting script...");

    int executed = 0;
    while (g_current_line < g_script_lines) {
        ExecResult result = interp_step();
        if (result == EXEC_ERROR) {
            Serial.println("[Interpreter] Script error, aborting");
            return -1;
        }
        executed++;
    }

    Serial.print("[Interpreter] Script completed: ");
    Serial.print(executed);
    Serial.println(" commands executed");

    g_running = false;
    return executed;
}

bool interp_is_running(void)
{
    return g_running;
}
