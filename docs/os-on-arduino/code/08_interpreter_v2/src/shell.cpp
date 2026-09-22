/**
 * shell.cpp - Interactive Shell Implementation
 * Chapter 5: Interactive Shell
 */

#include "shell.h"
#include "os_kernel.h"
#include "os_fault.h"
#include "interpreter.h"
#include "os_display.h"
#include <Arduino.h>
#include <string.h>

#define CMD_BUF_SIZE 64

static char cmd_buf[CMD_BUF_SIZE];
static int cmd_pos = 0;

static void cmd_run(const char *arg);

// top: マトリクスが計測するたび（0.5 秒ごと）に画面を描き直す。もう一度 top で止める
static bool top_on = false;
static uint32_t top_drawn = 0;

static const char* state_to_str(TaskState state)
{
    switch (state) {
        case TASK_READY:      return "READY";
        case TASK_RUNNING:    return "RUNNING";
        case TASK_BLOCKED:    return "BLOCKED";
        case TASK_SUSPENDED:  return "SUSPEND";
        case TASK_TERMINATED: return "TERM";
        default:              return "?";
    }
}

static void cmd_help(void)
{
    Serial.println("Available commands:");
    Serial.println("  ps      - List all tasks");
    Serial.println("  top     - Live CPU view like the matrix (type top again to stop)");
    Serial.println("  kill N  - Suspend task N");
    Serial.println("  exec N  - Resume task N");
    Serial.println("  info    - System information");
    Serial.println("  reboot  - Reboot system");
    Serial.println("  help    - Show this help");
}

static void cmd_ps(void)
{
    Serial.println();
    Serial.println("ID  NAME           STATE      CPU%");
    Serial.println("--  ----           -----      ----");

    for (int i = 0; i < os_get_task_count(); i++) {
        Serial.print(i);
        Serial.print("   ");

        const char *name = os_get_task_name(i);
        Serial.print(name);
        for (int j = strlen(name); j < 15; j++) Serial.print(" ");

        Serial.print(state_to_str(os_get_task_state(i)));
        for (int j = strlen(state_to_str(os_get_task_state(i))); j < 10; j++) Serial.print(" ");

        Serial.print(os_get_cpu_usage(i));
        Serial.println("%");
    }
    Serial.println();
}

static void pad(const char *s, int width)
{
    Serial.print(s);
    for (int j = strlen(s); j < width; j++) Serial.print(" ");
}

// 画面を消して表を描き、その下にプロンプトと入力途中の文字を描き直す
static void top_draw(const char *typed)
{
    int load = display_get_load();
    int load_len = display_get_bar_length(load);
    Serial.print("\x1b[H\x1b[2J");
    Serial.print("CPU [");
    for (int j = 0; j < 12; j++) Serial.print(j < load_len ? "#" : " ");
    Serial.print("] ");
    Serial.print(load);
    Serial.println("%");
    Serial.println();
    Serial.println("ID  NAME      STATE     CPU%  MATRIX");
    for (int i = 0; i < os_get_task_count(); i++) {
        Serial.print(i);
        Serial.print("   ");
        pad(os_get_task_name(i), 10);
        pad(state_to_str(os_get_task_state(i)), 9);
        int pct = display_get_task_percent(i);
        if (pct < 100) Serial.print(" ");
        if (pct < 10) Serial.print(" ");
        Serial.print(pct);
        Serial.print("%  ");
        if (i >= 1 && i <= 5) {   // マトリクスの行 3-7 に出ているタスク
            int len = display_get_bar_length(pct);
            for (int j = 0; j < len; j++) Serial.print("#");
        }
        Serial.println();
    }
    Serial.println();
    Serial.print("> ");
    Serial.print(typed);
}

static void cmd_top(void)
{
    top_on = !top_on;
    if (top_on) {
        top_drawn = display_get_sample_count();
        top_draw("");
    }
}

static void cmd_kill(int id)
{
    if (id <= 0) {
        Serial.println("Error: Cannot kill idle task (ID 0)");
        return;
    }
    if (id >= os_get_task_count()) {
        Serial.println("Error: Invalid task ID");
        return;
    }

    os_suspend_task(id);
    Serial.print("Task ");
    Serial.print(id);
    Serial.print(" (");
    Serial.print(os_get_task_name(id));
    Serial.println(") suspended.");
}

static void cmd_exec(int id)
{
    if (id <= 0 || id >= os_get_task_count()) {
        Serial.println("Error: Invalid task ID");
        return;
    }

    os_resume_task(id);
    Serial.print("Task ");
    Serial.print(id);
    Serial.print(" (");
    Serial.print(os_get_task_name(id));
    Serial.println(") resumed.");
}

static void cmd_info(void)
{
    Serial.println();
    Serial.println("=== System Information ===");
    Serial.print("System Ticks:  ");
    Serial.println(os_get_tick());
    Serial.print("Uptime:        ");
    Serial.print(os_get_tick() / 1000);
    Serial.println(" s");
    Serial.print("Task Count:    ");
    Serial.println(os_get_task_count());
    Serial.print("Time Slice:    ");
    Serial.print(TIME_SLICE_MS);
    Serial.println(" ms");
    Serial.println();
}

static void cmd_reboot(void)
{
    Serial.println("Rebooting...");
    delay(100);
    NVIC_SystemReset();
}

void shell_process_command(char *cmd)
{
    // Trim whitespace
    while (*cmd == ' ') cmd++;
    char *end = cmd + strlen(cmd) - 1;
    while (end > cmd && *end == ' ') *end-- = '\0';

    if (strlen(cmd) == 0) return;

    // Parse command
    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        cmd_help();
    } else if (strcmp(cmd, "ps") == 0) {
        cmd_ps();
    } else if (strcmp(cmd, "top") == 0) {
        cmd_top();
    } else if (strncmp(cmd, "kill ", 5) == 0) {
        int id = atoi(cmd + 5);
        cmd_kill(id);
    } else if (strncmp(cmd, "exec ", 5) == 0) {
        int id = atoi(cmd + 5);
        cmd_exec(id);
    } else if (strncmp(cmd, "run", 3) == 0) {
        const char *arg = cmd + 3;
        while (*arg == ' ') arg++;
        cmd_run(arg);
    } else if (strcmp(cmd, "info") == 0) {
        cmd_info();
    } else if (strcmp(cmd, "reboot") == 0) {
        cmd_reboot();
    } else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
        Serial.println("Type 'help' for available commands.");
    }
}

// シリアルから 1 行読み取る（本書 8.4「シェルとの統合」）
static void read_line(char *buf, int size)
{
    int pos = 0;
    memset(buf, 0, size);

    while (1) {
        if (Serial.available()) {
            char ch = Serial.read();

            if (ch == '\r' || ch == '\n') {
                Serial.println();
                return;
            } else if (ch == '\b' || ch == 127) {
                if (pos > 0) {
                    pos--;
                    buf[pos] = '\0';
                    Serial.print("\b \b");
                }
            } else if (pos < size - 1) {
                buf[pos++] = ch;
                Serial.print(ch);
            }
        }
        // top の表示中は、マトリクスが測り直すたびに描き直す（入力途中の文字も残す）
        if (top_on && display_get_sample_count() != top_drawn) {
            top_drawn = display_get_sample_count();
            top_draw(buf);
        }
        os_sleep(10);
    }
}

// run コマンド: 引数があれば 1 行スクリプト、無ければ複数行の入力モード
static void cmd_run(const char *arg)
{
    static char script[MAX_SCRIPT_LINES * MAX_LINE_LENGTH];
    static char line[MAX_LINE_LENGTH];

    if (arg && *arg) {
        interp_load(arg);
        interp_run();
        return;
    }

    Serial.println("Enter script (empty line to execute):");
    script[0] = '\0';

    while (1) {
        Serial.print(". ");
        read_line(line, MAX_LINE_LENGTH);
        if (line[0] == '\0') break;

        if (strlen(script) + strlen(line) + 2 >= sizeof(script)) {
            Serial.println("Error: script too long");
            return;
        }
        strcat(script, line);
        strcat(script, "\n");
    }

    if (script[0] != '\0') {
        interp_load(script);
        interp_run();
    }
}

void shell_task(void)
{
    Serial.println();
    Serial.println("Mini OS Shell");
    Serial.println("Type 'help' for commands.");

    interp_init();

    while (1) {
        // 保留中のフォルト情報をタスク文脈で表示する (第6章 6.3)
        fault_print_pending();

        if (!top_on) Serial.print("> ");
        read_line(cmd_buf, CMD_BUF_SIZE);
        shell_process_command(cmd_buf);
    }
}
