/**
 * shell.cpp - Interactive Shell Implementation
 * Chapter 5: Interactive Shell
 */

#include "shell.h"
#include "os_kernel.h"
#include "os_fault.h"
#include <Arduino.h>
#include <string.h>

#define CMD_BUF_SIZE 64

static char cmd_buf[CMD_BUF_SIZE];
static int cmd_pos = 0;

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
    } else if (strncmp(cmd, "kill ", 5) == 0) {
        int id = atoi(cmd + 5);
        cmd_kill(id);
    } else if (strncmp(cmd, "exec ", 5) == 0) {
        int id = atoi(cmd + 5);
        cmd_exec(id);
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

void shell_task(void)
{
    Serial.println();
    Serial.println("Mini OS Shell");
    Serial.println("Type 'help' for commands.");

    while (1) {
        // 保留中のフォルト情報をタスク文脈で表示する (第6章 6.3)
        fault_print_pending();

        Serial.print("> ");

        cmd_pos = 0;
        memset(cmd_buf, 0, CMD_BUF_SIZE);

        while (1) {
            if (Serial.available()) {
                char ch = Serial.read();

                if (ch == '\r' || ch == '\n') {
                    Serial.println();
                    break;
                } else if (ch == '\b' || ch == 127) {
                    if (cmd_pos > 0) {
                        cmd_pos--;
                        cmd_buf[cmd_pos] = '\0';
                        Serial.print("\b \b");
                    }
                } else if (cmd_pos < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_pos++] = ch;
                    Serial.print(ch);
                }
            }
            os_sleep(10);
        }

        shell_process_command(cmd_buf);
    }
}
