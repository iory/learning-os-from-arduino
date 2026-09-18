/**
 * os_kernel.h - Mini OS Kernel Header
 * Chapter 6: Memory Protection with HardFault Handler
 */

#ifndef OS_KERNEL_H
#define OS_KERNEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_SUSPENDED,
    TASK_TERMINATED
} TaskState;

typedef struct {
    uint32_t *sp;
    uint32_t *stack_base;
    uint32_t stack_size;
    TaskState state;
    uint8_t id;
    uint8_t priority;
    char name[16];
    void (*entry)(void);
    uint32_t wake_tick;
    uint32_t cpu_ticks;
} TCB;

#define MAX_TASKS 8
#define DEFAULT_STACK_SIZE 512
// タスク用スタックをまとめて置く領域。os_create_task() は
// ここから要求されたぶんだけ切り出す。
// 既定より大きなスタックを要求するタスク（シェルなど）があるため、
// MAX_TASKS * DEFAULT_STACK_SIZE では足りない。余裕を持たせる。
#define STACK_POOL_SIZE (MAX_TASKS * DEFAULT_STACK_SIZE + 2048)

// スタックを埋めておく目印。どこまで書き換わったかで使用量が分かる。
#define STACK_FILL_PATTERN 0xA5A5A5A5u
#define IDLE_STACK_SIZE 256
#define TIME_SLICE_MS 10

void os_init(void);
int os_create_task(void (*entry)(void), const char *name, uint32_t stack_size);
void os_start(void);
void os_yield(void);
void os_sleep(uint32_t ms);

void os_suspend_task(int id);
void os_resume_task(int id);
void os_terminate_task(int id);

int os_get_task_count(void);
const char* os_get_task_name(int id);
TaskState os_get_task_state(int id);
int os_get_current_task(void);
int os_get_cpu_usage(int task_id);
uint32_t os_get_task_cpu_ticks(int task_id);
uint32_t os_get_tick(void);
uint32_t os_get_stack_unused(int id);   // 一度も使われていないスタックのバイト数

extern TCB g_tasks[MAX_TASKS];
extern volatile int g_task_count;
extern volatile int g_current_task;
extern volatile int g_next_task;
extern volatile uint32_t g_system_ticks;
extern const uint32_t g_tcb_size;

#ifdef __cplusplus
}
#endif

#endif
