/**
 * os_kernel.h - Mini OS Kernel Header
 * Chapter 4: Preemptive Scheduler with SysTick
 */

#ifndef OS_KERNEL_H
#define OS_KERNEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Task states
typedef enum {
    TASK_READY,
    TASK_RUNNING,
    TASK_BLOCKED,
    TASK_SUSPENDED,
    TASK_TERMINATED
} TaskState;

// Task Control Block (TCB)
typedef struct {
    uint32_t *sp;
    uint32_t *stack_base;
    uint32_t stack_size;
    TaskState state;
    uint8_t id;
    uint8_t priority;
    char name[16];
    void (*entry)(void);
    uint32_t wake_tick;     // For os_sleep()
    uint32_t cpu_ticks;     // CPU usage counter
} TCB;

// System configuration
#define MAX_TASKS 8
#define DEFAULT_STACK_SIZE 1024
// タスク用スタックをまとめて置く領域。os_create_task() は
// ここから要求されたぶんだけ切り出す。
// 既定より大きなスタックを要求するタスク（シェルなど）があるため、
// MAX_TASKS * DEFAULT_STACK_SIZE では足りない。余裕を持たせる。
#define STACK_POOL_SIZE (MAX_TASKS * DEFAULT_STACK_SIZE + 2048)
#define IDLE_STACK_SIZE 256
#define TIME_SLICE_MS 10

// Kernel API
void os_init(void);
int os_create_task(void (*entry)(void), const char *name, uint32_t stack_size);
void os_start(void);
void os_yield(void);
void os_sleep(uint32_t ms);

// Task info API
int os_get_task_count(void);
const char* os_get_task_name(int id);
TaskState os_get_task_state(int id);
int os_get_current_task(void);
int os_get_cpu_usage(int task_id);

// Internal
void os_schedule(void);
uint32_t os_get_tick(void);

// Global variables
extern TCB g_tasks[MAX_TASKS];
extern volatile int g_task_count;
extern volatile int g_current_task;
extern volatile int g_next_task;
extern volatile uint32_t g_system_ticks;
extern const uint32_t g_tcb_size;

#ifdef __cplusplus
}
#endif

#endif // OS_KERNEL_H
