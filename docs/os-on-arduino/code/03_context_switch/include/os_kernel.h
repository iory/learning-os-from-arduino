/**
 * os_kernel.h - Mini OS Kernel Header
 * Chapter 3: Context Switch Implementation
 */

#ifndef OS_KERNEL_H
#define OS_KERNEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Task states
typedef enum {
    TASK_READY,       // Ready to run
    TASK_RUNNING,     // Currently running
    TASK_BLOCKED,     // Blocked (waiting for I/O, etc.)
    TASK_SUSPENDED,   // Suspended (by kill command)
    TASK_TERMINATED   // Terminated
} TaskState;

// Task Control Block (TCB)
typedef struct {
    uint32_t *sp;           // Current stack pointer
    uint32_t *stack_base;   // Stack base (high address)
    uint32_t stack_size;    // Stack size in bytes
    TaskState state;        // Task state
    uint8_t id;             // Task ID
    uint8_t priority;       // Priority (0 = highest)
    char name[16];          // Task name
    void (*entry)(void);    // Entry point
} TCB;

// System configuration
#define MAX_TASKS 8
#define DEFAULT_STACK_SIZE 512
// タスク用スタックをまとめて置く領域。os_create_task() は
// ここから要求されたぶんだけ切り出す。
// 既定より大きなスタックを要求するタスク（シェルなど）があるため、
// MAX_TASKS * DEFAULT_STACK_SIZE では足りない。余裕を持たせる。
#define STACK_POOL_SIZE (MAX_TASKS * DEFAULT_STACK_SIZE + 2048)
#define IDLE_STACK_SIZE 256

// Kernel API
void os_init(void);
int os_create_task(void (*entry)(void), const char *name, uint32_t stack_size);
void os_start(void);
void os_yield(void);

// Internal functions
void os_schedule(void);

// Global variables (for assembly access)
extern TCB g_tasks[MAX_TASKS];
extern volatile int g_task_count;
extern volatile int g_current_task;
extern volatile int g_next_task;
extern const uint32_t g_tcb_size;

#ifdef __cplusplus
}
#endif

#endif // OS_KERNEL_H
