/**
 * os_kernel.cpp - Mini OS Kernel Implementation
 * Chapter 3: Context Switch Implementation
 */

#include "os_kernel.h"
#include <Arduino.h>
#include <string.h>

// Global variables
TCB g_tasks[MAX_TASKS];
volatile int g_task_count = 0;
volatile int g_current_task = -1;
volatile int g_next_task = -1;

// TCB size for PendSV assembly (compiler may use -fshort-enums, changing sizeof)
const uint32_t g_tcb_size = sizeof(TCB);

// Stack areas (statically allocated)
// All tasks use DEFAULT_STACK_SIZE for simplicity
static uint32_t g_stack_pool[STACK_POOL_SIZE / 4];
static uint32_t g_stack_used;   // 切り出し済みのワード数

// Idle task (does nothing, waits for interrupt)
static void idle_task(void)
{
    while (1) {
        __WFI();  // Wait For Interrupt: sleep CPU until interrupt
    }
}

// Task exit handler - called if a task returns from its entry function
static void task_exit(void)
{
    // Mark current task as terminated and yield
    g_tasks[g_current_task].state = TASK_TERMINATED;
    os_yield();
    while (1) { }  // Should never reach here
}

// Initialize stack for a task so it can "return" from context switch
static uint32_t* init_task_stack(uint32_t *stack_top, void (*entry)(void))
{
    // Stack grows from high to low address
    uint32_t *sp = stack_top;

    // Hardware auto-restores these on exception return (in order: R0,R1,R2,R3,R12,LR,PC,xPSR)
    // Note: Stack is filled from high to low, so we push in reverse order
    *(--sp) = 0x01000000;           // xPSR: Thumb bit set (bit 24)
    *(--sp) = (uint32_t)entry;      // PC: entry point
    *(--sp) = (uint32_t)task_exit;  // LR: return address if task exits
    *(--sp) = 0;                    // R12
    *(--sp) = 0;                    // R3
    *(--sp) = 0;                    // R2
    *(--sp) = 0;                    // R1
    *(--sp) = 0;                    // R0

    // Software saves/restores these (R4-R11)
    *(--sp) = 0;  // R11
    *(--sp) = 0;  // R10
    *(--sp) = 0;  // R9
    *(--sp) = 0;  // R8
    *(--sp) = 0;  // R7
    *(--sp) = 0;  // R6
    *(--sp) = 0;  // R5
    *(--sp) = 0;  // R4

    return sp;
}

void os_init(void)
{
    // Initialize task array
    memset(g_tasks, 0, sizeof(g_tasks));
    g_task_count = 0;
    g_current_task = -1;
    g_next_task = -1;

    // Create idle task (ID=0)
    os_create_task(idle_task, "idle", IDLE_STACK_SIZE);
}

int os_create_task(void (*entry)(void), const char *name, uint32_t stack_size)
{
    if (g_task_count >= MAX_TASKS) {
        return -1;  // Task limit reached
    }

    int id = g_task_count;
    TCB *task = &g_tasks[id];

    // Initialize TCB
    task->id = id;
    task->state = TASK_READY;
    task->priority = id;  // Simple: use ID as priority
    task->entry = entry;
    strncpy(task->name, name, sizeof(task->name) - 1);

    // Set up stack (all tasks use DEFAULT_STACK_SIZE)
    if (stack_size == 0) {
        stack_size = DEFAULT_STACK_SIZE;
    }
    uint32_t words = (stack_size + 3) / 4;
    if (g_stack_used + words > STACK_POOL_SIZE / 4) {
        return -1;          // プールが尽きた。黙って縮めず、失敗として返す
    }
    g_stack_used += words;
    task->stack_size = stack_size;
    task->stack_base = &g_stack_pool[g_stack_used];   // スタックは上端から下へ伸びる  // High address (end of array)
    task->sp = init_task_stack(task->stack_base, entry);

    g_task_count++;
    return id;
}

void os_schedule(void)
{
    // Simple round-robin scheduling (skip idle task 0 for now)
    int next = g_current_task;

    do {
        next = (next + 1) % g_task_count;
        // Skip idle task (task 0) for debugging
        if (next == 0 && g_task_count > 1) {
            next = 1;
        }
        if (g_tasks[next].state == TASK_READY ||
            g_tasks[next].state == TASK_RUNNING) {
            break;
        }
    } while (next != g_current_task);

    if (next != g_current_task) {
        if (g_current_task >= 0) {
            g_tasks[g_current_task].state = TASK_READY;
        }
        g_tasks[next].state = TASK_RUNNING;
        g_next_task = next;

        // Trigger PendSV
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    }
}

void os_yield(void)
{
    os_schedule();
}

void os_start(void)
{
    // Set PendSV to lowest priority (so it runs after other interrupts)
    NVIC_SetPriority(PendSV_IRQn, 0xFF);

    // g_current_task = -1 means no task is running yet.
    // PendSV will skip saving context and just load the first task.
    g_current_task = -1;
    g_next_task = (g_task_count > 1) ? 1 : 0;

    // Trigger PendSV to load the first task's context
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

    // Wait here - PendSV will switch to the first task
    while (1) { }
}

// PendSV Handler - Context Switch
extern "C" __attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "   MRS     R0, PSP              \n"
        "   LDR     R1, =g_current_task  \n"
        "   LDR     R2, [R1]             \n"
        "   CMP     R2, #0               \n"
        "   BLT     _skip_save           \n"
        // Save R4-R11 only when there is a valid current task
        "   STMDB   R0!, {R4-R11}        \n"
        "   LDR     R3, =g_tasks         \n"
        "   LDR     R4, =g_tcb_size      \n"
        "   LDR     R4, [R4]             \n"
        "   MUL     R2, R2, R4           \n"
        "   ADD     R3, R3, R2           \n"
        "   STR     R0, [R3]             \n"
        "_skip_save:                     \n"
        "   LDR     R1, =g_next_task     \n"
        "   LDR     R2, [R1]             \n"
        "   LDR     R1, =g_current_task  \n"
        "   STR     R2, [R1]             \n"
        "   LDR     R3, =g_tasks         \n"
        "   LDR     R4, =g_tcb_size      \n"
        "   LDR     R4, [R4]             \n"
        "   MUL     R2, R2, R4           \n"
        "   ADD     R3, R3, R2           \n"
        "   LDR     R0, [R3]             \n"
        "   LDMIA   R0!, {R4-R11}        \n"
        "   MSR     PSP, R0              \n"
        "   LDR     LR, =0xFFFFFFFD      \n"
        "   BX      LR                   \n"
    );
}
