/**
 * os_kernel.cpp - Mini OS Kernel Implementation
 * Chapter 4: Preemptive Scheduler with SysTick
 */

#include "os_kernel.h"
#include <Arduino.h>
#include <string.h>

// Global variables
TCB g_tasks[MAX_TASKS];
volatile int g_task_count = 0;
volatile int g_current_task = -1;
volatile int g_next_task = -1;
volatile uint32_t g_system_ticks = 0;
static volatile uint32_t g_tick_count = 0;

const uint32_t g_tcb_size = sizeof(TCB);

// Stack areas
static uint32_t g_stack_pool[STACK_POOL_SIZE / 4];
static uint32_t g_stack_used;   // 切り出し済みのワード数

// Idle task
static void idle_task(void)
{
    while (1) {
        __WFI();
    }
}

// Initialize stack for a task
// タスク関数が return したときの受け皿。例外フレームの LR スロットは
// ここを指す (第3章 3.4)。EXC_RETURN を置く場所ではない。
static void task_exit(void)
{
    g_tasks[g_current_task].state = TASK_TERMINATED;
    os_yield();
    while (1) { }  // ここには到達しない
}

static uint32_t* init_task_stack(uint32_t *stack_top, void (*entry)(void))
{
    uint32_t *sp = stack_top;

    // Hardware auto-saved registers
    *(--sp) = 0x01000000;        // xPSR: Thumb bit
    *(--sp) = (uint32_t)entry;   // PC
    *(--sp) = (uint32_t)task_exit;  // LR: タスクが return したときの戻り先
    *(--sp) = 0;                 // R12
    *(--sp) = 0;                 // R3
    *(--sp) = 0;                 // R2
    *(--sp) = 0;                 // R1
    *(--sp) = 0;                 // R0

    // Software saved registers
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
    memset(g_tasks, 0, sizeof(g_tasks));
    g_task_count = 0;
    g_current_task = -1;
    g_next_task = -1;
    g_system_ticks = 0;

    // Create idle task (ID=0)
    os_create_task(idle_task, "idle", IDLE_STACK_SIZE);
}

int os_create_task(void (*entry)(void), const char *name, uint32_t stack_size)
{
    if (g_task_count >= MAX_TASKS) {
        return -1;
    }

    int id = g_task_count;
    TCB *task = &g_tasks[id];

    task->id = id;
    task->state = TASK_READY;
    task->priority = id;
    task->entry = entry;
    task->wake_tick = 0;
    task->cpu_ticks = 0;
    strncpy(task->name, name, sizeof(task->name) - 1);

    if (stack_size == 0) {
        stack_size = DEFAULT_STACK_SIZE;
    }
    uint32_t words = (stack_size + 3) / 4;
    if (g_stack_used + words > STACK_POOL_SIZE / 4) {
        return -1;          // プールが尽きた。黙って縮めず、失敗として返す
    }
    g_stack_used += words;
    task->stack_size = stack_size;
    task->stack_base = &g_stack_pool[g_stack_used];   // スタックは上端から下へ伸びる
    task->sp = init_task_stack(task->stack_base, entry);

    g_task_count++;
    return id;
}

void os_schedule(void)
{
    // Guard: don't schedule before OS is initialized
    if (g_task_count == 0) return;

    // Wake up blocked tasks
    for (int i = 0; i < g_task_count; i++) {
        if (g_tasks[i].state == TASK_BLOCKED) {
            if (g_system_ticks >= g_tasks[i].wake_tick) {
                g_tasks[i].state = TASK_READY;
            }
        }
    }

    // Round-robin scheduling
    int next = g_current_task;
    do {
        next = (next + 1) % g_task_count;
        if (g_tasks[next].state == TASK_READY ||
            g_tasks[next].state == TASK_RUNNING) {
            break;
        }
    } while (next != g_current_task);

    if (next != g_current_task || g_current_task < 0) {
        if (g_current_task >= 0) {
            if (g_tasks[g_current_task].state == TASK_RUNNING) {
                g_tasks[g_current_task].state = TASK_READY;
            }
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

void os_sleep(uint32_t ms)
{
    if (g_current_task > 0) {  // Don't block idle task
        g_tasks[g_current_task].state = TASK_BLOCKED;
        g_tasks[g_current_task].wake_tick = g_system_ticks + ms;
        os_schedule();
    }
}

uint32_t os_get_tick(void)
{
    return g_system_ticks;
}

int os_get_task_count(void)
{
    return g_task_count;
}

const char* os_get_task_name(int id)
{
    if (id < 0 || id >= g_task_count) return "";
    return g_tasks[id].name;
}

TaskState os_get_task_state(int id)
{
    if (id < 0 || id >= g_task_count) return TASK_TERMINATED;
    return g_tasks[id].state;
}

int os_get_current_task(void)
{
    return g_current_task;
}

void os_start(void)
{
    // Set PendSV to lowest priority
    NVIC_SetPriority(PendSV_IRQn, 0xFF);

    // Configure SysTick for TIME_SLICE_MS interval
    SysTick_Config(SystemCoreClock / 1000 * TIME_SLICE_MS);

    g_current_task = -1;
    g_next_task = 0;

    // Do NOT switch CONTROL to PSP here — PSP is not yet initialized.
    // PendSV returns with EXC_RETURN=0xFFFFFFFD, which automatically
    // switches thread mode to PSP after loading the first task's context.
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

    while (1) { }
}

// SysTick Handler - called every TIME_SLICE_MS
extern "C" void SysTick_Handler(void)
{
    // 実際に経過した時間を、いま走っていたタスクの取り分として計上する。
    // os_schedule() 側で数えると os_yield() の回数まで混ざってしまう。
    if (g_current_task >= 0) {
        g_tasks[g_current_task].cpu_ticks += TIME_SLICE_MS;
    }
    g_system_ticks += TIME_SLICE_MS;
    os_schedule();
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
        "                                \n"
        "   LDR     R3, =g_tasks         \n"
        "   LDR     R4, =g_tcb_size      \n"
        "   LDR     R4, [R4]             \n"
        "   MUL     R2, R2, R4           \n"
        "   ADD     R3, R3, R2           \n"
        "   LDR     R0, [R3]             \n"
        "                                \n"
        "   LDMIA   R0!, {R4-R11}        \n"
        "   MSR     PSP, R0              \n"
        "                                \n"
        "   LDR     LR, =0xFFFFFFFD      \n"
        "   BX      LR                   \n"
    );
}

// CPU使用率を取得（パーセント）
// 直前にこの関数を呼んだ時点からの差分で計算する。
// ps のようにタスクごとにループで呼ぶため、基準時刻もタスクごとに持つ。
int os_get_cpu_usage(int task_id)
{
    if (task_id < 0 || task_id >= g_task_count) {
        return -1;
    }

    static uint32_t last_total[MAX_TASKS] = {0};
    static uint32_t last_task_ticks[MAX_TASKS] = {0};

    uint32_t total = g_system_ticks;
    uint32_t delta_total = total - last_total[task_id];

    if (delta_total == 0) {
        return 0;
    }

    uint32_t delta_task = g_tasks[task_id].cpu_ticks - last_task_ticks[task_id];

    last_total[task_id] = total;
    last_task_ticks[task_id] = g_tasks[task_id].cpu_ticks;

    return (delta_task * 100) / delta_total;
}
