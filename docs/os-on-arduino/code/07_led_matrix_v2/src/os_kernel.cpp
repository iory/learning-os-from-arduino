/**
 * os_kernel.cpp - Mini OS Kernel Implementation
 * Chapter 6: Memory Protection
 */

#include "os_kernel.h"
#include <Arduino.h>
#include <string.h>

TCB g_tasks[MAX_TASKS];
volatile int g_task_count = 0;
volatile int g_current_task = -1;
volatile int g_next_task = -1;
volatile uint32_t g_system_ticks = 0;

const uint32_t g_tcb_size = sizeof(TCB);

static uint32_t g_stack_pool[STACK_POOL_SIZE / 4];
static uint32_t g_stack_used;   // 切り出し済みのワード数

static void idle_task(void) { while (1) __WFI(); }

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
    *(--sp) = 0x01000000;
    *(--sp) = (uint32_t)entry;
    *(--sp) = (uint32_t)task_exit;  // LR: タスクが return したときの戻り先
    *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0;
    // FPU を使ったタスクは例外フレームが拡張される（FP レジスタ込み）。
    // どちらのフレームで中断したかは EXC_RETURN の値に現れるため、
    // タスクごとに EXC_RETURN も保存する。初期値は「PSP・基本フレーム」。
    *(--sp) = 0xFFFFFFFD;
    *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0;
    *(--sp) = 0; *(--sp) = 0; *(--sp) = 0; *(--sp) = 0;
    return sp;
}

void os_init(void)
{
    memset(g_tasks, 0, sizeof(g_tasks));
    g_task_count = 0;
    g_current_task = -1;
    g_next_task = -1;
    g_system_ticks = 0;
    os_create_task(idle_task, "idle", IDLE_STACK_SIZE);
}

int os_create_task(void (*entry)(void), const char *name, uint32_t stack_size)
{
    if (g_task_count >= MAX_TASKS) return -1;
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
    for (int i = 0; i < g_task_count; i++) {
        if (g_tasks[i].state == TASK_BLOCKED && g_system_ticks >= g_tasks[i].wake_tick) {
            g_tasks[i].state = TASK_READY;
        }
    }

    int next = g_current_task;
    do {
        next = (next + 1) % g_task_count;
        if (g_tasks[next].state == TASK_READY || g_tasks[next].state == TASK_RUNNING) break;
    } while (next != g_current_task);

    if (next != g_current_task || g_current_task < 0) {
        if (g_current_task >= 0 && g_tasks[g_current_task].state == TASK_RUNNING) {
            g_tasks[g_current_task].state = TASK_READY;
        }
        g_tasks[next].state = TASK_RUNNING;
        g_next_task = next;
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    }
}

void os_yield(void) { os_schedule(); }
void os_sleep(uint32_t ms) {
    if (g_current_task > 0) {
        g_tasks[g_current_task].state = TASK_BLOCKED;
        g_tasks[g_current_task].wake_tick = g_system_ticks + ms;
        os_schedule();
    }
}
void os_suspend_task(int id) {
    if (id > 0 && id < g_task_count) {
        g_tasks[id].state = TASK_SUSPENDED;
        if (id == g_current_task) os_schedule();
    }
}
void os_resume_task(int id) {
    if (id > 0 && id < g_task_count && g_tasks[id].state == TASK_SUSPENDED) {
        g_tasks[id].state = TASK_READY;
    }
}
void os_terminate_task(int id) {
    if (id > 0 && id < g_task_count) {
        g_tasks[id].state = TASK_TERMINATED;
        if (id == g_current_task) os_schedule();
    }
}

uint32_t os_get_tick(void) { return g_system_ticks; }
int os_get_task_count(void) { return g_task_count; }
const char* os_get_task_name(int id) { return (id >= 0 && id < g_task_count) ? g_tasks[id].name : ""; }
TaskState os_get_task_state(int id) { return (id >= 0 && id < g_task_count) ? g_tasks[id].state : TASK_TERMINATED; }
int os_get_current_task(void) { return g_current_task; }

void os_start(void)
{
    NVIC_SetPriority(PendSV_IRQn, 0xFF);
    SysTick_Config(SystemCoreClock / 1000 * TIME_SLICE_MS);
    g_current_task = -1;
    g_next_task = 0;
    // Do NOT switch CONTROL to PSP here — PSP is not yet initialized.
    // PendSV returns with EXC_RETURN=0xFFFFFFFD, which automatically
    // switches thread mode to PSP after loading the first task's context.
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
    while (1) { }
}

extern "C" void SysTick_Handler(void) {
    // 実際に経過した時間を、いま走っていたタスクの取り分として計上する。
    // os_schedule() 側で数えると os_yield() の回数まで混ざってしまう。
    if (g_current_task >= 0) {
        g_tasks[g_current_task].cpu_ticks += TIME_SLICE_MS;
    }
    g_system_ticks += TIME_SLICE_MS;
    os_schedule();
}

extern "C" __attribute__((naked)) void PendSV_Handler(void)
{
    __asm volatile (
        "   MRS     R0, PSP              \n"
        "   LDR     R1, =g_current_task  \n"
        "   LDR     R2, [R1]             \n"
        "   CMP     R2, #0               \n"
        "   BLT     _skip_save           \n"
        // FPU を使って中断したタスク（EXC_RETURN の bit4 が 0）は、
        // ハードウェアが積まない S16-S31 もここで退避する
        "   TST     LR, #0x10            \n"
        "   IT      EQ                   \n"
        "   VSTMDBEQ R0!, {S16-S31}      \n"
        // R4-R11 と EXC_RETURN(LR) を保存する
        "   STMDB   R0!, {R4-R11, LR}    \n"
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
        "   LDMIA   R0!, {R4-R11, LR}    \n"
        // FPU を使って中断していたタスクなら S16-S31 も復元する
        "   TST     LR, #0x10            \n"
        "   IT      EQ                   \n"
        "   VLDMIAEQ R0!, {S16-S31}      \n"
        "   MSR     PSP, R0              \n"
        "   BX      LR                   \n"
    );
}

// CPU使用率を取得（パーセント）
// 直前にこの関数を呼んだ時点からの差分で計算する。
// ps のようにタスクごとにループで呼ぶため、基準時刻もタスクごとに持つ。
// CPU 使用カウンタの生値。os_get_cpu_usage() と違って状態を持たないので、
// 何度呼んでも他の呼び手の計測窓を壊さない (第4章 4.7)。
uint32_t os_get_task_cpu_ticks(int task_id)
{
    if (task_id < 0 || task_id >= g_task_count) {
        return 0;
    }
    return g_tasks[task_id].cpu_ticks;
}

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
