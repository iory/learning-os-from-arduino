/**
 * os_fault.cpp - Fault Handler Implementation
 * Chapter 6: Memory Protection
 */

#include "os_fault.h"
#include "os_kernel.h"
#include <Arduino.h>

static volatile int g_fault_occurred = 0;
static volatile const char* g_fault_reason = "";
static volatile int g_faulted_task = -1;

// Forward declarations for vector table override
extern "C" __attribute__((naked)) void HardFault_Handler(void);
extern "C" void PendSV_Handler(void);
extern "C" void SysTick_Handler(void);

void fault_init(void)
{
    // Enable divide by zero trap
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

    // Do NOT enable separate UsageFault/BusFault/MemFault handlers.
    // By leaving them disabled, all faults escalate to HardFault,
    // which we handle via the VTOR override below.

    // The Arduino framework copies the vector table to RAM (VTOR points to RAM).
    // We must overwrite the HardFault entry in the RAM vector table directly.
    // Also override PendSV and SysTick entries for the OS scheduler.
    uint32_t *vtor = (uint32_t *)SCB->VTOR;
    vtor[3] = (uint32_t)HardFault_Handler;   // Vector #3 = HardFault
    vtor[14] = (uint32_t)PendSV_Handler;     // Vector #14 = PendSV
    vtor[15] = (uint32_t)SysTick_Handler;    // Vector #15 = SysTick

    g_fault_occurred = 0;
    g_fault_reason = "";
}

const char* fault_get_reason(void)
{
    return (const char*)g_fault_reason;
}

int fault_occurred(void)
{
    return g_fault_occurred;
}

void fault_clear(void)
{
    g_fault_occurred = 0;
    g_fault_reason = "";
}

static void analyze_fault(void)
{
    uint32_t cfsr = SCB->CFSR;

    // Check Usage Fault Status Register (UFSR)
    if (cfsr & SCB_CFSR_DIVBYZERO_Msk) {
        g_fault_reason = "Divide by zero";
    } else if (cfsr & SCB_CFSR_UNALIGNED_Msk) {
        g_fault_reason = "Unaligned access";
    } else if (cfsr & SCB_CFSR_UNDEFINSTR_Msk) {
        g_fault_reason = "Undefined instruction";
    } else if (cfsr & SCB_CFSR_INVSTATE_Msk) {
        g_fault_reason = "Invalid state (Thumb bit)";
    } else if (cfsr & SCB_CFSR_INVPC_Msk) {
        g_fault_reason = "Invalid PC";
    }
    // Check Bus Fault Status Register (BFSR)
    else if (cfsr & SCB_CFSR_PRECISERR_Msk) {
        g_fault_reason = "Precise data bus error";
    } else if (cfsr & SCB_CFSR_IMPRECISERR_Msk) {
        g_fault_reason = "Imprecise data bus error";
    } else if (cfsr & SCB_CFSR_IBUSERR_Msk) {
        g_fault_reason = "Instruction bus error";
    }
    // Check Memory Management Fault Status Register (MMFSR)
    else if (cfsr & SCB_CFSR_DACCVIOL_Msk) {
        g_fault_reason = "Data access violation";
    } else if (cfsr & SCB_CFSR_IACCVIOL_Msk) {
        g_fault_reason = "Instruction access violation";
    }
    else {
        g_fault_reason = "Unknown fault";
    }

    // Clear fault flags
    SCB->CFSR = cfsr;
}

// Safe landing pad for terminated tasks after fault recovery
extern "C" void _fault_task_sink(void)
{
    while (1) {
        __WFI();
    }
}

// Flag to defer fault message printing to a safe context
static volatile int g_fault_pending_print = 0;
static volatile uint32_t g_fault_pc = 0;

extern "C" void HardFault_Handler_C(FaultFrame *frame)
{
    g_fault_occurred = 1;
    g_faulted_task = g_current_task;
    g_fault_pc = frame->pc;

    analyze_fault();

    // Terminate the faulted task and redirect PC to safe sink
    if (g_faulted_task > 0) {
        os_terminate_task(g_faulted_task);

        // Rewrite the stacked PC to a safe infinite loop so that
        // returning from the fault doesn't re-execute the faulting
        // instruction. The SysTick scheduler will switch away from
        // this terminated task on the next tick.
        frame->pc = (uint32_t)_fault_task_sink;
        frame->xpsr = 0x01000000;  // Thumb bit set

        // Defer printing to next safe context
        g_fault_pending_print = 1;
        return;
    }

    // If idle task or no task crashed, halt
    while (1) { }
}

// Call this from a task (e.g., shell) to print deferred fault info
void fault_print_pending(void)
{
    if (!g_fault_pending_print) return;
    g_fault_pending_print = 0;

    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!!!       FAULT DETECTED      !!!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println();
    Serial.print("Task ");
    Serial.print(g_faulted_task);
    Serial.print(" (");
    Serial.print(os_get_task_name(g_faulted_task));
    Serial.println(") crashed!");
    Serial.print("Reason: ");
    Serial.println((const char*)g_fault_reason);
    Serial.print("PC at fault: 0x");
    Serial.println(g_fault_pc, HEX);
    Serial.println("Task terminated. Other tasks continue.");
    Serial.println();
}

extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile (
        "   TST     LR, #4              \n"  // Check which stack was used
        "   ITE     EQ                  \n"
        "   MRSEQ   R0, MSP             \n"  // MSP if LR bit 2 is 0
        "   MRSNE   R0, PSP             \n"  // PSP if LR bit 2 is 1
        "   PUSH    {LR}                \n"  // Save EXC_RETURN
        "   BL      HardFault_Handler_C \n"  // Call C handler (preserves LR on stack)
        "   POP     {LR}                \n"  // Restore EXC_RETURN
        "   BX      LR                  \n"  // Return from exception properly
    );
}
