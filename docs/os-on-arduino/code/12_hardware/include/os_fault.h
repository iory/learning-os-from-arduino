/**
 * os_fault.h - Fault Handler Header
 * Chapter 6: Memory Protection
 */

#ifndef OS_FAULT_H
#define OS_FAULT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Fault information structure
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} FaultFrame;

// Initialize fault handling
void fault_init(void);

// Get fault reason string
const char* fault_get_reason(void);

// Check if a fault occurred
int fault_occurred(void);

// Clear fault flag
void fault_clear(void);

// Print pending fault info (call from task context, not from ISR)
void fault_print_pending(void);

#ifdef __cplusplus
}
#endif

#endif
