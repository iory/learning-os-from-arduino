// os_syscall_user.h
//
// ユーザー側のラッパ。svc 命令の即値はコンパイル時定数でなければならないため、
// static inline にして呼び出し元で定数畳み込みされることを前提にしている。

#pragma once

#include <stdint.h>
#include "os_syscall.h"

static inline uint32_t _svc_call(uint8_t num, uint32_t a0,
                                 uint32_t a1, uint32_t a2)
{
    register uint32_t r0 __asm("r0") = a0;
    register uint32_t r1 __asm("r1") = a1;
    register uint32_t r2 __asm("r2") = a2;
    uint32_t ret;

    __asm volatile (
        "svc %[n]"
        : "=r" (r0)
        : [n] "i" (num), "r" (r0), "r" (r1), "r" (r2)
        : "memory"
    );
    ret = r0;
    return ret;
}

// ★ 引数がインライン即値である必要があるため、番号ごとにマクロ展開
#define SYSCALL0(n)         _svc_call((n), 0, 0, 0)
#define SYSCALL1(n, a)      _svc_call((n), (uint32_t)(a), 0, 0)
#define SYSCALL2(n, a, b)   _svc_call((n), (uint32_t)(a), (uint32_t)(b), 0)

static inline void user_putchar(char c)   { SYSCALL1(SYS_putchar, c); }
static inline void user_yield(void)       { SYSCALL0(SYS_yield); }
static inline void user_sleep(uint32_t m) { SYSCALL1(SYS_sleep,  m); }
static inline void user_exit(int code)    { SYSCALL1(SYS_exit,   code); }
static inline int  user_getpid(void)      { return (int)SYSCALL0(SYS_getpid); }
