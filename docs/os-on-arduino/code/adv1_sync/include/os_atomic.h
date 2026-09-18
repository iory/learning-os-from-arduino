// os_atomic.h
#pragma once
#include <stdint.h>
#include <Arduino.h>   // __LDREXW / __STREXW / __CLREX (CMSIS)

// アトミックなインクリメント (古い値を返す)
static inline uint32_t atomic_inc(volatile uint32_t *p)
{
    uint32_t old, status;
    do {
        old = __LDREXW(p);
        status = __STREXW(old + 1, p);
    } while (status != 0);  // 失敗したら再試行
    return old;
}

// アトミックな比較交換 (CAS)
static inline int atomic_cas(volatile uint32_t *p,
                             uint32_t expected,
                             uint32_t desired)
{
    uint32_t cur = __LDREXW(p);
    if (cur != expected) {
        __CLREX();   // 排他モニタを解除して終了
        return 0;
    }
    return __STREXW(desired, p) == 0;
}
