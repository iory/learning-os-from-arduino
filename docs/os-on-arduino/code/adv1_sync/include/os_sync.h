// os_sync.h
//
// 本書 応用編 第1章「ロックと同期」で作る同期プリミティブ。
//   mutex_t     : 同時に 1 つのタスクしか入れない部屋（1.3）
//   semaphore_t : 同時に N 個まで通す（1.4）
//   condvar_t   : 条件が成り立つまで待つ（1.5）

#pragma once

#include <stdint.h>
#include "os_kernel.h"   // MAX_TASKS / TaskState / g_tasks

#ifdef __cplusplus
extern "C" {
#endif

// ---- mutex (相互排他ロック) ----
typedef struct {
    volatile uint32_t locked;     // 0=空き, 1=ロック中
    int8_t  owner;                // 取得中のタスク ID (-1 なら誰もいない)
    uint8_t waiters[MAX_TASKS];   // 待ち行列 (タスク ID をキュー)
    uint8_t n_waiters;
} mutex_t;

void mutex_init(mutex_t *m);
void mutex_lock(mutex_t *m);
void mutex_unlock(mutex_t *m);

// ---- counting semaphore ----
typedef struct {
    volatile int32_t count;
    uint8_t waiters[MAX_TASKS];
    uint8_t n_waiters;
} semaphore_t;

void sem_init(semaphore_t *s, int32_t initial);
void sem_wait(semaphore_t *s);   // P 操作 (down)
void sem_post(semaphore_t *s);   // V 操作 (up)

// ---- condition variable (条件変数) ----
typedef struct {
    uint8_t waiters[MAX_TASKS];
    uint8_t n_waiters;
} condvar_t;

void cv_init(condvar_t *cv);
void cv_wait(condvar_t *cv, mutex_t *m);  // m を一旦離して待つ → 起きたら m を取り直す
void cv_signal(condvar_t *cv);            // 1 人起こす
void cv_broadcast(condvar_t *cv);         // 全員起こす

#ifdef __cplusplus
}
#endif
