// os_sync.cpp
//
// 本書 応用編 第1章の mutex / semaphore / condvar の実装。

#include <Arduino.h>
#include "os_sync.h"
#include "os_atomic.h"
#include "os_kernel.h"

// 同期オブジェクト待ちで寝る共通処理。
//
// os_sleep() と違ってタイムアウトを持たない。os_schedule() は
// 「wake_tick を過ぎた TASK_BLOCKED」を無条件に READY へ戻すので、
// wake_tick を最大値にしておかないと待った次の瞬間に起こされてしまう。
// 起こすのは mutex_unlock / sem_post / cv_signal の側の仕事。
static void block_current_task(uint8_t *queue, uint8_t *n_waiters)
{
    int me = os_get_current_task();
    queue[(*n_waiters)++] = (uint8_t)me;
    g_tasks[me].wake_tick = UINT32_MAX;
    g_tasks[me].state = TASK_BLOCKED;
}

// 待ち行列の先頭を 1 つ起こす
static void wake_one(uint8_t *queue, uint8_t *n_waiters)
{
    if (*n_waiters == 0) return;
    uint8_t next = queue[0];
    for (int i = 0; i < *n_waiters - 1; i++) {
        queue[i] = queue[i + 1];
    }
    (*n_waiters)--;
    g_tasks[next].wake_tick = 0;
    g_tasks[next].state = TASK_READY;
}

// ---------------- mutex ----------------

void mutex_init(mutex_t *m)
{
    m->locked = 0;
    m->owner = -1;
    m->n_waiters = 0;
}

void mutex_lock(mutex_t *m)
{
    while (1) {
        // LDREX/STREX で 0 → 1 のアトミック書き換えを試す
        if (atomic_cas(&m->locked, 0, 1)) {
            m->owner = (int8_t)os_get_current_task();
            return;
        }
        // 取れなかったら待ち行列に入って寝る
        __disable_irq();
        if (m->locked == 0) {
            // CAS 失敗から割り込み禁止までの隙間で unlock されていた。
            // このまま寝ると誰にも起こされない (lost wakeup) ので取り直す。
            __enable_irq();
            continue;
        }
        block_current_task(m->waiters, &m->n_waiters);
        __enable_irq();
        os_schedule();
    }
}

void mutex_unlock(mutex_t *m)
{
    __disable_irq();
    m->locked = 0;
    m->owner = -1;
    wake_one(m->waiters, &m->n_waiters);
    __enable_irq();
    os_schedule();
}

// ---------------- counting semaphore ----------------

void sem_init(semaphore_t *s, int32_t initial)
{
    s->count = initial;
    s->n_waiters = 0;
}

void sem_wait(semaphore_t *s)
{
    __disable_irq();
    if (s->count > 0) {
        s->count--;
        __enable_irq();
        return;
    }
    block_current_task(s->waiters, &s->n_waiters);
    __enable_irq();
    os_schedule();
}

void sem_post(semaphore_t *s)
{
    __disable_irq();
    if (s->n_waiters > 0) {
        // 待っているタスクへ直接渡す（count は増やさない）。
        // 増やしてしまうと、起きたタスクが再び count を奪い合うことになる。
        wake_one(s->waiters, &s->n_waiters);
    } else {
        s->count++;
    }
    __enable_irq();
    os_schedule();
}

// ---------------- condition variable ----------------

void cv_init(condvar_t *cv)
{
    cv->n_waiters = 0;
}

void cv_wait(condvar_t *cv, mutex_t *m)
{
    __disable_irq();
    // 待ち行列に入れた "後" に mutex を解放するのが肝心。
    // 順番が逆だと、解放から待機までの隙間で signal を取り逃す。
    block_current_task(cv->waiters, &cv->n_waiters);
    m->locked = 0;
    m->owner = -1;
    wake_one(m->waiters, &m->n_waiters);
    __enable_irq();
    os_schedule();

    // 起きたらここに戻ってくる。mutex を取り直してから呼び出し元へ返る。
    mutex_lock(m);
}

void cv_signal(condvar_t *cv)
{
    __disable_irq();
    wake_one(cv->waiters, &cv->n_waiters);
    __enable_irq();
}

void cv_broadcast(condvar_t *cv)
{
    __disable_irq();
    while (cv->n_waiters > 0) {
        wake_one(cv->waiters, &cv->n_waiters);
    }
    __enable_irq();
}
