/**
 * 応用編 第2章: ユーザー／カーネルモードと SVC
 *
 * タスクが Serial や os_sleep() を直接呼ばず、すべて svc 命令を経由して
 * カーネルに依頼する形にしたデモ。
 *
 * user_putchar() などのラッパは svc を発行するだけで、実際に Serial を
 * 触るのは SVC_Handler の先にいる sys_putchar()。ユーザーコードから
 * ペリフェラルへの直接アクセスが 1 箇所も無いことがポイント。
 *
 * なお本章 2.3 の「非特権モードへ落とす」（CONTROL.nPRIV=1）は、
 * PendSV でタスクごとに CONTROL を切り替える改造が要る。本書はそこを
 * 擬似コードで示すにとどめているため、このサンプルも特権スレッドのまま
 * 動かしている。SVC の仕組み自体は特権／非特権のどちらからでも同じ。
 */

#include <Arduino.h>
#include "os_kernel.h"
#include "os_syscall.h"
#include "os_syscall_user.h"

// システムコールだけで書いた文字列出力
static void user_print(const char *s)
{
    while (*s) user_putchar(*s++);
}

static void user_print_int(int v)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", v);
    user_print(buf);
}

// ユーザータスク A: 自分の PID を名乗って寝るだけ
static void task_a(void)
{
    while (1) {
        user_print("[A] pid=");
        user_print_int(user_getpid());
        user_print("\r\n");
        user_sleep(1000);
    }
}

// ユーザータスク B: yield を挟みながら数える
static void task_b(void)
{
    int n = 0;
    while (1) {
        user_print("[B] count=");
        user_print_int(n++);
        user_print("\r\n");
        user_yield();
        user_sleep(1500);
    }
}

// 未定義の SVC 番号を呼ぶと -1 (ENOSYS 相当) が返ることを確かめる
static void task_probe(void)
{
    user_sleep(2500);
    int r = (int)SYSCALL0(99);
    user_print("[probe] svc #99 -> ");
    user_print_int(r);
    user_print("  (-1 なら ENOSYS 相当が返っている)\r\n");
    user_exit(0);
    while (1) user_sleep(1000);   // ここへは戻らない
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }

    Serial.println(F("=== Advanced 2: SVC system calls ==="));
    Serial.println(F("以降の出力はすべて svc 経由（sys_putchar）で出ています"));

    os_init();

    // SVC の優先度を下げておく（重要）。
    // リセット直後の SVC は最高優先度 0 のため、SVC_Handler の中で
    // Serial.write() がバッファ空きを待つと、それを配る UART/USB 割り込み
    // （優先度はもっと低い）が永遠に走れず、システム全体が止まる。
    // 「ペリフェラル割り込みより低く、PendSV/SysTick(15) より高く」が正解。
    NVIC_SetPriority(SVCall_IRQn, 14);

    os_create_task(task_a,     "UserA", 512);
    os_create_task(task_b,     "UserB", 512);
    os_create_task(task_probe, "Probe", 512);
    os_start();
}

void loop() { }
