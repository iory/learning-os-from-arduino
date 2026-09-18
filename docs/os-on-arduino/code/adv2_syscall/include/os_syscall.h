// os_syscall.h
//
// 本書 応用編 第2章「ユーザー／カーネルモードと SVC」のシステムコール番号と、
// カーネル側の実体。番号は svc 命令の即値としてそのまま埋め込まれる。

#pragma once

#include <stdint.h>

#define SYS_putchar  0
#define SYS_yield    1
#define SYS_sleep    2
#define SYS_exit     3
#define SYS_getpid   4
#define SYS_NR       5   // システムコール総数

#ifdef __cplusplus
extern "C" {
#endif

// カーネル側の実体（SVC_Handler から呼ばれる）
void sys_putchar(char c);
void sys_yield(void);
void sys_sleep(uint32_t ms);
void sys_exit(int code);
int  sys_getpid(void);

// SVC ハンドラの C 側ディスパッチャ
void svc_dispatch_c(uint32_t *sp);

#ifdef __cplusplus
}
#endif
