// os_svc.cpp
//
// 本書 応用編 第2章 2.5「SVC_Handler の実装」。
//
// SVC 番号は命令そのものに埋め込まれているので、ハンドラからは
// 「スタックに積まれた戻り先 PC の 2 バイト手前」を読んで取り出す。

#include <Arduino.h>
#include "os_syscall.h"
#include "os_kernel.h"

// ---- カーネル側のシステムコール実体 ----
//
// 本文では svc_dispatch_c から呼ぶ形だけを示しているので、中身はここで書く。
// いずれも「ユーザーが直接触れないもの」をカーネルが代行する形になっている。

void sys_putchar(char c)
{
    Serial.write(c);
}

void sys_yield(void)
{
    os_yield();
}

void sys_sleep(uint32_t ms)
{
    os_sleep(ms);
}

void sys_exit(int code)
{
    (void)code;
    os_terminate_task(os_get_current_task());
}

int sys_getpid(void)
{
    return os_get_current_task();
}

// ---- ディスパッチャ ----

// 引数 sp は SVC を発行したコードのスタックフレーム先頭を指す
extern "C" void svc_dispatch_c(uint32_t *sp)
{
    // sp[0]=R0, sp[1]=R1, sp[2]=R2, sp[3]=R3
    // sp[6]=元の PC (SVC の次の命令)
    uint8_t  *pc      = (uint8_t *)sp[6];
    uint8_t   svc_num = pc[-2];          // svc 命令の下位バイト = 番号

    uint32_t arg0 = sp[0];
    uint32_t ret  = 0;

    switch (svc_num) {
    case SYS_putchar:
        sys_putchar((char)arg0);
        break;
    case SYS_yield:
        sys_yield();
        break;
    case SYS_sleep:
        sys_sleep(arg0);  // ms 単位
        break;
    case SYS_exit:
        sys_exit((int)arg0);
        break;
    case SYS_getpid:
        ret = (uint32_t)sys_getpid();
        break;
    default:
        ret = (uint32_t)-1;  // ENOSYS 相当
        break;
    }

    // 戻り値は R0 (sp[0]) に書き戻す
    sp[0] = ret;
}

// ---- 例外ハンドラ本体 ----
//
// SVC が特権スレッドから来たか非特権スレッドから来たかで、引数が積まれている
// スタックが MSP か PSP かが変わる。EXC_RETURN の bit2 で見分ける。

extern "C" __attribute__((naked)) void SVC_Handler(void)
{
    __asm volatile (
        "TST   lr, #4            \n"  // EXC_RETURN bit2: 0=MSP, 1=PSP
        "ITE   EQ                \n"
        "MRSEQ r0, MSP           \n"  // bit2==0 なら MSP
        "MRSNE r0, PSP           \n"  // bit2==1 なら PSP
        "B     svc_dispatch_c    \n"  // R0 = スタックポインタ
    );
}
