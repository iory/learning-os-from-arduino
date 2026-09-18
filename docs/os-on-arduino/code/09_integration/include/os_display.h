// os_display.h

#ifndef OS_DISPLAY_H
#define OS_DISPLAY_H

#include "os_kernel.h"
#include "Arduino_LED_Matrix.h"

// 表示を初期化
void display_init(void);

// 表示を更新（定期的に呼ぶ）
void display_update(void);

// 表示タスク
void display_task(void);

#endif // OS_DISPLAY_H
