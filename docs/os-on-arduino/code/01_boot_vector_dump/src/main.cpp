// vector_dump.ino
// 第1章 演習1-2: ベクタテーブルの内容をダンプ

#include <Arduino.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }
    while (Serial.read() != 'S') { delay(1); }

    Serial.println("=== Vector Table Dump ===");
    Serial.println();

    // ベクタテーブルのアドレス（VTORから取得）
    uint32_t *vtor = (uint32_t *)SCB->VTOR;

    const char *names[] = {
        "Initial SP",
        "Reset",
        "NMI",
        "HardFault",
        "MemManage",
        "BusFault",
        "UsageFault",
        "Reserved",
        "Reserved",
        "Reserved",
        "Reserved",
        "SVCall",
        "DebugMon",
        "Reserved",
        "PendSV",
        "SysTick"
    };

    for (int i = 0; i < 16; i++) {
        Serial.print("[");
        if (i < 10) Serial.print(" ");
        Serial.print(i);
        Serial.print("] ");

        // アドレス
        char hex[16];
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)vtor[i]);
        Serial.print(hex);

        // 名前
        Serial.print("  ");
        Serial.println(names[i]);
    }

    Serial.println();
    Serial.println("Note: Handler addresses point to Flash (0x000xxxxx)");
    Serial.println("      Initial SP points to RAM top (0x200xxxxx)");
}

void loop()
{
}
