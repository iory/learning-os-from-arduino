/**
 * 応用編 第3章: ヒープ自作 — malloc / free を書く
 *
 * 本章の課題 3.9.1 をそのまま実行する。
 *
 *   heap_init() → a=malloc(100) → b=malloc(100) → c=malloc(100)
 *               → free(b) → free(a)
 *
 * 各ステップで heap_dump() を呼び、free(a) のあとに a と b が
 * ひとつの空きブロックへ結合されることを確かめる。
 * 最後に外部断片化（合計は足りるのに連続では取れない）を再現する。
 */

#include <Arduino.h>
#include "os_heap.h"

static void step(const char *label)
{
    Serial.println();
    Serial.print(F("### ")); Serial.println(label);
    heap_dump();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { }
    while (Serial.read() != 'S') { delay(1); }

    Serial.println(F("=== Advanced 3: my_malloc / my_free ==="));

    heap_init();
    step("heap_init()");

    void *a = my_malloc(100);  step("a = my_malloc(100)");
    void *b = my_malloc(100);  step("b = my_malloc(100)");
    void *c = my_malloc(100);  step("c = my_malloc(100)");

    my_free(b);                step("my_free(b)");
    my_free(a);                step("my_free(a)  <- a と b が結合されるはず");
    my_free(c);                step("my_free(c)  <- ヒープが 1 つの空きブロックに戻る");

    // 課題: ヒープを満杯にしてから 1 つおきに解放すると、
    // 空き合計は足りるのに連続では取れない（外部断片化）
    Serial.println();
    Serial.println(F("### 外部断片化の確認"));

    void *p[8];
    for (int i = 0; i < 8; i++) {
        p[i] = my_malloc(496);   // (ヘッダ 16 + 496) × 8 = 4096 で満杯
    }
    my_free(p[1]);
    my_free(p[3]);
    my_free(p[5]);               // 496 バイトの穴が 3 つ
    Serial.print(F("空き合計 = ")); Serial.println((unsigned)heap_free_total());

    void *d = my_malloc(600);
    Serial.print(F("my_malloc(600) -> ")); Serial.println(d ? "OK" : "NULL");
    Serial.println(F("空きの合計は足りていても、連続した領域が無ければ取れない"));

    my_free(p[2]);               // p[1]〜p[3] が 1520 バイトの穴に結合される
    void *e = my_malloc(600);
    Serial.print(F("my_free(p[2]) 後の my_malloc(600) -> ")); Serial.println(e ? "OK" : "NULL");

    // 3.7 デバッグ機能: 二重 free を検出する
    Serial.println();
    Serial.println(F("### 二重 free の検出"));
    my_free(p[0]);
    my_free(p[0]);   // ここで [heap] double free detected が出る

    Serial.println();
    Serial.println(F("done."));
}

void loop() { }
