// os_heap.cpp
//
// 本書 応用編 第3章の実装。ブロックヘッダ + 単方向フリーリスト + first-fit +
// 隣接結合。ヒープは明示的な配列として持つ（3.2.2 の「スタックとヒープの衝突」
// を避けるため）。

#include <Arduino.h>
#include <string.h>
#include "os_heap.h"

typedef struct block_header {
    size_t                size;   // データ部のバイト数 (ヘッダは含まない)
    int                   free;   // 1 なら空き、0 なら使用中
    struct block_header  *next;   // 次のブロックヘッダ
    uint32_t              magic;  // 二重 free / 破壊検出用
} block_header_t;

#define HEAP_MAGIC 0xA110CA7E    // "ALLOCATE" の語呂合わせ
#define ALIGN8(x)  (((x) + 7u) & ~7u)
#define HEAP_SIZE  4096

static uint8_t heap_area[HEAP_SIZE] __attribute__((aligned(8)));
static block_header_t *heap_head = NULL;

void heap_init(void)
{
    heap_head        = (block_header_t *)heap_area;
    heap_head->size  = HEAP_SIZE - sizeof(block_header_t);
    heap_head->free  = 1;
    heap_head->next  = NULL;
    heap_head->magic = HEAP_MAGIC;
}

void *my_malloc(size_t n)
{
    if (n == 0) return NULL;
    n = ALIGN8(n);                 // 8 の倍数に切り上げ

    block_header_t *b = heap_head;
    while (b != NULL) {
        if (b->magic != HEAP_MAGIC) {
            // ヘッダが破壊されている (バッファオーバーラン等)
            return NULL;
        }
        if (b->free && b->size >= n) {
            // 余りが十分なら分割 (split)
            if (b->size >= n + sizeof(block_header_t) + 8) {
                block_header_t *rest =
                    (block_header_t *)((uint8_t *)b + sizeof(block_header_t) + n);
                rest->size  = b->size - n - sizeof(block_header_t);
                rest->free  = 1;
                rest->next  = b->next;
                rest->magic = HEAP_MAGIC;
                b->size = n;
                b->next = rest;
            }
            b->free = 0;
            return (uint8_t *)b + sizeof(block_header_t);
        }
        b = b->next;
    }
    return NULL;                   // 空きなし
}

void my_free(void *p)
{
    if (p == NULL) return;

    block_header_t *b =
        (block_header_t *)((uint8_t *)p - sizeof(block_header_t));

    if (b->magic != HEAP_MAGIC) {
        // 不正なポインタ、もしくはヘッダ破壊
        Serial.println(F("[heap] bad pointer or corrupted header"));
        return;
    }
    if (b->free) {
        // すでに free 済み → 二重 free 検出
        Serial.println(F("[heap] double free detected"));
        return;
    }
    b->free = 1;

    // 後続ブロックと結合
    if (b->next != NULL && b->next->free) {
        b->size += sizeof(block_header_t) + b->next->size;
        b->next  = b->next->next;
    }

    // 前のブロックと結合 (単方向リストなので先頭から探す)
    block_header_t *prev = heap_head;
    while (prev != NULL && prev->next != b) prev = prev->next;
    if (prev != NULL && prev->free) {
        prev->size += sizeof(block_header_t) + b->size;
        prev->next  = b->next;
    }
}

void heap_dump(void)
{
    block_header_t *b = heap_head;
    int i = 0;
    size_t total_free = 0, total_used = 0;

    Serial.println(F("---- heap dump ----"));
    while (b != NULL) {
        char line[72];
        snprintf(line, sizeof(line), "[%02d] +%04u size=%5u %s",
                 i, (unsigned)((uint8_t *)b - heap_area),
                 (unsigned)b->size, b->free ? "FREE" : "USED");
        Serial.println(line);
        if (b->free) total_free += b->size;
        else         total_used += b->size;
        b = b->next;
        i++;
    }
    Serial.print(F("total free=")); Serial.print((unsigned)total_free);
    Serial.print(F(" used="));      Serial.println((unsigned)total_used);
}

size_t heap_free_total(void)
{
    size_t total = 0;
    for (block_header_t *b = heap_head; b != NULL; b = b->next) {
        if (b->free) total += b->size;
    }
    return total;
}
