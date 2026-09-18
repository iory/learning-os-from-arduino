// os_heap.h
//
// 本書 応用編 第3章「ヒープ自作」で作る freelist 型アロケータ。

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void  heap_init(void);
void *my_malloc(size_t n);
void  my_free(void *p);
void  heap_dump(void);          // 3.7 デバッグ機能
size_t heap_free_total(void);   // 空き合計（断片化の観察用）

#ifdef __cplusplus
}
#endif
