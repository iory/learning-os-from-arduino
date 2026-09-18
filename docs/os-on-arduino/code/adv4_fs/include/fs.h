// fs.h
//
// 本書 応用編 第4章。LittleFS を Data Flash の上にマウントする薄いラッパ。

#pragma once

#include "lfs.h"

#ifdef __cplusplus
extern "C" {
#endif

int    fs_init(void);   // mount。失敗したら format してから再 mount
lfs_t *fs_get(void);

#ifdef __cplusplus
}
#endif
