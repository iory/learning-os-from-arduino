// fs_flash_hal.h

#ifndef FS_FLASH_HAL_H
#define FS_FLASH_HAL_H

#include <stdint.h>
#include <stddef.h>

#define FLASH_DATA_BASE   0x40100000UL  // Data Flash の先頭物理アドレス
#define FLASH_DATA_SIZE   (8 * 1024)    // 8 KB
#define FLASH_SECTOR_SIZE 1024          // 1 KB = Data Flash の消去ブロック = LittleFS の block_size

int  flash_hal_init(void);
int  flash_hal_read (uint32_t offset, void *buf, size_t len);
int  flash_hal_prog (uint32_t offset, const void *buf, size_t len);
int  flash_hal_erase(uint32_t offset);   // セクタ消去
int  flash_hal_sync (void);

#endif
