// fs.c

#include <Arduino.h>
#include "lfs.h"
#include "fs.h"
#include "fs_flash_hal.h"

static lfs_t        g_lfs;
static struct lfs_config g_cfg;

static uint8_t g_read_buf[16];
static uint8_t g_prog_buf[16];
static uint8_t g_lookahead_buf[16];

// --- LittleFS から呼ばれる callback ---

static int blk_read(const struct lfs_config *c, lfs_block_t block,
                    lfs_off_t off, void *buf, lfs_size_t size) {
    (void)c;
    return flash_hal_read(block * FLASH_SECTOR_SIZE + off, buf, size);
}

static int blk_prog(const struct lfs_config *c, lfs_block_t block,
                    lfs_off_t off, const void *buf, lfs_size_t size) {
    (void)c;
    return flash_hal_prog(block * FLASH_SECTOR_SIZE + off, buf, size);
}

static int blk_erase(const struct lfs_config *c, lfs_block_t block) {
    (void)c;
    return flash_hal_erase(block * FLASH_SECTOR_SIZE);
}

static int blk_sync(const struct lfs_config *c) {
    (void)c;
    return flash_hal_sync();
}

// --- 初期化 ---

int fs_init(void) {
    flash_hal_init();

    g_cfg.read  = blk_read;
    g_cfg.prog  = blk_prog;
    g_cfg.erase = blk_erase;
    g_cfg.sync  = blk_sync;

    g_cfg.read_size      = 4;
    g_cfg.prog_size      = 4;
    g_cfg.block_size     = FLASH_SECTOR_SIZE;        // 1 KB
    g_cfg.block_count    = FLASH_DATA_SIZE / FLASH_SECTOR_SIZE;  // 8
    g_cfg.cache_size     = 16;
    g_cfg.lookahead_size = 16;
    g_cfg.block_cycles   = 500;

    g_cfg.read_buffer      = g_read_buf;
    g_cfg.prog_buffer      = g_prog_buf;
    g_cfg.lookahead_buffer = g_lookahead_buf;

    // 既存 FS を mount。失敗したら新規 format してから再 mount
    int err = lfs_mount(&g_lfs, &g_cfg);
    if (err) {
        lfs_format(&g_lfs, &g_cfg);
        err = lfs_mount(&g_lfs, &g_cfg);
    }
    return err;
}

lfs_t *fs_get(void) { return &g_lfs; }
