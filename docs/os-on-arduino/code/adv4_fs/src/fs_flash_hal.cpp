// fs_flash_hal.c
// RA4M1 の Data Flash (8KB) を FSP の R_FLASH_LP ドライバ経由で読み書きする。

#include <Arduino.h>
#include "fs_flash_hal.h"
#include "r_flash_lp.h"  // FSP のドライバ

static flash_lp_instance_ctrl_t g_ctrl;
static flash_cfg_t              g_cfg;

int flash_hal_init(void) {
    g_cfg.data_flash_bgo  = false;     // foreground (同期) 動作
    g_cfg.p_callback      = NULL;
    g_cfg.p_context       = NULL;
    g_cfg.ipl             = 0;
    g_cfg.irq             = FSP_INVALID_VECTOR;
    g_cfg.err_irq         = FSP_INVALID_VECTOR;   // 本文では省略されていた
    g_cfg.err_ipl         = 0;

    if (R_FLASH_LP_Open(&g_ctrl, &g_cfg) != FSP_SUCCESS) {
        return -1;
    }
    return 0;
}

int flash_hal_read(uint32_t offset, void *buf, size_t len) {
    // Data Flash はメモリマップされているので、直接ロードで読める
    const uint8_t *src = (const uint8_t *)(FLASH_DATA_BASE + offset);
    uint8_t *dst = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) dst[i] = src[i];
    return 0;
}

int flash_hal_prog(uint32_t offset, const void *buf, size_t len) {
    fsp_err_t err = R_FLASH_LP_Write(&g_ctrl,
                                     (uint32_t)buf,
                                     FLASH_DATA_BASE + offset,
                                     len);
    return (err == FSP_SUCCESS) ? 0 : -1;
}

int flash_hal_erase(uint32_t offset) {
    // offset は FLASH_SECTOR_SIZE の倍数であることが前提
    fsp_err_t err = R_FLASH_LP_Erase(&g_ctrl,
                                     FLASH_DATA_BASE + offset,
                                     1);  // 1 セクタ
    return (err == FSP_SUCCESS) ? 0 : -1;
}

int flash_hal_sync(void) {
    // foreground モードでは書き込みが完了してから戻るので何もしなくてよい
    return 0;
}
