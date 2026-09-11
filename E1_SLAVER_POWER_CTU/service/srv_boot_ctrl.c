/**
 * @file    srv_boot_ctrl.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-07
 * @brief   升级跳转控制服务实现（E1_SLAVER_POWER_CTU → E1_CTU_BOOT）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_boot_ctrl.h"

#include "boot_flash.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 本机 485 设备 ID（写入 metadata.reserved，供 Boot 定向寻址应答） */
#define SRV_BOOT_DEV_ID (0x02U)

/* Private variables ---------------------------------------------------------*/

static boot_flash_context_t s_flash_ctx;
static boot_metadata_t s_meta;

/* Exported functions --------------------------------------------------------*/

bool srv_boot_ctrl_request_upgrade(void)
{
    if (boot_flash_init(&s_flash_ctx) != BOOT_FLASH_OK) {
        return false;
    }
    if (boot_flash_read_metadata(&s_flash_ctx, &s_meta) != BOOT_FLASH_OK) {
        return false;
    }

    s_meta.upgrade_flag = 1U;
    /* 记录本机设备 ID（低字节），Boot 据此只应答寻址到本机的帧 */
    s_meta.reserved = (s_meta.reserved & 0xFFFFFF00U) | (uint32_t)SRV_BOOT_DEV_ID;
    if (boot_flash_write_metadata(&s_flash_ctx, &s_meta) != BOOT_FLASH_OK) {
        return false;
    }
    return true;
}
