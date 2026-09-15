/**
 * @file    srv_boot_ctrl.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-15
 * @brief   升级控制服务实现（E1_MASTER_POWER_CTU → E1_CTU_BOOT）
 *
 * App 运行于 A 槽，收到 0x06 后不跳转，而是在 App 内直接接收固件写入 B 槽，
 * 保证下载期间电源正常输出；END 通过后仅写 metadata{flag=2} 并退出会话，
 * 继续以当前固件运行，下次重新上电由 Boot 续提交（promote B→A）后生效。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_boot_ctrl.h"

#include <stddef.h>

#include "boot_session.h"
#include "log.h"

/* App 镜像签名 ---------------------------------------------------------------*/

/** App 镜像签名：链接器固定放在 FLASH+0x200（.app_sig 段），
 *  供 Boot（BOOT_SKIP_APP_VERIFY 调试模式）在不看 metadata 时判定有效固件 */
__attribute__((section(".app_sig"), used))
const uint32_t s_app_image_sig = 0x41505031U;

/* Private constants ---------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_BOOT_CTRL_LOG_ENABLE 1

#if SRV_BOOT_CTRL_LOG_ENABLE
#define SRV_BOOT_CTRL_LOG_E(...) LOG_E("srv_boot_ctrl", __VA_ARGS__)
#define SRV_BOOT_CTRL_LOG_W(...) LOG_W("srv_boot_ctrl", __VA_ARGS__)
#define SRV_BOOT_CTRL_LOG_I(...) LOG_I("srv_boot_ctrl", __VA_ARGS__)
#define SRV_BOOT_CTRL_LOG_D(...) LOG_D("srv_boot_ctrl", __VA_ARGS__)
#else
#define SRV_BOOT_CTRL_LOG_E(...) ((void)0)
#define SRV_BOOT_CTRL_LOG_W(...) ((void)0)
#define SRV_BOOT_CTRL_LOG_I(...) ((void)0)
#define SRV_BOOT_CTRL_LOG_D(...) ((void)0)
#endif

/* Private variables ---------------------------------------------------------*/

static srv_boot_ctrl_config_t s_cfg;
static boot_flash_context_t s_flash_ctx; /**< 全会话/信息查询共用（单一 ring_storage 实例） */
static boot_session_t s_session;
static boot_session_config_t s_session_cfg;

static bool s_active;         /**< 升级会话进行中 */

/* Private function prototypes -----------------------------------------------*/

static void boot_commit(void* user, uint32_t size, uint32_t checksum);
static void boot_abort(void* user);

/* Exported functions --------------------------------------------------------*/

void srv_boot_ctrl_init(const srv_boot_ctrl_config_t* cfg)
{
    if (!cfg || !cfg->tx) {
        return;
    }
    s_cfg = *cfg;
    s_flash_ctx.initialized = false;
    s_active = false;
}

bool srv_boot_ctrl_enter_upgrade(void)
{
    boot_metadata_t meta;

    if (!s_cfg.tx) {
        return false;
    }

    /* 只读加载 metadata 缓存（不累加启动次数、不写 Flash） */
    if (boot_flash_peek_metadata(&s_flash_ctx, &meta) != BOOT_FLASH_OK) {
        SRV_BOOT_CTRL_LOG_E("metadata 加载失败，无法进入升级");
        return false;
    }

    s_session_cfg.flash = &s_flash_ctx;
    s_session_cfg.target = BOOT_PARTITION_B;
    s_session_cfg.tx = s_cfg.tx;
    s_session_cfg.on_commit = boot_commit;
    s_session_cfg.on_abort = boot_abort;
    s_session_cfg.on_id = NULL;
    s_session_cfg.my_id = s_cfg.dev_id;
    s_session_cfg.user = NULL;
    boot_session_init(&s_session, &s_session_cfg);

    s_active = true;
    SRV_BOOT_CTRL_LOG_I("进入 App 内升级会话（ID=0x%02X，暂存 B 槽）",
        (unsigned)s_cfg.dev_id);
    return true;
}

bool srv_boot_ctrl_is_upgrade_active(void)
{
    return s_active;
}

void srv_boot_ctrl_feed(const uint8_t* data, uint32_t len)
{
    if (!s_active || !data) {
        return;
    }
    for (uint32_t i = 0U; i < len; i++) {
        boot_session_feed(&s_session, data[i]);
    }
}

void srv_boot_ctrl_abort_session(void)
{
    if (!s_active) {
        return;
    }
    boot_abort(NULL);
}

bool srv_boot_ctrl_peek_metadata(boot_metadata_t* metadata)
{
    if (!metadata) {
        return false;
    }
    return boot_flash_peek_metadata(&s_flash_ctx, metadata) == BOOT_FLASH_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 提交回调：写 metadata{flag=2,size,sum,ID} 后退出会话，不自动复位
 * @note  固件已写入 B 槽、电源保持输出；下次上电 Boot 见 flag==2 提升 B→A
 *        并复位运行新固件（落实“升级期间不跳转，重新上电才生效”）。
 */
static void boot_commit(void* user, uint32_t size, uint32_t checksum)
{
    boot_metadata_t meta;
    (void)user;

    if (boot_flash_peek_metadata(&s_flash_ctx, &meta) != BOOT_FLASH_OK) {
        SRV_BOOT_CTRL_LOG_E("提交：metadata 重读失败");
        return;
    }

    meta.upgrade_flag = 2U;
    meta.boot_partition = (uint8_t)BOOT_PARTITION_A;
    meta.version = (uint16_t)(meta.version + 1U);
    meta.fw_size = size;
    meta.fw_checksum = checksum;
    meta.reserved = (meta.reserved & 0xFFFFFF00U) | (uint32_t)s_cfg.dev_id;

    if (boot_flash_write_metadata(&s_flash_ctx, &meta) != BOOT_FLASH_OK) {
        SRV_BOOT_CTRL_LOG_E("提交：metadata 写入失败");
        return;
    }

    s_active = false; /* 退出会话，App 继续以当前固件正常运行（不断电） */
    SRV_BOOT_CTRL_LOG_I("下载完成 %lu/0x%08lX，已写入 B 槽；重新上电后由 Boot 提升生效",
        (unsigned long)size, (unsigned long)checksum);
}

/** @brief 中止回调：退出会话恢复正常 App 运行（不复位） */
static void boot_abort(void* user)
{
    (void)user;
    s_active = false;
    SRV_BOOT_CTRL_LOG_W("升级会话中止，恢复 App 正常运行");
}
