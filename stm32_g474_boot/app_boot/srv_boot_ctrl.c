/**
 * @file    srv_boot_ctrl.c
 * @brief   Boot 控制服务实现 — 自持 BOOT 分区 ring_storage 实例（G474 单分区）
 *
 * 与 stm32_g474_boot bootloader 共享同一 metadata 区域（0x0801C000，16KB）。
 * boot_metadata_t 为私有类型，字段布局与 bootloader 的 boot_flash.h 完全一致。
 *
 * ## 依赖
 * 本目录:  hal_flash, ring_storage, ring_storage_port_hal（已随附）
 * CMSIS:   NVIC_SystemReset()（通过 HAL 间接引入，无需额外配置）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_boot_ctrl.h"

#include "hal_flash.h"
#include "ring_storage.h"
#include "ring_storage_port_hal.h"

/* 模块日志开关 ----------------------------------------------------------------*/

#define SRV_BOOT_CTRL_LOG_ENABLE 0

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

/* Private types -------------------------------------------------------------*/

/** @brief Metadata 魔数 "BOOT"（与 g474_boot 一致） */
#define BOOT_METADATA_MAGIC (0x424F4F54U)

/**
 * @brief Boot 引导 Metadata（私有类型）
 * @note 字段布局与 public_layer/service/boot/boot_flash.h 的 boot_metadata_t
 *       完全一致，构成与 bootloader 的共享字节契约，不得增删字段/改变类型。
 */
typedef struct {
    uint32_t magic;            /**< 魔数，标识有效 metadata */
    uint8_t  boot_partition;   /**< 当前引导分区 */
    uint8_t  upgrade_flag;     /**< 升级标志：0=正常，1=升级中 */
    uint16_t version;          /**< 固件版本号 */
    uint32_t fw_size;          /**< 固件大小 */
    uint32_t fw_checksum;      /**< 固件校验和 */
    uint32_t reboot_counts;    /**< MCU 上电启动次数 */
    uint32_t reserved;         /**< 预留 */
} boot_metadata_t;

/* Private constants ---------------------------------------------------------*/

/** @brief Metadata 区起始地址（与 bootloader 的 BOOT_META_OFFSET 一致）
 *  BOOT(64K) + APP(48K) = 0x0001C000 */
#define SRV_BOOT_CTRL_START_ADDR  (0x0801C000U)
/** @brief Metadata 区大小（与 BOOT_FLASH_META_SIZE 一致） */
#define SRV_BOOT_CTRL_AREA_SIZE   (0x00004000U)  /* 16KB */

/** @brief 帧缓冲区大小 (RAM)：28B 开销 + "meta" KV(31B) ≈ 59B */
#define SRV_BOOT_CTRL_FRAME_BUF_SIZE (128U)

/* Private variables ---------------------------------------------------------*/

static ring_storage_context_t s_ctx;
static uint8_t s_frame_buf[SRV_BOOT_CTRL_FRAME_BUF_SIZE];
static boot_metadata_t s_boot_meta;     /**< "meta" KV 绑定的运行时缓存 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void boot_meta_init_defaults(void);

/* Exported functions --------------------------------------------------------*/

srv_boot_ctrl_error_t srv_boot_ctrl_init(void)
{
    /* 幂等 */
    if (s_initialized) {
        return SRV_BOOT_CTRL_OK;
    }

    /* Flash 硬件初始化 */
    if (hal_flash_init() != HAL_FLASH_OK) {
        SRV_BOOT_CTRL_LOG_E("hal_flash 初始化失败");
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 初始化 BOOT 分区 ring_storage（G474 均匀页，扇区大小取 hal_flash 上报值） */
    const ring_storage_config_t rs_cfg = {
        .port              = ring_storage_port_hal(),
        .start_addr        = SRV_BOOT_CTRL_START_ADDR,
        .area_size         = SRV_BOOT_CTRL_AREA_SIZE,
        .sector_size       = hal_flash_get_caps()->erase_size,
        .write_gran        = (ring_storage_write_gran_t)hal_flash_get_caps()->write_gran,
        .frame_buffer      = s_frame_buf,
        .frame_buffer_size = sizeof(s_frame_buf),
    };
    const ring_storage_error_t rs_err = ring_storage_init(&s_ctx, &rs_cfg);
    if (rs_err != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_E("BOOT 分区 ring_storage 初始化失败 (err=%d)", (int)rs_err);
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 注册 "meta" KV */
    if (ring_storage_register(&s_ctx, "meta",
            &s_boot_meta, sizeof(s_boot_meta)) != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_E("注册 meta KV 失败");
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 加载 metadata；首次使用初始化默认字段 */
    ring_storage_error_t err = ring_storage_load(&s_ctx);
    if (err == RING_STORAGE_ERROR_NO_VALID_FRAME) {
        SRV_BOOT_CTRL_LOG_I("BOOT 分区首次使用，初始化 metadata");
        boot_meta_init_defaults();
        err = RING_STORAGE_OK;
    }
    if (err != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_W("BOOT 分区加载失败 (err=%d)", (int)err);
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 上电计数 +1 并持久化（与 bootloader 行为一致） */
    s_boot_meta.reboot_counts++;
    err = ring_storage_save(&s_ctx);
    if (err != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_W("BOOT 分区保存失败 (err=%d)", (int)err);
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    s_initialized = true;
    SRV_BOOT_CTRL_LOG_I("初始化完成 (reboot=%u, ver=%u, fw_size=%u, checksum=0x%08lX)",
        (unsigned)s_boot_meta.reboot_counts,
        (unsigned)s_boot_meta.version,
        (unsigned)s_boot_meta.fw_size,
        (unsigned long)s_boot_meta.fw_checksum);

    return SRV_BOOT_CTRL_OK;
}

srv_boot_ctrl_error_t srv_boot_ctrl_request_boot(void)
{
    if (!s_initialized) {
        return SRV_BOOT_CTRL_ERROR_UNINITIALIZED;
    }

    /* 置升级标志：bootloader 启动时判定 upgrade_flag != 0 → 进入 boot 模式 */
    s_boot_meta.upgrade_flag = 1U;

    if (ring_storage_save(&s_ctx) != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_E("请求进入 bootloader 失败：保存 upgrade_flag 失败");
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    SRV_BOOT_CTRL_LOG_I("请求进入 bootloader，系统复位...");


    
    NVIC_SystemReset();

    /* 不可达 */
    return SRV_BOOT_CTRL_OK;
}

/* Private functions ---------------------------------------------------------*/

static void boot_meta_init_defaults(void)
{
    s_boot_meta.magic          = BOOT_METADATA_MAGIC;
    s_boot_meta.boot_partition = 0U;        /* BOOT_PARTITION_A */
    s_boot_meta.upgrade_flag   = 0U;
    s_boot_meta.version        = 1U;
    s_boot_meta.fw_size        = 0U;
    s_boot_meta.fw_checksum    = 0U;
    s_boot_meta.reboot_counts  = 0U;
    s_boot_meta.reserved       = 0U;
}
