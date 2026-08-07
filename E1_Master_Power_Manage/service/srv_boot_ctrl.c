/**
 * @file    srv_boot_ctrl.c
 * @author  maximillian
 * @version V1.2.0
 * @date    2026-08-06
 * @brief   Boot 控制服务实现 — 自持 BOOT 分区 ring_storage 实例
 *
 * 同层解耦：本服务独立持有 BOOT 分区 flash 实例，不调用 srv_param_store。
 * boot_metadata_t 为私有类型（原 boot_metadata.h 已并入本文件）。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_boot_ctrl.h"

#include "drv_systick.h"
#include "hal_flash.h"
#include "log.h"
/* log_task_flush：跳转/复位前排空日志（跨层系统工具，服务层显式调用） */
#include "log_task.h"
#include "ring_storage.h"
#include "ring_storage_port_hal.h"

/* 模块日志开关 ----------------------------------------------------------------*/

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

/* Private types -------------------------------------------------------------*/

/** @brief Metadata 魔数 "BOOT"（与 g474_boot 一致） */
#define BOOT_METADATA_MAGIC (0x424F4F54U)

/** @brief 引导分区标识 */
typedef enum {
    BOOT_PARTITION_A = 0U, /**< App A 分区 */
    BOOT_PARTITION_B = 1U, /**< App B 分区 */
} boot_partition_t;

/**
 * @brief Boot 引导 Metadata（私有类型，原 boot_metadata.h 已并入本文件）
 * @note 字段布局与 stm32_g474_boot/service/boot/boot_flash.h 的 boot_metadata_t
 *       完全一致，构成与 bootloader 的共享字节契约，不得增删字段/改变类型。
 *       sizeof = 24B，字段对齐无需 packed。
 */
typedef struct {
    uint32_t magic; /**< 魔数，标识有效 metadata */
    uint8_t boot_partition; /**< 当前引导分区 */
    uint8_t upgrade_flag; /**< 升级标志：0=正常，1=升级中，2=升级完成待验证 */
    uint16_t version; /**< 固件版本号 */
    uint32_t fw_size; /**< 固件大小 */
    uint32_t fw_checksum; /**< 固件校验和 */
    uint32_t reboot_counts; /**< MCU 上电启动次数 */
    uint32_t reserved; /**< 预留 */
} boot_metadata_t;

/* Private constants ---------------------------------------------------------*/

/** @brief BOOT 分区 Flash 几何（Metadata，扇区 7-8；与 boot_flash 共享同一区域与字节契约） */
#define SRV_BOOT_CTRL_START_ADDR (0x08060000U) /* 扇区 7-8，128KB×2 */
#define SRV_BOOT_CTRL_AREA_SIZE (0x00040000U) /* 256KB = 2×128KB */
#define SRV_BOOT_CTRL_SECTOR_SIZE (RING_STORAGE_SECTOR_128K)

/** @brief 帧缓冲区大小 (RAM)：28B 开销 + "meta" KV(31B) = ~59B */
#define SRV_BOOT_CTRL_FRAME_BUF_SIZE (128U)

/* Private variables ---------------------------------------------------------*/

static ring_storage_context_t s_ctx; /**< 本服务独立的 BOOT 分区 ring_storage 实例 */
static uint8_t s_frame_buf[SRV_BOOT_CTRL_FRAME_BUF_SIZE];
static boot_metadata_t s_boot_meta; /**< boot metadata（"meta" KV 绑定） */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void boot_meta_init_defaults(void);

/* Exported functions --------------------------------------------------------*/

srv_boot_ctrl_error_t srv_boot_ctrl_init(void)
{
    /* 幂等：二次初始化直接返回 */
    if (s_initialized) {
        return SRV_BOOT_CTRL_OK;
    }

    /* Flash 硬件初始化（与 srv_param_store 共享底层驱动，各自负责自己的 init） */
    if (hal_flash_init() != HAL_FLASH_OK) {
        SRV_BOOT_CTRL_LOG_E("hal_flash 初始化失败");
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 初始化本服务独立的 BOOT 分区 ring_storage */
    const ring_storage_config_t rs_cfg = {
        .port = ring_storage_port_hal(),
        .start_addr = SRV_BOOT_CTRL_START_ADDR,
        .area_size = SRV_BOOT_CTRL_AREA_SIZE,
        .sector_size = SRV_BOOT_CTRL_SECTOR_SIZE,
        /* 直接使用 flash 驱动上报的编程颗粒度（f4_dev.caps.write_gran），与 hal_flash 零漂移 */
        .write_gran = (ring_storage_write_gran_t)hal_flash_get_caps()->write_gran,
        .frame_buffer = s_frame_buf,
        .frame_buffer_size = sizeof(s_frame_buf),
    };
    const ring_storage_error_t rs_err = ring_storage_init(&s_ctx, &rs_cfg);
    if (rs_err != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_E("BOOT 分区 ring_storage 初始化失败 (err=%d)", (int)rs_err);
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 注册 "meta" KV（值绑定本服务静态结构体） */
    if (ring_storage_register(&s_ctx, "meta",
            &s_boot_meta, sizeof(s_boot_meta)) != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_E("BOOT 分区注册 meta KV 失败");
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    /* 加载 metadata；首次使用（无有效帧）初始化默认字段 */
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

    /* 上电计数 +1 并整帧持久化（与 g474_boot 行为一致） */
    s_boot_meta.reboot_counts++;
    err = ring_storage_save(&s_ctx);
    if (err != RING_STORAGE_OK) {
        SRV_BOOT_CTRL_LOG_W("BOOT 分区保存失败 (err=%d)", (int)err);
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    s_initialized = true;
    SRV_BOOT_CTRL_LOG_I("Boot 控制服务初始化完成 (reboot_counts=%u, partition=%c, version=%u)",
        (unsigned)s_boot_meta.reboot_counts,
        (s_boot_meta.boot_partition == BOOT_PARTITION_A) ? 'A' : 'B',
        (unsigned)s_boot_meta.version);

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
        SRV_BOOT_CTRL_LOG_E("请求进入 bootloader 失败：BOOT 分区保存失败");
        return SRV_BOOT_CTRL_ERROR_FLASH;
    }

    SRV_BOOT_CTRL_LOG_I("请求进入 bootloader，系统复位...");
    /* 复位前排空日志（UART DMA 异步），确保"请求进入 bootloader"已真正发出 */
    log_task_flush();
    drv_system_reset();

    /* 复位后不可达 */
    return SRV_BOOT_CTRL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 初始化默认 metadata 字段（首次使用）
 */
static void boot_meta_init_defaults(void)
{
    s_boot_meta.magic = BOOT_METADATA_MAGIC;
    s_boot_meta.boot_partition = BOOT_PARTITION_A;
    s_boot_meta.upgrade_flag = 0U;
    s_boot_meta.version = 1U;
    s_boot_meta.fw_size = 0U;
    s_boot_meta.fw_checksum = 0U;
    s_boot_meta.reboot_counts = 0U;
    s_boot_meta.reserved = 0U;
}
