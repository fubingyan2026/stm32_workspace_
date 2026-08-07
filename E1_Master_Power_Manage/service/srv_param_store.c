/**
 * @file    srv_param_store.c
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-08-06
 * @brief   参数存储服务实现 — 单分区（APP 参数），自持 ring_storage 实例
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_param_store.h"

#include "hal_flash.h"
#include "log.h"
#include "ring_storage.h"
#include "ring_storage_port_hal.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_PARAM_STORE_LOG_ENABLE 1

#if SRV_PARAM_STORE_LOG_ENABLE
#define SRV_PARAM_STORE_LOG_E(...) LOG_E("srv_param_store", __VA_ARGS__)
#define SRV_PARAM_STORE_LOG_W(...) LOG_W("srv_param_store", __VA_ARGS__)
#define SRV_PARAM_STORE_LOG_I(...) LOG_I("srv_param_store", __VA_ARGS__)
#define SRV_PARAM_STORE_LOG_D(...) LOG_D("srv_param_store", __VA_ARGS__)
#else
#define SRV_PARAM_STORE_LOG_E(...) ((void)0)
#define SRV_PARAM_STORE_LOG_W(...) ((void)0)
#define SRV_PARAM_STORE_LOG_I(...) ((void)0)
#define SRV_PARAM_STORE_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 本分区 Flash 几何（APP 常用参数分区，扇区 9-10，128KB×2） */
#define SRV_PARAM_STORE_START_ADDR (0x080A0000U) /* 扇区 9-10，128KB×2 */
#define SRV_PARAM_STORE_AREA_SIZE (0x00040000U) /* 256KB = 2×128KB */
#define SRV_PARAM_STORE_SECTOR_SIZE (RING_STORAGE_SECTOR_128K)

/** @brief 帧缓冲区大小 (RAM)，须能容纳最大一帧（28B 开销 + 所有 KV） */
#define SRV_PARAM_STORE_FRAME_BUF_SIZE (512U)

/* Private variables ---------------------------------------------------------*/

static ring_storage_context_t s_ctx; /**< 本服务独立的 ring_storage 实例 */
static uint8_t s_frame_buf[SRV_PARAM_STORE_FRAME_BUF_SIZE];
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static srv_param_store_error_t rs_to_srv(ring_storage_error_t err);

/* Exported functions --------------------------------------------------------*/

srv_param_store_error_t srv_param_store_init(void)
{
    /* 幂等：二次初始化直接返回 */
    if (s_initialized) {
        return SRV_PARAM_STORE_OK;
    }

    /* Flash 硬件初始化（与 srv_boot_ctrl 共享底层驱动，各自负责自己的 init） */
    if (hal_flash_init() != HAL_FLASH_OK) {
        SRV_PARAM_STORE_LOG_E("hal_flash 初始化失败");
        return SRV_PARAM_STORE_ERROR_HAL_FLASH;
    }

    const ring_storage_config_t rs_cfg = {
        .port = ring_storage_port_hal(),
        .start_addr = SRV_PARAM_STORE_START_ADDR,
        .area_size = SRV_PARAM_STORE_AREA_SIZE,
        .sector_size = SRV_PARAM_STORE_SECTOR_SIZE,
        /* 直接使用 flash 驱动上报的编程颗粒度（f4_dev.caps.write_gran），与 hal_flash 零漂移 */
        .write_gran = (ring_storage_write_gran_t)hal_flash_get_caps()->write_gran,
        .frame_buffer = s_frame_buf,
        .frame_buffer_size = sizeof(s_frame_buf),
    };
    const ring_storage_error_t err = ring_storage_init(&s_ctx, &rs_cfg);
    if (err != RING_STORAGE_OK) {
        SRV_PARAM_STORE_LOG_E("APP 分区 ring_storage 初始化失败 (err=%d)", (int)err);
        return SRV_PARAM_STORE_ERROR_RING_STORAGE;
    }

    s_initialized = true;
    SRV_PARAM_STORE_LOG_I("参数存储服务初始化完成: app=0x%08X/%luKB",
        (unsigned)SRV_PARAM_STORE_START_ADDR,
        (unsigned long)(SRV_PARAM_STORE_AREA_SIZE >> 10));

    return SRV_PARAM_STORE_OK;
}

srv_param_store_error_t srv_param_store_register(const char* key,
    void* value, uint16_t value_len)
{
    if (!s_initialized) {
        return SRV_PARAM_STORE_ERROR_UNINITIALIZED;
    }
    if (!key || !value) {
        return SRV_PARAM_STORE_ERROR_INVALID_PARAM;
    }
    return rs_to_srv(ring_storage_register(&s_ctx, key, value, value_len));
}

srv_param_store_error_t srv_param_store_load(void)
{
    if (!s_initialized) {
        return SRV_PARAM_STORE_ERROR_UNINITIALIZED;
    }
    return rs_to_srv(ring_storage_load(&s_ctx));
}

srv_param_store_error_t srv_param_store_save(void)
{
    if (!s_initialized) {
        return SRV_PARAM_STORE_ERROR_UNINITIALIZED;
    }
    return rs_to_srv(ring_storage_save(&s_ctx));
}

/* Exported functions (continued) --------------------------------------------*/

const char* srv_param_store_err_str(srv_param_store_error_t err)
{
    switch (err) {
    case SRV_PARAM_STORE_OK: return "操作成功";
    case SRV_PARAM_STORE_ERROR_NULL_PTR: return "空指针参数";
    case SRV_PARAM_STORE_ERROR_INVALID_PARAM: return "非法参数";
    case SRV_PARAM_STORE_ERROR_UNINITIALIZED: return "服务未初始化";
    case SRV_PARAM_STORE_ERROR_HAL_FLASH: return "hal_flash 底层错误";
    case SRV_PARAM_STORE_ERROR_RING_STORAGE: return "ring_storage 底层错误";
    case SRV_PARAM_STORE_ERROR_NO_VALID_FRAME: return "分区无有效帧（首次使用）";
    default: return "未知错误";
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief ring_storage 错误码 → 服务错误码
 */
static srv_param_store_error_t rs_to_srv(ring_storage_error_t err)
{
    if (err == RING_STORAGE_OK) {
        return SRV_PARAM_STORE_OK;
    }
    if (err == RING_STORAGE_ERROR_NO_VALID_FRAME) {
        return SRV_PARAM_STORE_ERROR_NO_VALID_FRAME;
    }
    return SRV_PARAM_STORE_ERROR_RING_STORAGE;
}
