/**
 * @file    srv_pwr_det.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   电源状态监控服务实现 (E1_SLAVER_POWER_CTU)
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_pwr_det.h"

#include "drv_status.h"
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_PWR_DET_LOG_ENABLE 1

#if SRV_PWR_DET_LOG_ENABLE
#define SRV_PWR_DET_LOG_E(...) LOG_E("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_W(...) LOG_W("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_I(...) LOG_I("srv_pwr_det", __VA_ARGS__)
#define SRV_PWR_DET_LOG_D(...) LOG_D("srv_pwr_det", __VA_ARGS__)
#else
#define SRV_PWR_DET_LOG_E(...) ((void)0)
#define SRV_PWR_DET_LOG_W(...) ((void)0)
#define SRV_PWR_DET_LOG_I(...) ((void)0)
#define SRV_PWR_DET_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 状态遥测日志限频窗口 (ms)：轮询需限频防刷屏 */
#define SRV_PWR_DET_LOG_PERIOD_MS (1000U)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/** @brief 状态遥测日志时间戳 (ms) */
static uint32_t s_pwr_det_log_ts;

/* Exported functions --------------------------------------------------------*/

void srv_pwr_det_init(void)
{
    /* 本服务封装 drv_status 读取 PGOOD，必须先初始化其状态位 */
    drv_status_init();

    s_initialized = true;

    SRV_PWR_DET_LOG_I("电源状态监控服务初始化完成 (%u 路 PGOOD)", (unsigned)DRV_STATUS_NUM);
}

void srv_pwr_det_read(srv_pwr_det_status_t* status)
{
    if (!status) {
        return;
    }

    uint32_t sta = s_initialized ? drv_status_read_all() : 0;

    status->dc24v_pgood = ((sta >> DRV_STATUS_24V_PGD) & 1U) != 0;
    status->iso12v_pgood = ((sta >> DRV_STATUS_ISO12V_PGD) & 1U) != 0;

    /* 状态遥测日志（限频 1s） */
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - s_pwr_det_log_ts) >= SRV_PWR_DET_LOG_PERIOD_MS) {
        s_pwr_det_log_ts = now_ms;

        SRV_PWR_DET_LOG_D("电源状态: 24V_PGD=%u ISO12V_PGD=%u",
            (unsigned)status->dc24v_pgood, (unsigned)status->iso12v_pgood);
    }
}
