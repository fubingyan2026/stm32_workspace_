/**
 * @file    srv_pwr_det.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-09-03
 * @brief   电源状态监控服务 — 封装 drv_status PGOOD 读取 (E1_SLAVER_POWER_CTU)
 *
 * 提供语义化的电源状态查询接口。PGOOD 经 drv_status 读取。
 * 使能门控/故障锁存判定由 srv_pwr_ctrl 负责，本服务只做瞬时物理量采集。
 */

#ifndef __SRV_PWR_DET_H
#define __SRV_PWR_DET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电源状态汇总（瞬时，未做门控）
 */
typedef struct {
    bool dc24v_pgood;   /**< 24V 降压输出正常（EXT_PCOOG_24V） */
    bool iso12v_pgood;  /**< 12V_ISO 输出正常（ISO_PGOOD_12V） */
} srv_pwr_det_status_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化电源监控服务（内部初始化 drv_status） */
void srv_pwr_det_init(void);

/** @brief 读取电源状态（批量，推荐用于 RS485 上报打包） */
void srv_pwr_det_read(srv_pwr_det_status_t* status);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_PWR_DET_H */
