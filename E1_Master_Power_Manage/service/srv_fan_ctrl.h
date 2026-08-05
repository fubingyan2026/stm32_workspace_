/**
 * @file    srv_fan_ctrl.h
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-07-8
 * @brief   风扇控制服务 — RPM 计算 + 温控调速 + 故障检测
 *
 * service 层提供 RPM 换算、温度→PWM 映射、堵转判断。
 * 不管理 sw_timer，由 task 层定期调用 srv_fan_ctrl_step()。
 * 硬件引脚配置内置在 drv_fan 中，上层无需传递。
 */

#ifndef __SRV_FAN_CTRL_H
#define __SRV_FAN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/** @brief 风扇状态 */
typedef struct {
    uint32_t rpm; /**< 当前转速（低通滤波后） */
    uint8_t duty; /**< 当前占空比 (0-100) */
    bool fault; /**< 故障标志（堵转/转速过低） */
} srv_fan_ctrl_status_t;

/**
 * @brief 读取风扇区域温度回调（task 层实现）
 * @param id 风扇编号 (0-max)
 * @return 当前温度（0.01°C，如 4500 = 45.00°C）
 */
typedef int16_t (*srv_fan_ctrl_temp_read_cb_t)(uint8_t id);

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化风扇控制服务
 * @param temp_read 温度读取回调（NULL=禁用温控，仅手动调速）
 */
void srv_fan_ctrl_init(srv_fan_ctrl_temp_read_cb_t temp_read);

/**
 * @brief 周期步进（task 层每周期调用）
 * @param elapsed_ms 距离上次调用的毫秒数
 */
void srv_fan_ctrl_step(uint16_t elapsed_ms);

/** @brief 手动设置风扇占空比 */
void srv_fan_ctrl_set_duty(uint8_t id, uint8_t duty);

/** @brief 获取风扇状态 */
const srv_fan_ctrl_status_t* srv_fan_ctrl_get_status(uint8_t id);

/** @brief 是否有任何风扇故障 */
bool srv_fan_ctrl_any_fault(void);

/** @brief 查询单个风扇是否故障 */
bool srv_fan_ctrl_is_fault(uint8_t id);

/** @brief 使能/禁用温度自动调速 */
void srv_fan_ctrl_set_auto(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_FAN_CTRL_H */
