/**
 * @file    srv_key.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   按键服务（基于 key_base 中间件，KEY1/KEY2 事件）
 * @attention
 *
 * 依赖 key_base（public_layer/m_middlewares）：
 *   - 消抖 + 单击/双击/三连击/长按等事件状态机
 *   - 事件回调在 key_base_task() 中（主循环 sw_timer 上下文）执行
 * 引脚读取经 drv_key 解耦，不直接依赖 HAL。
 */

#ifndef __SRV_KEY_H
#define __SRV_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "key_base.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 按键事件回调函数类型（key_base 事件，携带按键名）
 * @param name  按键名称（"key1"/"key2"）
 * @param event 按键事件
 */
typedef void (*srv_key_event_cb_t)(const char* name, key_base_event_t event);

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

/**
 * @brief 初始化按键服务
 * @note  内部调用 key_base 注册 KEY1/KEY2 静态实例；
 *        需在 drv_key_init() 之后调用。
 */
void srv_key_init(void);

/* --- 事件 --- */

/**
 * @brief 注册按键事件回调（上层业务订阅按键事件）
 * @param callback 事件回调（NULL=取消）
 */
void srv_key_register_event_cb(srv_key_event_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_KEY_H */
