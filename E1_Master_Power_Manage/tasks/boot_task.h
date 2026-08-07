/**
 * @file    boot_task.h
 * @brief   Boot 主任务 — 启动决策 + CAN 升级接收（胶水层）
 * @attention
 *
 * 自 stm32_g474_boot/tasks/boot_task.h 移植，适配 STM32F407。
 * 串联 CAN ISR → 升级状态机 → Flash 分区。
 */

#ifndef __BOOT_TASK_H
#define __BOOT_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 上电启动决策：检查 Metadata，决定跳转 App 或进入 bootloader
 * @return true 表示已跳转到 App（不会返回），false 表示进入 bootloader 模式
 */
bool boot_task_try_boot_app(void);

/**
 * @brief 初始化 bootloader 主任务
 *
 * 初始化 CAN、Flash、状态机，注册 ISR 回调，创建轮询定时器。
 * 调用后进入 CAN 升级监听循环。
 */
void boot_task_init(void);

/**
 * @brief 查询当前升级状态（boot_fsm 状态）
 * @return 当前状态（BOOT_STATE_*；FSM 未初始化时返回 BOOT_STATE_IDLE）
 */
uint8_t boot_task_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_TASK_H */
