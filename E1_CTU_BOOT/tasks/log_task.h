/**
 * @file    log_task.h
 * @brief   日志输出任务（本地副本，E1_CTU_BOOT，参考兄弟工程 log_task）
 *
 * 支持 USART1 TX DMA 输出后端（drv_log_uart），行格式/时间戳与兄弟工程一致。
 * 与兄弟工程差异：Boot 无 sw_timer 框架，由主循环周期调用 log_task_poll()
 * 排空 log FIFO（等价于兄弟工程的 sw_timer 回调）。
 */

#ifndef LOG_TASK_H
#define LOG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化日志任务
 * @note  初始化 m_middlewares log 模块（log_init + log_set_level）与
 *        drv_log_uart（USART1 TX DMA）输出后端
 */
void log_task_init(void);

/**
 * @brief 主循环周期调用：把 log FIFO 排空到 UART 后端（单次最多一包）
 */
void log_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_TASK_H */
