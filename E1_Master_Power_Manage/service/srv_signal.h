//
// Created by fubingyan on 25-9-20.
//

/**
 * @file    srv_signal.h
 * @brief   信号/输出控制模块 — 支持 ON/OFF/编码闪烁/呼吸四种状态
 * @note    使用 FSM 管理状态转换，kfifo 异步命令队列接收外部指令。
 *          引脚/输出操作通过 config 回调解耦（uint16_t PWM 接口），不直接依赖 HAL。
 *          适用于 LED、蜂鸣器等任何可用 0-1023 逻辑值驱动的输出设备。
 *          实例链表使用侵入式 clist 管理。
 */

#ifndef __SRV_SIGNAL_H
#define __SRV_SIGNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "clist.h"
#include "fsm.h"
#include "msg_fifo.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 信号输出错误码
 */
typedef enum {
    SRV_SIGNAL_OK = 0, /**< 操作成功 */
    SRV_SIGNAL_OK_EXISTED = 1, /**< 已初始化 */
    SRV_SIGNAL_ERROR_INVALID_PARAM = -1, /**< 无效参数 */
    SRV_SIGNAL_ERROR_NO_MEMORY = -2, /**< 内存不足 */
    SRV_SIGNAL_ERROR_NOT_FOUND = -3, /**< 未找到实例 */
    SRV_SIGNAL_ERROR_ALREADY_EXIST = -4, /**< 同名实例已存在 */
    SRV_SIGNAL_ERROR_INTERNAL = -5, /**< 内部错误 */
} srv_signal_error_t;

/**
 * @brief 输出工作状态
 */
typedef enum __attribute__((packed)) {
    SRV_SIGNAL_STATE_NONE = 0, /**< 无状态（空闲） */
    SRV_SIGNAL_STATE_OFF, /**< 关闭（输出 0） */
    SRV_SIGNAL_STATE_ON, /**< 常开（输出最大） */
    SRV_SIGNAL_STATE_BLINK_CODE, /**< 编码闪烁 */
    SRV_SIGNAL_STATE_BREATHING, /**< 呼吸 */
    SRV_SIGNAL_STATE_MAX, /**< 状态总数 */
} srv_signal_state_t;

/**
 * @brief 闪烁阶段
 */
typedef enum __attribute__((packed)) {
    SRV_SIGNAL_BLINK_PHASE_BLINKING = 0, /**< 闪烁中：按 cycle_ms 切换开/关 */
    SRV_SIGNAL_BLINK_PHASE_INTERVAL, /**< 间隔中：按 wait_ms 保持关闭 */
} srv_signal_blink_phase_t;

/**
 * @brief 信号输出句柄前向声明
 */
typedef struct srv_signal_handle srv_signal_handle_t;

/**
 * @brief 系统时间获取回调
 * @return 毫秒时间戳
 */
typedef uint32_t (*srv_signal_get_time_cb_t)(void);

/**
 * @brief 状态变化回调
 * @param instance 触发回调的实例
 * @param new_state 新状态
 * @param user_data 用户数据
 */
typedef void (*srv_signal_state_change_cb_t)(srv_signal_handle_t* instance,
    srv_signal_state_t new_state, void* user_data);

/**
 * @brief 闪烁阶段变化回调
 * @param instance 实例
 * @param phase 当前闪烁阶段
 * @param user_data 用户数据
 */
typedef void (*srv_signal_blink_phase_cb_t)(srv_signal_handle_t* instance,
    srv_signal_blink_phase_t phase, void* user_data);

/**
 * @brief 输出电平边沿变化回调
 * @param instance 实例
 * @param rising true=上升沿(开)，false=下降沿(关)
 * @param user_data 用户数据
 */
typedef void (*srv_signal_edge_cb_t)(srv_signal_handle_t* instance, bool rising,
    void* user_data);

/**
 * @brief 呼吸默认参数
 */
#define SRV_SIGNAL_BREATH_CYCLE_MS_DEFAULT (2000U)
#define SRV_SIGNAL_BREATH_STEP_MS_DEFAULT (30U)
#define SRV_SIGNAL_BREATH_MIN_DUTY_DEFAULT (0U)
#define SRV_SIGNAL_BREATH_MAX_DUTY_DEFAULT (1024U)

/** @brief 单实例命令队列缓冲区大小（字节，须为 2 的幂） */
#define SRV_SIGNAL_CMD_BUF_SIZE (128U)

/**
 * @brief 实例配置
 * @note write_output 由用户实现，负责实际设备 PWM/电平操作
 */
typedef struct {
    const char* name; /**< 实例名称（唯一标识） */
    srv_signal_state_t init_state; /**< 初始状态 */
    void (*write_output)(uint16_t value); /**< 输出写入：0=关, 1023=最大, 中间=呼吸 */

    /* 呼吸默认参数（0 表示使用默认值） */
    uint16_t breath_cycle_ms; /**< 呼吸周期(ms), 0=2000 */
    uint16_t breath_step_ms; /**< 步进间隔(ms), 0=30 */
    uint16_t breath_min_duty; /**< 最小亮度(0-1023), 0=0 */
    uint16_t breath_max_duty; /**< 最大亮度(0-1023), 0=1023 */
} srv_signal_config_t;

/**
 * @brief 异步命令
 */
typedef struct {
    srv_signal_state_t set_state; /**< 目标状态 */
    uint16_t blink_cycle_ms; /**< 闪烁间隔(ms) */
    uint16_t blink_wait_ms; /**< 等待间隔(ms) */
    uint16_t blink_code_counts; /**< 闪烁次数（0=无限循环） */

    /* 呼吸参数（运行时修改） */
    uint16_t breath_cycle_ms; /**< 呼吸周期(ms), 0=不变 */
    uint16_t breath_min_duty; /**< 最小亮度, 0xFFFF=不变 */
    uint16_t breath_max_duty; /**< 最大亮度, 0xFFFF=不变 */
} srv_signal_cmd_t;

/**
 * @brief 控制句柄
 */
struct srv_signal_handle {
    srv_signal_config_t config; /**< 配置副本 */
    clist_head_t node; /**< clist 链表节点 */
    fsm_t fsm; /**< FSM 状态机上下文 */

    srv_signal_cmd_t current_cmd; /**< 当前命令参数 */
    uint32_t last_toggle_time; /**< 上次翻转时间戳 */
    uint32_t interval_start_time; /**< 间隔阶段起始时间戳 */

    uint16_t current_blink_code_counts; /**< 当前闪烁计数 */
    srv_signal_blink_phase_t blink_code_phase; /**< 当前闪烁阶段 */
    srv_signal_blink_phase_t blink_code_phase_last; /**< 上次闪烁阶段 */
    bool blink_sw_on; /**< 软件跟踪输出开/关状态 */

    /* 呼吸状态 */
    uint32_t last_breath_time; /**< 上次呼吸步进时间戳 */
    uint16_t breath_cycle; /**< 呼吸步进计数器 */
    uint16_t breath_step_ms; /**< 步进间隔(ms) */
    uint16_t breath_cycle_ms; /**< 呼吸周期(ms) */
    uint16_t breath_min_duty; /**< 最小亮度(0-1023) */
    uint16_t breath_max_duty; /**< 最大亮度(0-1023) */
    uint16_t breath_value; /**< 当前计算出的输出值 */

    uint16_t last_write_value; /**< 上次写入的值（用于状态过渡） */

    bool is_static; /**< 静态分配标志 */
    bool initialized; /**< 初始化完成标志 */
    bool pending_blink_update; /**< 待处理的闪烁参数更新 */

    msg_fifo_t cmd_fifo; /**< 异步命令队列实例（每实例独立，支持多实例注册） */
    uint8_t cmd_buffer[SRV_SIGNAL_CMD_BUF_SIZE]; /**< 命令队列缓冲区（2 的幂字节） */

    void* state_change_cb; /**< 状态变化回调 */
    void* blink_phase_cb; /**< 闪烁阶段变化回调 */
    void* edge_cb; /**< 电平边沿回调 */
    void* callback_user_data; /**< 回调用户数据 */
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define SRV_SIGNAL_IS_OK(err) ((err) >= 0)
#define SRV_SIGNAL_IS_ERR(err) ((err) < 0)

/* Exported functions prototypes ---------------------------------------------*/

srv_signal_error_t srv_signal_init(srv_signal_get_time_cb_t get_time_cb);
void srv_signal_deinit(void);

srv_signal_error_t srv_signal_register_static(const srv_signal_config_t* config,
    srv_signal_handle_t* instance);
srv_signal_error_t srv_signal_unregister(const char* name);
srv_signal_handle_t* srv_signal_get_instance(const char* name);
clist_head_t* srv_signal_get_head(void);

void srv_signal_set_state(srv_signal_handle_t* instance, srv_signal_state_t state);
srv_signal_error_t srv_signal_set_blink_interval(srv_signal_handle_t* instance,
    const srv_signal_cmd_t* cmd);
srv_signal_blink_phase_t srv_signal_get_blink_phase(srv_signal_handle_t* instance);

void srv_signal_set_callbacks(srv_signal_handle_t* instance, srv_signal_state_change_cb_t state_cb,
    srv_signal_blink_phase_cb_t blink_phase_cb,
    srv_signal_edge_cb_t edge_cb, void* user_data);

void srv_signal_task_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_SIGNAL_H */
