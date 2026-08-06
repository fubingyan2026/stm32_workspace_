//
// Created by fubingyan on 25-9-20.
//

/**
 * @file    srv_signal.c
 * @brief   信号/输出控制模块实现
 * @note    通过 FSM 管理 ON/OFF/BLINK_CODE/BREATHING 四种工作状态。
 *          命令通过 kfifo 异步队列传递，支持闪烁/呼吸参数热更新。
 *          输出通过 uint16_t PWM 接口回调（0=关, 1023=最大）。
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_signal.h"

#include "log.h"
#include "maths.h"
#include "msg_fifo.h"
#include <string.h>

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_SIGNAL_LOG_ENABLE 1

#if SRV_SIGNAL_LOG_ENABLE
#define SRV_SIGNAL_LOG_E(...) LOG_E("srv_signal", __VA_ARGS__)
#define SRV_SIGNAL_LOG_W(...) LOG_W("srv_signal", __VA_ARGS__)
#define SRV_SIGNAL_LOG_I(...) LOG_I("srv_signal", __VA_ARGS__)
#define SRV_SIGNAL_LOG_D(...) LOG_D("srv_signal", __VA_ARGS__)
#else
#define SRV_SIGNAL_LOG_E(...) ((void)0)
#define SRV_SIGNAL_LOG_W(...) ((void)0)
#define SRV_SIGNAL_LOG_I(...) ((void)0)
#define SRV_SIGNAL_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

#define SRV_SIGNAL_PWM_MAX 1023U
#define SRV_SIGNAL_GAMMA 2.2f

/* Private variables ---------------------------------------------------------*/

static clist_head_t s_signal_head;
static srv_signal_get_time_cb_t s_signal_get_time;
static bool s_signal_initialized;

/* Private function prototypes -----------------------------------------------*/

static void signal_phys_write(srv_signal_handle_t* handle, uint16_t value);
static fsm_state_t signal_fsm_static_handler(fsm_t* ctx);
static fsm_state_t signal_fsm_blink_handler(fsm_t* ctx);
static fsm_state_t signal_fsm_breathing_handler(fsm_t* ctx);
static void signal_fsm_on_entry(fsm_t* ctx, fsm_state_t state);
static void signal_process_cmds(srv_signal_handle_t* handle);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 物理写输出 PWM
 */
static void signal_phys_write(srv_signal_handle_t* handle, uint16_t value)
{
    handle->last_write_value = value;
    handle->config.write_output(value);

    if (handle->edge_cb) {
        ((srv_signal_edge_cb_t)handle->edge_cb)(handle, value > 0,
            handle->callback_user_data);
    }
}

/* FSM handlers --------------------------------------------------------------*/

/** @brief 无需定时逻辑的稳定状态（NONE/OFF/ON）统一用此 handler */
static fsm_state_t signal_fsm_static_handler(fsm_t* ctx)
{
    return fsm_current_state(ctx);
}

/**
 * @brief FSM BLINK_CODE 状态处理：编码闪烁
 */
static fsm_state_t signal_fsm_blink_handler(fsm_t* ctx)
{
    srv_signal_handle_t* handle = (srv_signal_handle_t*)fsm_user_data(ctx);
    uint32_t now = s_signal_get_time ? s_signal_get_time() : 0;

    if (handle->blink_code_phase == SRV_SIGNAL_BLINK_PHASE_BLINKING) {
        if ((uint32_t)(now - handle->last_toggle_time) >= handle->current_cmd.blink_cycle_ms) {
            handle->last_toggle_time = now;
            handle->blink_sw_on = !handle->blink_sw_on;
            signal_phys_write(handle, handle->blink_sw_on ? SRV_SIGNAL_PWM_MAX : 0);
            if (!handle->blink_sw_on) {
                handle->current_blink_code_counts++;
                if (handle->current_cmd.blink_code_counts == 0 || handle->current_blink_code_counts >= handle->current_cmd.blink_code_counts) {
                    handle->blink_code_phase_last = handle->blink_code_phase;
                    handle->blink_code_phase = SRV_SIGNAL_BLINK_PHASE_INTERVAL;
                    handle->interval_start_time = now;
                    handle->current_blink_code_counts = 0;
                    if (handle->current_cmd.blink_code_counts > 0) {
                        return SRV_SIGNAL_STATE_OFF;
                    }
                }
            }
        }
    } else {
        if ((uint32_t)(now - handle->interval_start_time) >= handle->current_cmd.blink_wait_ms) {
            handle->blink_code_phase = SRV_SIGNAL_BLINK_PHASE_BLINKING;
            handle->last_toggle_time = now;
        }
    }
    return SRV_SIGNAL_STATE_BLINK_CODE;
}

/**
 * @brief FSM BREATHING 状态处理：呼吸
 */
static fsm_state_t signal_fsm_breathing_handler(fsm_t* ctx)
{
    srv_signal_handle_t* handle = (srv_signal_handle_t*)fsm_user_data(ctx);
    uint32_t now = s_signal_get_time ? s_signal_get_time() : 0;

    if ((uint32_t)(now - handle->last_breath_time) >= handle->breath_step_ms) {
        handle->last_breath_time = now;

        uint16_t total_steps = handle->breath_cycle_ms / handle->breath_step_ms;
        if (total_steps == 0)
            total_steps = 1;

        float phase = (float)handle->breath_cycle * 2.0f * M_PIf / (float)total_steps;
        float brightness = (sin_approx(phase) + 1.0f) * 0.5f;
        float gamma = powerf(brightness, SRV_SIGNAL_GAMMA);
        uint16_t range = handle->breath_max_duty - handle->breath_min_duty;
        handle->breath_value = handle->breath_min_duty + (uint16_t)(gamma * (float)range);
        signal_phys_write(handle, handle->breath_value);

        handle->breath_cycle++;
        if (handle->breath_cycle >= total_steps) {
            handle->breath_cycle = 0;
        }
    }

    return SRV_SIGNAL_STATE_BREATHING;
}

/* FSM entry / exit ----------------------------------------------------------*/

static void signal_fsm_on_entry(fsm_t* ctx, fsm_state_t state)
{
    srv_signal_handle_t* handle = (srv_signal_handle_t*)fsm_user_data(ctx);

    /* 仅在状态进入时打印（FSM 转换边沿），呼吸/闪烁刷新不在此路径 */
    SRV_SIGNAL_LOG_I("SIG[%s] 状态 -> %s", handle->config.name, fsm_name(ctx, state));

    if (handle->state_change_cb) {
        ((srv_signal_state_change_cb_t)handle->state_change_cb)(handle, state,
            handle->callback_user_data);
    }

    switch (state) {
    case SRV_SIGNAL_STATE_ON:
        signal_phys_write(handle, SRV_SIGNAL_PWM_MAX);
        break;
    case SRV_SIGNAL_STATE_OFF:
        signal_phys_write(handle, 0);
        break;
    case SRV_SIGNAL_STATE_BLINK_CODE:
        handle->current_blink_code_counts = 0;
        handle->blink_code_phase_last = handle->blink_code_phase;
        handle->blink_code_phase = SRV_SIGNAL_BLINK_PHASE_INTERVAL;
        handle->interval_start_time = s_signal_get_time ? s_signal_get_time() : 0;
        handle->blink_sw_on = false;
        signal_phys_write(handle, 0);
        break;
    case SRV_SIGNAL_STATE_BREATHING: {
        handle->last_breath_time = s_signal_get_time ? s_signal_get_time() : 0;
        handle->breath_value = handle->last_write_value;

        uint16_t total_steps = handle->breath_cycle_ms / handle->breath_step_ms;
        if (total_steps == 0)
            total_steps = 1;

        uint16_t mid = (uint16_t)(((uint32_t)handle->breath_min_duty + handle->breath_max_duty) / 2U);
        handle->breath_cycle = (handle->last_write_value >= mid)
            ? (uint16_t)((uint32_t)total_steps * 1U / 4U)
            : (uint16_t)((uint32_t)total_steps * 3U / 4U);
        break;
    }
    default:
        break;
    }
}

/* Command processing --------------------------------------------------------*/

static void signal_process_cmds(srv_signal_handle_t* handle)
{
    srv_signal_cmd_t cmd;
    while (msg_fifo_pop(&handle->cmd_fifo, &cmd)) {
        if (cmd.set_state != SRV_SIGNAL_STATE_NONE) {
            fsm_goto(&handle->fsm, cmd.set_state);
        }

        if (cmd.set_state == SRV_SIGNAL_STATE_BLINK_CODE) {
            if (handle->pending_blink_update && fsm_current_state(&handle->fsm) == SRV_SIGNAL_STATE_BLINK_CODE) {
                if (handle->blink_sw_on) {
                    msg_fifo_push(&handle->cmd_fifo, &cmd);
                    break;
                }
                handle->pending_blink_update = false;
            }

            if (cmd.blink_cycle_ms > 0)
                handle->current_cmd.blink_cycle_ms = cmd.blink_cycle_ms;
            if (cmd.blink_wait_ms > 0)
                handle->current_cmd.blink_wait_ms = cmd.blink_wait_ms;
            if (cmd.blink_code_counts > 0)
                handle->current_cmd.blink_code_counts = cmd.blink_code_counts;

            if (fsm_current_state(&handle->fsm) == SRV_SIGNAL_STATE_BLINK_CODE) {
                handle->current_blink_code_counts = 0;
                handle->blink_code_phase = SRV_SIGNAL_BLINK_PHASE_INTERVAL;
                handle->interval_start_time = s_signal_get_time ? s_signal_get_time() : 0;
                handle->blink_sw_on = false;
                signal_phys_write(handle, 0);
            }
        }

        if (cmd.set_state == SRV_SIGNAL_STATE_BREATHING || cmd.breath_cycle_ms > 0) {
            if (cmd.breath_cycle_ms > 0)
                handle->breath_cycle_ms = cmd.breath_cycle_ms;
            if (cmd.breath_min_duty < 0xFFFF)
                handle->breath_min_duty = cmd.breath_min_duty;
            if (cmd.breath_max_duty < 0xFFFF)
                handle->breath_max_duty = cmd.breath_max_duty;
            handle->breath_step_ms = handle->breath_cycle_ms / 66;
            if (handle->breath_step_ms < 10)
                handle->breath_step_ms = 10;
        }
    }
}

/* Exported functions --------------------------------------------------------*/

srv_signal_error_t srv_signal_init(srv_signal_get_time_cb_t get_time_cb)
{
    if (get_time_cb == NULL) {
        SRV_SIGNAL_LOG_E("信号服务初始化失败: 时间回调为空");
        return SRV_SIGNAL_ERROR_INVALID_PARAM;
    }
    if (s_signal_initialized)
        return SRV_SIGNAL_OK_EXISTED;

    s_signal_get_time = get_time_cb;
    clist_init(&s_signal_head);
    s_signal_initialized = true;

    SRV_SIGNAL_LOG_I("信号输出服务初始化完成");
    return SRV_SIGNAL_OK;
}

void srv_signal_deinit(void)
{
    if (!s_signal_initialized)
        return;

    clist_head_t *pos, *tmp;
    clist_for_each_safe(pos, tmp, &s_signal_head)
    {
        srv_signal_handle_t* h = clist_entry(pos, srv_signal_handle_t, node);
        clist_del(pos);
        kfifo_reset(&h->cmd_fifo.fifo);
    }

    clist_init(&s_signal_head);
    s_signal_initialized = false;
}

srv_signal_handle_t* srv_signal_get_instance(const char* name)
{
    if (name == NULL || !s_signal_initialized)
        return NULL;

    srv_signal_handle_t* h;
    clist_for_each_entry(h, &s_signal_head, node)
    {
        if (strcmp(h->config.name, name) == 0)
            return h;
    }
    return NULL;
}

clist_head_t* srv_signal_get_head(void)
{
    return s_signal_initialized ? &s_signal_head : NULL;
}

srv_signal_error_t srv_signal_register_static(const srv_signal_config_t* config,
    srv_signal_handle_t* instance)
{
    if (config == NULL || instance == NULL || config->name == NULL || config->write_output == NULL) {
        SRV_SIGNAL_LOG_E("实例注册失败: 参数为空或输出回调未设置");
        return SRV_SIGNAL_ERROR_INVALID_PARAM;
    }
    if (!s_signal_initialized)
        return SRV_SIGNAL_ERROR_INTERNAL;
    if (srv_signal_get_instance(config->name))
        return SRV_SIGNAL_ERROR_ALREADY_EXIST;

    memset(instance, 0, sizeof(srv_signal_handle_t));
    memcpy(&instance->config, config, sizeof(srv_signal_config_t));

    instance->breath_cycle_ms = config->breath_cycle_ms ? config->breath_cycle_ms : SRV_SIGNAL_BREATH_CYCLE_MS_DEFAULT;
    instance->breath_step_ms = config->breath_step_ms ? config->breath_step_ms : SRV_SIGNAL_BREATH_STEP_MS_DEFAULT;
    instance->breath_min_duty = config->breath_min_duty ? config->breath_min_duty : SRV_SIGNAL_BREATH_MIN_DUTY_DEFAULT;
    instance->breath_max_duty = config->breath_max_duty ? config->breath_max_duty : SRV_SIGNAL_BREATH_MAX_DUTY_DEFAULT;

    /* 初始化 FSM */
    static const char* names[] = { "NONE", "OFF", "ON", "BLINK", "BREATH" };
    static fsm_handler_t handlers[SRV_SIGNAL_STATE_MAX];
    static fsm_guard_t transitions[SRV_SIGNAL_STATE_MAX * SRV_SIGNAL_STATE_MAX];
    memset(handlers, 0, sizeof(handlers));
    memset(transitions, 0, sizeof(transitions));

    handlers[SRV_SIGNAL_STATE_NONE] = signal_fsm_static_handler;
    handlers[SRV_SIGNAL_STATE_OFF] = signal_fsm_static_handler;
    handlers[SRV_SIGNAL_STATE_ON] = signal_fsm_static_handler;
    handlers[SRV_SIGNAL_STATE_BLINK_CODE] = signal_fsm_blink_handler;
    handlers[SRV_SIGNAL_STATE_BREATHING] = signal_fsm_breathing_handler;

    fsm_config_t fsm_cfg = {
        .handlers = handlers,
        .transitions = transitions,
        .state_count = SRV_SIGNAL_STATE_MAX,
        .entry_cb = signal_fsm_on_entry,
        .exit_cb = NULL,
        .state_names = names,
        .user_data = instance,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);
    fsm_init(&instance->fsm, config->init_state, &fsm_cfg);

    /* 每实例独立的命令队列（存储在句柄内，支持多实例注册互不干扰） */
    msg_fifo_init(&instance->cmd_fifo, instance->cmd_buffer,
        sizeof(instance->cmd_buffer), sizeof(srv_signal_cmd_t));

    instance->is_static = true;
    instance->initialized = true;

    clist_add_tail(&s_signal_head, &instance->node);

    SRV_SIGNAL_LOG_I("SIG[%s] 注册成功, 初始状态=%s",
        config->name, fsm_name(&instance->fsm, config->init_state));
    return SRV_SIGNAL_OK;
}

srv_signal_error_t srv_signal_unregister(const char* name)
{
    if (name == NULL || !s_signal_initialized)
        return SRV_SIGNAL_ERROR_INVALID_PARAM;

    srv_signal_handle_t *h, *tmp;
    clist_for_each_entry_safe(h, tmp, &s_signal_head, node)
    {
        if (strcmp(h->config.name, name) == 0) {
            clist_del(&h->node);
            kfifo_reset(&h->cmd_fifo.fifo);
            return SRV_SIGNAL_OK;
        }
    }
    return SRV_SIGNAL_ERROR_NOT_FOUND;
}

void srv_signal_set_state(srv_signal_handle_t* instance, srv_signal_state_t state)
{
    if (instance == NULL)
        return;

    SRV_SIGNAL_LOG_D("SIG[%s] 收到状态命令: %u", instance->config.name, (unsigned)state);
    srv_signal_cmd_t cmd = { .set_state = state };
    msg_fifo_push(&instance->cmd_fifo, &cmd);
}

srv_signal_error_t srv_signal_set_blink_interval(srv_signal_handle_t* instance,
    const srv_signal_cmd_t* cmd)
{
    if (instance == NULL || cmd == NULL)
        return SRV_SIGNAL_ERROR_INVALID_PARAM;

    bool changed = (instance->current_cmd.set_state != SRV_SIGNAL_STATE_BLINK_CODE)
        || (instance->current_cmd.blink_cycle_ms != cmd->blink_cycle_ms)
        || (instance->current_cmd.blink_wait_ms != cmd->blink_wait_ms)
        || (instance->current_cmd.blink_code_counts != cmd->blink_code_counts);
    if (!changed)
        return SRV_SIGNAL_OK;

    if (fsm_current_state(&instance->fsm) == SRV_SIGNAL_STATE_BLINK_CODE) {
        if (instance->blink_sw_on) {
            memcpy(&instance->current_cmd, cmd, sizeof(srv_signal_cmd_t));
            instance->current_cmd.set_state = SRV_SIGNAL_STATE_BLINK_CODE;
            instance->pending_blink_update = true;
            return msg_fifo_push(&instance->cmd_fifo, &instance->current_cmd)
                ? SRV_SIGNAL_OK
                : SRV_SIGNAL_ERROR_INTERNAL;
        }
        memcpy(&instance->current_cmd, cmd, sizeof(srv_signal_cmd_t));
        instance->pending_blink_update = false;
        instance->current_blink_code_counts = 0;
        instance->blink_code_phase = SRV_SIGNAL_BLINK_PHASE_INTERVAL;
        instance->interval_start_time = s_signal_get_time ? s_signal_get_time() : 0;
        instance->blink_sw_on = false;
        signal_phys_write(instance, 0);
        return SRV_SIGNAL_OK;
    }

    memcpy(&instance->current_cmd, cmd, sizeof(srv_signal_cmd_t));
    instance->pending_blink_update = false;
    return SRV_SIGNAL_OK;
}

srv_signal_blink_phase_t srv_signal_get_blink_phase(srv_signal_handle_t* instance)
{
    return instance ? (srv_signal_blink_phase_t)instance->blink_code_phase
                    : SRV_SIGNAL_BLINK_PHASE_INTERVAL;
}

void srv_signal_set_callbacks(srv_signal_handle_t* instance, srv_signal_state_change_cb_t state_cb,
    srv_signal_blink_phase_cb_t blink_phase_cb,
    srv_signal_edge_cb_t edge_cb, void* user_data)
{
    if (instance == NULL)
        return;
    instance->state_change_cb = (void*)state_cb;
    instance->blink_phase_cb = (void*)blink_phase_cb;
    instance->edge_cb = (void*)edge_cb;
    instance->callback_user_data = user_data;
}

void srv_signal_task_refresh(void)
{
    if (!s_signal_initialized || clist_empty(&s_signal_head))
        return;

    srv_signal_handle_t* h;
    clist_for_each_entry(h, &s_signal_head, node)
    {
        if (!h->initialized)
            continue;
        signal_process_cmds(h);
        fsm_step(&h->fsm);

        /* 检测闪烁阶段变化并触发回调 */
        if (h->blink_phase_cb && h->blink_code_phase != h->blink_code_phase_last) {
            ((srv_signal_blink_phase_cb_t)h->blink_phase_cb)(
                h, h->blink_code_phase, h->callback_user_data);
            h->blink_code_phase_last = h->blink_code_phase;
        }
    }
}
