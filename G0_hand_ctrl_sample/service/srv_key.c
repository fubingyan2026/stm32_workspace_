/**
 * @file    srv_key.c
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-12
 * @brief   按键服务实现（基于 key_base 中间件，KEY1/KEY2 事件）
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_key.h"

#include "drv_key.h"
#include "drv_systick.h"
#include "log.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_KEY_LOG_ENABLE 0

#if SRV_KEY_LOG_ENABLE
#define SRV_KEY_LOG_E(...) LOG_E("srv_key", __VA_ARGS__)
#define SRV_KEY_LOG_W(...) LOG_W("srv_key", __VA_ARGS__)
#define SRV_KEY_LOG_I(...) LOG_I("srv_key", __VA_ARGS__)
#define SRV_KEY_LOG_D(...) LOG_D("srv_key", __VA_ARGS__)
#else
#define SRV_KEY_LOG_E(...) ((void)0)
#define SRV_KEY_LOG_W(...) ((void)0)
#define SRV_KEY_LOG_I(...) ((void)0)
#define SRV_KEY_LOG_D(...) ((void)0)
#endif

/* Private constants ---------------------------------------------------------*/

/** @brief 长按判定超时窗口 (ms)，低于 key_base 内部下限 500ms 会退化为 500ms */
#define SRV_KEY_LONG_PRESS_MS (1000U)

/** @brief 连击判定超时窗口 (ms)：单击/双击/三连击判定窗口 */
#define SRV_KEY_MULTI_CLICK_MS (300U)

/** @brief 按键事件名称表（来自 key_base.h） */
static const char* const s_event_names[] = KEY_BASE_EVENT_NAME_TABLE;

/* Private types -------------------------------------------------------------*/

typedef struct {
    const char* name;
    drv_key_channel_t ch;
    key_base_context_t ctx;
} srv_key_item_t;

/* Private variables ---------------------------------------------------------*/

static srv_key_item_t s_keys[DRV_KEY_CH_NUM] = {
    [DRV_KEY_CH_1] = { .name = "key1", .ch = DRV_KEY_CH_1 },
    [DRV_KEY_CH_2] = { .name = "key2", .ch = DRV_KEY_CH_2 },
};

/** @brief 上层业务事件回调 */
static srv_key_event_cb_t s_event_cb;

/** @brief 初始化标志 */
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static uint8_t srv_key_read_pin_1(void);

static uint8_t srv_key_read_pin_2(void);

static void srv_key_event_callback(key_base_event_t event, const void* context);

/* Exported functions --------------------------------------------------------*/

void srv_key_init(void)
{
    if (s_initialized) {
        return;
    }

    static const key_base_read_pin_cb_t read_pin_cbs[DRV_KEY_CH_NUM] = {
        [DRV_KEY_CH_1] = srv_key_read_pin_1,
        [DRV_KEY_CH_2] = srv_key_read_pin_2,
    };

    for (uint32_t i = 0; i < DRV_KEY_CH_NUM; i++) {
        const key_base_config_t cfg = {
            .name = s_keys[i].name,
            .event_callback = srv_key_event_callback,
            .read_pin_cb = read_pin_cbs[i],
            .get_time_cb = millis,
            .long_press_time_ms = SRV_KEY_LONG_PRESS_MS,
            .multi_click_time_ms = SRV_KEY_MULTI_CLICK_MS,
        };
        const key_base_error_t err = key_base_register_static(&cfg, &s_keys[i].ctx);
        if (err != KEY_BASE_OK) {
            SRV_KEY_LOG_E("按键 %s 注册失败: %d", s_keys[i].name, (int)err);
        }
    }

    s_initialized = true;
    SRV_KEY_LOG_I("按键服务初始化完成 (%u 个按键)", (unsigned)DRV_KEY_CH_NUM);
}

void srv_key_register_event_cb(srv_key_event_cb_t callback)
{
    s_event_cb = callback;
}

/* Private functions ---------------------------------------------------------*/

static uint8_t srv_key_read_pin_1(void)
{
    return drv_key_is_pressed(DRV_KEY_CH_1) ? KEY_BASE_PIN_STATE_PRESS
                                            : KEY_BASE_PIN_STATE_RELEASE;
}

static uint8_t srv_key_read_pin_2(void)
{
    return drv_key_is_pressed(DRV_KEY_CH_2) ? KEY_BASE_PIN_STATE_PRESS
                                            : KEY_BASE_PIN_STATE_RELEASE;
}

/**
 * @brief key_base 事件回调（主循环 sw_timer 上下文）
 * @param event   按键事件
 * @param context 指向 key_base_context_t
 */
static void srv_key_event_callback(key_base_event_t event, const void* context)
{
    if (context == NULL) {
        return;
    }

    const key_base_context_t* kb_ctx = (const key_base_context_t*)context;
    const char* name = (kb_ctx->config.name != NULL) ? kb_ctx->config.name : "?";

    const char* event_name = (event < KEY_BASE_EVENT_MAX)
        ? s_event_names[event]
        : "UNKNOWN";

    SRV_KEY_LOG_I("按键 %s: %s", name, event_name);

    if (s_event_cb) {
        s_event_cb(name, event);
    }
}
