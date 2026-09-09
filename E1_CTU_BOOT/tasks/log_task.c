/**
 * @file    log_task.c
 * @brief   日志输出任务实现（本地副本，E1_CTU_BOOT）
 *
 * 参考兄弟工程（E1_SLAVER_POWER_CTU/E1_MASTER_POWER_CTU tasks/log_task.c）
 * 仅保留 UART（USART1 TX DMA）后端，去掉 SEGGER RTT 以控制 Boot Flash/依赖。
 * Boot 主循环每周期调用 log_task_poll() 代替兄弟工程的 sw_timer 驱动。
 */

/* Includes ------------------------------------------------------------------*/
#include "log_task.h"

#include <string.h>

#include "drv_log_uart.h"
#include "drv_systick.h"
#include "log.h"

/* Private constants ---------------------------------------------------------*/

#define LOG_TASK_TX_BUF_SIZE (128U)

/* Private variables ---------------------------------------------------------*/

static uint8_t s_tx_buf[LOG_TASK_TX_BUF_SIZE];
static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void log_task_drain_tx(void);

/* Exported functions --------------------------------------------------------*/

void log_task_init(void)
{
    log_config_t log_cfg = {
        .name = "E1_CTU_BOOT",
        .get_timestamp_cb = millis,
    };
    log_init(&log_cfg);
    log_set_level(LOG_LEVEL_DEBUG);

    /* UART 后端驱动初始化（本任务是 drv_log_uart 的唯一消费者；
       置于 log_init 之后，避免其内部初始化日志被静默丢弃） */
    (void)drv_log_uart_init();

    s_initialized = true;
}

void log_task_poll(void)
{
    if (!s_initialized) {
        return;
    }
    log_task_drain_tx();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 将 log FIFO 最新一段输出到 UART 后端（主循环周期调用）
 */
static void log_task_drain_tx(void)
{
    uint32_t log_len = log_tx_len();
    if (log_len == 0U) {
        return;
    }
    if (log_len > sizeof(s_tx_buf)) {
        log_len = sizeof(s_tx_buf);
    }

    if (!drv_log_uart_is_tx_busy()) {
        const uint32_t actual = log_tx_get(s_tx_buf, log_len);
        if (actual > 0U) {
            drv_log_uart_send(s_tx_buf, actual);
        }
    }
}
