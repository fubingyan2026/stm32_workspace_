/**
 * @file    app_main.c
 * @brief   Bootloader 主入口实现
 *
 * 参考 stm32_g474_boot/tasks/app_main.c 结构：
 *   delay_init → boot_task_try_boot_app()（决策/跳转）→ boot_task_init()+轮询。
 */

/* Includes ------------------------------------------------------------------*/
#include "app_main.h"

#include "boot_task.h"
#include "drv_systick.h"
#include "log.h"
#include "log_task.h"

void app_main(void)
{
    /* 系统节拍（延时/时间戳，基于 HAL tick） */
    delay_init();

    /* 日志任务（log_init + USART1 TX DMA 后端）——先于任何 LOG_* 输出 */
    log_task_init();
    LOG_I("app_main", "==== E1_CTU_BOOT start ====");

    /* 启动决策：有有效 App 则跳转（不会返回），否则进入 bootloader 升级模式 */
    if (boot_task_try_boot_app()) {
        /* 已跳转，不会到达这里 */
        for (;;) {
        }
    }

    boot_task_init();

    for (;;) {
        log_task_poll();
        boot_task_poll();
    }
}
