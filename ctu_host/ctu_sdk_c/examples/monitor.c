//
// Created by E1 on 2026-09-15.
//

/**
 * @file    monitor.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   示例：周期轮询两板状态并打印异常与统计。
 *
 * 编译（在 ctu_sdk_c 目录）：
 *   cmake -S . -B build && cmake --build build -j
 *   ./build/ctu_example_monitor /dev/ttyUSB0
 */

/* Includes ------------------------------------------------------------------*/
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "ctu/ctu.h"

/* Private variables ---------------------------------------------------------*/

static volatile sig_atomic_t s_stop = 0;

/* Private function prototypes -----------------------------------------------*/

static void ctu_example_on_sigint(int signum);
static void ctu_example_log(const char* level, const char* text, void* user);
static void ctu_example_sample(const ctu_poll_sample_t* sample, void* user);
static void ctu_example_error(ctu_device_t device, uint8_t command,
                              ctu_error_t error, void* user);

/* Exported functions --------------------------------------------------------*/

int main(int argc, char** argv)
{
    ctu_client_t client;
    ctu_client_config_t config = { 0 };
    ctu_poller_t poller;
    ctu_poller_config_t poller_config = { 0 };
    const char* port;

    if (argc < 2) {
        (void)fprintf(stderr, "用法: %s <串口> [波特率]\n"
                              "  例: %s /dev/ttyUSB0 115200\n",
                      argv[0], argv[0]);
        return 2;
    }
    port = argv[1];

    config.timeout_ms = CTU_DEFAULT_TIMEOUT_MS;
    config.log_cb = ctu_example_log;
    if (ctu_client_init(&client, &config) != CTU_OK) {
        return 1;
    }
    if (ctu_client_open(&client, port, (argc > 2)
                        ? (uint32_t)strtoul(argv[2], NULL, 10) : CTU_DEFAULT_BAUD)
        != CTU_OK) {
        (void)fprintf(stderr, "无法打开串口 %s: %s\n", port,
                      ctu_client_last_error_text(&client));
        ctu_client_deinit(&client);
        return 1;
    }
    (void)printf("已连接 %s\n", ctu_client_port_name(&client));

    poller_config.sample_cb = ctu_example_sample;
    poller_config.error_cb = ctu_example_error;
    if (ctu_poller_init(&poller, &client, &poller_config) != CTU_OK) {
        ctu_client_deinit(&client);
        return 1;
    }

    (void)signal(SIGINT, ctu_example_on_sigint);
    while (s_stop == 0) {
        ctu_poll_stats_t stats;

        (void)ctu_poller_poll_once(&poller);
        (void)ctu_poller_get_stats(&poller, &stats);
        (void)printf("tx=%u rx=%u 丢包=%u (%.1f%%)\n", (unsigned)stats.tx,
                     (unsigned)stats.rx, (unsigned)stats.loss,
                     (double)stats.loss_rate);

        {
            struct timespec pause;

            pause.tv_sec = 0;
            pause.tv_nsec = 500000000L; /* 500 ms */
            (void)nanosleep(&pause, NULL);
        }
    }

    (void)printf("\n已停止\n");
    ctu_client_deinit(&client);

    return 0;
}

/* Private functions ---------------------------------------------------------*/

static void ctu_example_on_sigint(int signum)
{
    (void)signum;
    s_stop = 1;
}

static void ctu_example_log(const char* level, const char* text, void* user)
{
    (void)user;
    (void)printf("  [%s] %s\n", level, text);
}

static void ctu_example_sample(const ctu_poll_sample_t* sample, void* user)
{
    (void)user;
    if (sample == NULL) {
        return;
    }

    if (sample->device == CTU_DEVICE_MASTER) {
        if (sample->status_valid) {
            (void)printf("master  状态=%s", ctu_master_status_has_fault(
                             &sample->master_status) ? "异常" : "正常");
        }
        if (sample->voltage_valid) {
            (void)printf("  VIN=%u mV", sample->master_voltage.vin_mv);
        }
        if (sample->temp_valid) {
            (void)printf("  MCU=%.2f °C", (double)sample->master_temp.mcu_c);
        }
        (void)printf("\n");
    } else {
        if (sample->status_valid) {
            (void)printf("slaver  状态=%s  输出=0x%02X",
                         ctu_slaver_status_has_fault(&sample->slaver_status)
                             ? "异常" : "正常",
                         ctu_slaver_status_output_mask(&sample->slaver_status));
        }
        if (sample->voltage_valid) {
            (void)printf("  AUX=%u mV", sample->slaver_voltage.aux_mv);
        }
        if (sample->temp_valid) {
            (void)printf("  MCU=%.2f °C", (double)sample->slaver_temp.mcu_c);
        }
        (void)printf("\n");
    }
}

static void ctu_example_error(ctu_device_t device, uint8_t command,
                              ctu_error_t error, void* user)
{
    (void)user;
    (void)fprintf(stderr, "  ! %s cmd=0x%02X 失败: %s\n", ctu_device_name(device),
                  command, ctu_error_str(error));
}
