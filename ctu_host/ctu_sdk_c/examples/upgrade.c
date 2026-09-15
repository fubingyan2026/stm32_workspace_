//
// Created by E1 on 2026-09-15.
//

/**
 * @file    upgrade.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   示例：固件升级（含进度显示与 Ctrl+C 中止）。
 *
 * 用法（在 ctu_sdk_c 目录）：
 *   cmake -S . -B build && cmake --build build -j
 *   ./build/ctu_example_upgrade /dev/ttyUSB0 master fw.bin
 */

/* Includes ------------------------------------------------------------------*/
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ctu/ctu.h"

/* Private variables ---------------------------------------------------------*/

static ctu_client_t* s_active_client = NULL; /**< 供 SIGINT 中止升级（示例专用） */
static volatile sig_atomic_t s_cancel = 0;

/* Private function prototypes -----------------------------------------------*/

static void ctu_example_on_sigint(int signum);
static void ctu_example_log(const char* level, const char* text, void* user);
static void ctu_example_progress(float fraction, void* user);
static void ctu_example_phase(const char* phase, void* user);
static bool ctu_example_parse_device(const char* text, ctu_device_t* device);

/* Exported functions --------------------------------------------------------*/

int main(int argc, char** argv)
{
    ctu_client_t client;
    ctu_client_config_t config = { 0 };
    ctu_device_t device = CTU_DEVICE_MASTER;
    char reason[160];
    uint32_t size = 0U;
    uint32_t checksum = 0U;
    ctu_error_t result;

    if (argc < 4) {
        (void)fprintf(stderr,
                      "用法: %s <串口> <master|slaver> <固件.bin>\n"
                      "  <固件.bin> 为 App 分区镜像（非 Boot），需给出路径；\n"
                      "  由固件工程构建产生，例如：\n"
                      "    E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin\n"
                      "  例: %s /dev/ttyUSB0 master ../E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin\n",
                      argv[0], argv[0]);
        return 2;
    }
    if (!ctu_example_parse_device(argv[2], &device)) {
        (void)fprintf(stderr, "未知设备: %s（可选 master / slaver）\n", argv[2]);
        return 2;
    }

    /* 1. 预检 */
    result = ctu_boot_check_file(argv[3], &size, &checksum, reason, sizeof(reason));
    (void)printf("固件 %s  %u B  校验和 0x%08X  %s\n", argv[3], (unsigned)size,
                 (unsigned)checksum, (result == CTU_OK) ? "预检通过" : reason);
    if (result != CTU_OK) {
        if (result == CTU_ERROR_FILE) {
            /* 当前目录没有该文件时，帮忙在常见构建产物目录中找一下 */
            char located[512];

            if (ctu_boot_locate_image(argv[3], located, sizeof(located)) == CTU_OK) {
                (void)fprintf(stderr,
                              "提示: 已在其它目录找到同名固件，请改用该路径重试:\n"
                              "  %s %s %s %s\n",
                              argv[0], argv[1], argv[2], located);
            } else {
                (void)fprintf(stderr,
                              "提示: 当前目录没有该文件，请给出固件路径，例如\n"
                              "  %s %s %s ../../E1_MASTER_POWER_CTU/build/RelWithDebInfo/E1_MASTER_POWER_CTU.bin\n",
                              argv[0], argv[1], argv[2]);
            }
        }
        return 1;
    }

    /* 2. 打开串口 */
    config.timeout_ms = CTU_DEFAULT_TIMEOUT_MS;
    config.log_cb = ctu_example_log;
    config.boot.progress_cb = ctu_example_progress;
    config.boot.phase_cb = ctu_example_phase;
    if (ctu_client_init(&client, &config) != CTU_OK) {
        return 1;
    }
    if (ctu_client_open(&client, argv[1], CTU_DEFAULT_BAUD) != CTU_OK) {
        (void)fprintf(stderr, "无法打开串口 %s: %s\n", argv[1],
                      ctu_client_last_error_text(&client));
        ctu_client_deinit(&client);
        return 1;
    }

    /* 3. 升级（Ctrl+C → 请求中止，SDK 会发送 ABORT） */
    (void)signal(SIGINT, ctu_example_on_sigint);
    s_active_client = &client;

    result = ctu_client_upgrade(&client, device, argv[3]);

    s_active_client = NULL;
    (void)fprintf(stderr, "\n");

    if (result == CTU_ERROR_CANCELED) {
        (void)fprintf(stderr, "升级已中止\n");
    } else if (result != CTU_OK) {
        (void)fprintf(stderr, "升级失败: %s\n", ctu_error_str(result));
    } else {
        (void)printf("%s 升级完成：%s\n", ctu_device_name(device), argv[3]);
    }

    ctu_client_deinit(&client);

    return (result == CTU_OK) ? 0 : 1;
}

/* Private functions ---------------------------------------------------------*/

static void ctu_example_on_sigint(int signum)
{
    (void)signum;
    s_cancel = 1;
    if (s_active_client != NULL) {
        ctu_client_request_cancel(s_active_client);
    }
}

static void ctu_example_log(const char* level, const char* text, void* user)
{
    bool* verbose = (bool*)user;

    if ((verbose != NULL) && (*verbose)) {
        (void)fprintf(stderr, "  [%s] %s\n", level, text);
    }
}

static void ctu_example_progress(float fraction, void* user)
{
    (void)user;
    (void)fprintf(stderr, "\r进度 %5.1f%%", (double)(fraction * 100.0f));
}

static void ctu_example_phase(const char* phase, void* user)
{
    (void)user;
    (void)fprintf(stderr, "\n· %s", phase);
}

static bool ctu_example_parse_device(const char* text, ctu_device_t* device)
{
    if (strcmp(text, "master") == 0) {
        *device = CTU_DEVICE_MASTER;
        return true;
    }
    if (strcmp(text, "slaver") == 0) {
        *device = CTU_DEVICE_SLAVER;
        return true;
    }

    return false;
}
