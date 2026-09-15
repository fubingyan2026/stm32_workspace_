//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_cli.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   CTU SDK 命令行工具。
 *
 * 用法示例：
 *   ctu_cli ports
 *   ctu_cli -p /dev/ttyUSB0 scan
 *   ctu_cli -p /dev/ttyUSB0 status --device master
 *   ctu_cli -p /dev/ttyUSB0 output --on 24v,12v --duty 500 --preserve
 *   ctu_cli -p /dev/ttyUSB0 upgrade --device slaver --file fw.bin
 *   ctu_cli -p /dev/ttyUSB0 monitor --interval 0.5
 */

/* Includes ------------------------------------------------------------------*/

#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE /* 启用 strtok_r / nanosleep / strtod */
#endif

#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ctu/ctu.h"

/* Private constants ---------------------------------------------------------*/

#define CTU_CLI_MAX_PORTS 32U /**< 端口枚举上限 */
#define CTU_CLI_INVALID_INT (-1) /**< 整型参数未指定标记 */

/* Private types -------------------------------------------------------------*/

/**
 * @brief 串口名收集列表
 */
typedef struct {
    char names[CTU_CLI_MAX_PORTS][CTU_PORT_NAME_MAX]; /**< 串口名副本 */
    size_t count;                                     /**< 已收集数量 */
} ctu_cli_port_list_t;

/**
 * @brief 命令行参数集合
 */
typedef struct {
    const char* port;      /**< 串口设备 */
    uint32_t baud;         /**< 波特率 */
    bool verbose;          /**< 是否打印收发帧 */
    const char* command;   /**< 子命令 */
    const char* device;    /**< 设备名 */
    const char* file;      /**< 固件路径 */
    int duty;              /**< --duty */
    int mask;              /**< --mask */
    const char* on;        /**< --on 列表 */
    const char* off;       /**< --off 列表 */
    bool preserve;         /**< --preserve */
    double interval;       /**< --interval（秒） */
    double duration;       /**< --duration（秒，0=不限） */
} ctu_cli_args_t;

/* Private variables ---------------------------------------------------------*/

static volatile sig_atomic_t s_ctu_cli_stop = 0;

/* Private function prototypes -----------------------------------------------*/

static void ctu_cli_usage(void);
static void ctu_cli_on_sigint(int signum);
static int ctu_cli_parse(int argc, char** argv, ctu_cli_args_t* args);
static bool ctu_cli_parse_device(const char* text, ctu_device_t* device);
static void ctu_cli_log(const char* level, const char* text, void* user);
static void ctu_cli_phase(const char* text, void* user);
static void ctu_cli_progress(float fraction, void* user);
static void ctu_cli_sleep_ms(uint32_t delay_ms);
static int ctu_cli_run(ctu_cli_args_t* args);
static int ctu_cli_cmd_ports(void);
static int ctu_cli_cmd_scan(ctu_client_t* client);
static int ctu_cli_cmd_status(ctu_client_t* client, ctu_device_t device);
static int ctu_cli_cmd_volt(ctu_client_t* client, ctu_device_t device);
static int ctu_cli_cmd_temp(ctu_client_t* client, ctu_device_t device);
static int ctu_cli_cmd_info(ctu_client_t* client, ctu_device_t device);
static int ctu_cli_cmd_buzzer(ctu_client_t* client, int duty);
static int ctu_cli_cmd_output(ctu_client_t* client, ctu_cli_args_t* args);
static int ctu_cli_cmd_clear_latch(ctu_client_t* client, ctu_device_t device);
static int ctu_cli_cmd_request_upgrade(ctu_client_t* client, ctu_device_t device);
static int ctu_cli_cmd_upgrade(ctu_client_t* client, ctu_device_t device,
                               const char* filepath);
static int ctu_cli_cmd_monitor(ctu_cli_args_t* args);
static bool ctu_cli_channel_bit(const char* name, uint8_t* bit);
static uint8_t ctu_cli_parse_channels(const char* list, uint8_t base, bool set);
static void ctu_cli_collect_port(const char* name, void* user);
static const char* ctu_cli_device_label(ctu_device_t device);

/* Exported functions --------------------------------------------------------*/

int main(int argc, char** argv)
{
    ctu_cli_args_t args;

    memset(&args, 0, sizeof(args));
    args.baud = CTU_DEFAULT_BAUD;
    args.duty = CTU_CLI_INVALID_INT;
    args.mask = CTU_CLI_INVALID_INT;
    args.interval = 0.5;

    int parse_result = ctu_cli_parse(argc, argv, &args);

    if (parse_result == 1) {
        return 0; /* 已打印帮助 */
    }
    if (parse_result != 0) {
        return 2;
    }
    if (args.command == NULL) {
        ctu_cli_usage();
        return 2;
    }

    (void)signal(SIGINT, ctu_cli_on_sigint);

    return ctu_cli_run(&args);
}

/**
 * @brief 打印用法
 */
static void ctu_cli_usage(void)
{
    (void)printf(
        "ctu_cli - E1 CTU 电源板 RS485 SDK 命令行工具 (SDK %s)\n"
        "\n"
        "用法: ctu_cli [-p PORT] [-b BAUD] [-v] <命令> [选项]\n"
        "\n"
        "命令:\n"
        "  ports                              列出可用串口\n"
        "  scan                               探测两板在线状态\n"
        "  status  --device M                 读系统状态 (0x01)\n"
        "  volt    --device M                 读电压 (0x02)\n"
        "  temp    --device M                 读温度 (0x03)\n"
        "  info    --device M                 读固件信息 (0x07)\n"
        "  buzzer  --duty N                   主控蜂鸣器 0-50%% (0x04)\n"
        "  output  [--mask 0xNN] [--on a,b] [--off a,b] [--duty N] [--preserve]\n"
        "  clear-latch --device M             清除故障锁存 (0x05)\n"
        "  request-upgrade --device M         升级请求 (0x06)\n"
        "  upgrade --device M --file FW.bin   固件升级\n"
        "  monitor [--device M] [--interval S] [--duration S]\n"
        "\n"
        "设备: master / slaver（也接受 0x01 / 0x02 / 1 / 2 / 主 / 副）\n"
        "输出通道: 24v, 12v, lsd1, lsd2\n",
        ctu_version_string());
}

/**
 * @brief SIGINT 处理：置位停止标志
 */
static void ctu_cli_on_sigint(int signum)
{
    (void)signum;
    s_ctu_cli_stop = 1;
}

/**
 * @brief 解析命令行
 * @return 0 成功；非 0 表示参数错误
 */
static int ctu_cli_parse(int argc, char** argv, ctu_cli_args_t* args)
{
    for (int i = 1; i < argc; i++) {
        const char* token = argv[i];

        if ((strcmp(token, "-h") == 0) || (strcmp(token, "--help") == 0)) {
            ctu_cli_usage();
            return 1;
        } else if ((strcmp(token, "-p") == 0) || (strcmp(token, "--port") == 0)) {
            if (++i >= argc) {
                return -1;
            }
            args->port = argv[i];
        } else if ((strcmp(token, "-b") == 0) || (strcmp(token, "--baud") == 0)) {
            if (++i >= argc) {
                return -1;
            }
            args->baud = (uint32_t)strtoul(argv[i], NULL, 10);
        } else if ((strcmp(token, "-v") == 0) || (strcmp(token, "--verbose") == 0)) {
            args->verbose = true;
        } else if (strcmp(token, "--device") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->device = argv[i];
        } else if (strcmp(token, "--file") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->file = argv[i];
        } else if (strcmp(token, "--duty") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->duty = (int)strtol(argv[i], NULL, 10);
        } else if (strcmp(token, "--mask") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->mask = (int)strtol(argv[i], NULL, 0);
        } else if (strcmp(token, "--on") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->on = argv[i];
        } else if (strcmp(token, "--off") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->off = argv[i];
        } else if (strcmp(token, "--preserve") == 0) {
            args->preserve = true;
        } else if (strcmp(token, "--interval") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->interval = strtod(argv[i], NULL);
        } else if (strcmp(token, "--duration") == 0) {
            if (++i >= argc) {
                return -1;
            }
            args->duration = strtod(argv[i], NULL);
        } else if (token[0] == '-') {
            (void)fprintf(stderr, "未知选项: %s\n", token);
            return -1;
        } else if (args->command == NULL) {
            args->command = token;
        } else {
            (void)fprintf(stderr, "多余的参数: %s\n", token);
            return -1;
        }
    }

    return 0;
}

/**
 * @brief 解析设备名
 */
static bool ctu_cli_parse_device(const char* text, ctu_device_t* device)
{
    if ((text == NULL) || (device == NULL)) {
        return false;
    }
    if ((strcmp(text, "master") == 0) || (strcmp(text, "MASTER") == 0)
        || (strcmp(text, "0x01") == 0) || (strcmp(text, "1") == 0)
        || (strcmp(text, "主") == 0)) {
        *device = CTU_DEVICE_MASTER;
        return true;
    }
    if ((strcmp(text, "slaver") == 0) || (strcmp(text, "SLAVER") == 0)
        || (strcmp(text, "0x02") == 0) || (strcmp(text, "2") == 0)
        || (strcmp(text, "副") == 0)) {
        *device = CTU_DEVICE_SLAVER;
        return true;
    }

    return false;
}

/**
 * @brief 设备中文标签
 */
static const char* ctu_cli_device_label(ctu_device_t device)
{
    return (device == CTU_DEVICE_MASTER) ? "E1_MASTER (0x01)" : "E1_SLAVER (0x02)";
}

/**
 * @brief 日志回调：输出到 stderr
 */
static void ctu_cli_log(const char* level, const char* text, void* user)
{
    bool verbose = (user != NULL) && (*(bool*)user);

    if ((strcmp(level, "tx") == 0) || (strcmp(level, "rx") == 0)) {
        if (verbose) {
            (void)fprintf(stderr, "  [%s] %s\n", level, text);
        }
        return;
    }
    (void)fprintf(stderr, "  [%s] %s\n", level, text);
}

/**
 * @brief 阶段回调
 */
static void ctu_cli_phase(const char* text, void* user)
{
    (void)user;
    (void)fprintf(stderr, "  · %s\n", text);
}

/**
 * @brief 进度回调
 */
static void ctu_cli_progress(float fraction, void* user)
{
    (void)user;
    (void)fprintf(stderr, "\r  进度 %5.1f%%", (double)(fraction * 100.0f));
}

/**
 * @brief 毫秒延时
 */
static void ctu_cli_sleep_ms(uint32_t delay_ms)
{
    struct timespec request;

    request.tv_sec = (time_t)(delay_ms / 1000U);
    request.tv_nsec = (long)((delay_ms % 1000U) * 1000000U);
    while ((nanosleep(&request, &request) != 0) && (errno == EINTR)) {
        if (s_ctu_cli_stop != 0) {
            return;
        }
    }
}

/**
 * @brief 收集串口名
 */
static void ctu_cli_collect_port(const char* name, void* user)
{
    ctu_cli_port_list_t* list = (ctu_cli_port_list_t*)user;

    /* 必须复制：globfree 之后原始指针会失效 */
    if ((list != NULL) && (list->count < CTU_CLI_MAX_PORTS) && (name != NULL)) {
        (void)snprintf(list->names[list->count], CTU_PORT_NAME_MAX, "%s", name);
        list->count++;
    }
}

/**
 * @brief 输出通道名转位
 */
static bool ctu_cli_channel_bit(const char* name, uint8_t* bit)
{
    if ((strcmp(name, "24v") == 0) || (strcmp(name, "24V") == 0)) {
        *bit = 0U;
        return true;
    }
    if ((strcmp(name, "12v") == 0) || (strcmp(name, "12V") == 0)) {
        *bit = 1U;
        return true;
    }
    if ((strcmp(name, "lsd1") == 0) || (strcmp(name, "LSD1") == 0)) {
        *bit = 2U;
        return true;
    }
    if ((strcmp(name, "lsd2") == 0) || (strcmp(name, "LSD2") == 0)) {
        *bit = 3U;
        return true;
    }

    return false;
}

/**
 * @brief 解析逗号分隔的通道列表，叠加到 base 位域
 */
static uint8_t ctu_cli_parse_channels(const char* list, uint8_t base, bool set)
{
    char buffer[128];
    char* token;
    char* save = NULL;
    uint8_t mask = base;

    if (strlen(list) >= sizeof(buffer)) {
        (void)fprintf(stderr, "通道列表过长\n");
        return mask;
    }
    (void)strncpy(buffer, list, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';

    token = strtok_r(buffer, ",", &save);
    while (token != NULL) {
        uint8_t bit = 0U;
        if (ctu_cli_channel_bit(token, &bit)) {
            if (set) {
                mask |= (uint8_t)(1U << bit);
            } else {
                mask &= (uint8_t)~(1U << bit);
            }
        } else {
            (void)fprintf(stderr, "未知输出通道: %s\n", token);
        }
        token = strtok_r(NULL, ",", &save);
    }

    return mask;
}

/**
 * @brief 分发子命令
 */
static int ctu_cli_run(ctu_cli_args_t* args)
{
    ctu_client_t client;
    ctu_client_config_t config;
    ctu_device_t device = CTU_DEVICE_MASTER;
    int status = 0;

    if (strcmp(args->command, "ports") == 0) {
        return ctu_cli_cmd_ports();
    }
    if (strcmp(args->command, "monitor") == 0) {
        return ctu_cli_cmd_monitor(args);
    }

    if ((args->port == NULL) || (args->port[0] == '\0')) {
        (void)fprintf(stderr, "请用 -p/--port 指定串口（可用 `ports` 列出）\n");
        return 2;
    }
    if (!ctu_transport_baud_is_supported(args->baud)) {
        (void)fprintf(stderr, "不支持的波特率: %u\n", (unsigned)args->baud);
        return 2;
    }
    if ((args->device != NULL) && !ctu_cli_parse_device(args->device, &device)) {
        (void)fprintf(stderr, "未知设备: %s\n", args->device);
        return 2;
    }

    memset(&config, 0, sizeof(config));
    config.timeout_ms = CTU_DEFAULT_TIMEOUT_MS;
    config.log_cb = ctu_cli_log;
    config.user = &args->verbose;
    config.boot.log_cb = ctu_cli_log;
    config.boot.phase_cb = ctu_cli_phase;
    config.boot.progress_cb = ctu_cli_progress;
    config.boot.user = &args->verbose;

    if (ctu_client_init(&client, &config) != CTU_OK) {
        (void)fprintf(stderr, "客户端初始化失败\n");
        return 1;
    }
    if (ctu_client_open(&client, args->port, args->baud) != CTU_OK) {
        (void)fprintf(stderr, "无法打开串口 %s: %s\n", args->port,
                      ctu_client_last_error_text(&client));
        ctu_client_deinit(&client);
        return 1;
    }
    (void)printf("已打开 %s @ %u\n", ctu_client_port_name(&client),
                 (unsigned)args->baud);

    if (strcmp(args->command, "scan") == 0) {
        status = ctu_cli_cmd_scan(&client);
    } else if (strcmp(args->command, "status") == 0) {
        status = ctu_cli_cmd_status(&client, device);
    } else if (strcmp(args->command, "volt") == 0) {
        status = ctu_cli_cmd_volt(&client, device);
    } else if (strcmp(args->command, "temp") == 0) {
        status = ctu_cli_cmd_temp(&client, device);
    } else if (strcmp(args->command, "info") == 0) {
        status = ctu_cli_cmd_info(&client, device);
    } else if (strcmp(args->command, "buzzer") == 0) {
        status = ctu_cli_cmd_buzzer(&client, (args->duty < 0) ? 0 : args->duty);
    } else if (strcmp(args->command, "output") == 0) {
        status = ctu_cli_cmd_output(&client, args);
    } else if (strcmp(args->command, "clear-latch") == 0) {
        status = ctu_cli_cmd_clear_latch(&client, device);
    } else if (strcmp(args->command, "request-upgrade") == 0) {
        status = ctu_cli_cmd_request_upgrade(&client, device);
    } else if (strcmp(args->command, "upgrade") == 0) {
        if (args->file == NULL) {
            (void)fprintf(stderr, "upgrade 需要 --file <固件.bin>\n");
            status = 2;
        } else {
            status = ctu_cli_cmd_upgrade(&client, device, args->file);
        }
    } else {
        (void)fprintf(stderr, "未知命令: %s\n", args->command);
        ctu_cli_usage();
        status = 2;
    }

    ctu_client_deinit(&client);

    return status;
}

/* ---- 子命令实现 ---- */

static int ctu_cli_cmd_ports(void)
{
    ctu_cli_port_list_t list;

    memset(&list, 0, sizeof(list));
    (void)ctu_transport_foreach_port(ctu_cli_collect_port, &list);

    if (list.count == 0U) {
        (void)printf("(未发现串口)\n");
        return 0;
    }
    for (size_t i = 0U; i < list.count; i++) {
        (void)printf("%s\n", list.names[i]);
    }

    return 0;
}

static int ctu_cli_cmd_scan(ctu_client_t* client)
{
    for (uint8_t index = 0U; index < 2U; index++) {
        ctu_device_t device = (index == 0U) ? CTU_DEVICE_MASTER : CTU_DEVICE_SLAVER;
        bool online = false;

        (void)ctu_client_probe(client, device, &online);
        (void)printf("%-10s %s\n", ctu_device_name(device), online ? "在线" : "离线");
    }

    return 0;
}

static int ctu_cli_cmd_status(ctu_client_t* client, ctu_device_t device)
{
    ctu_error_t result;

    if (device == CTU_DEVICE_MASTER) {
        ctu_master_status_t status;

        result = ctu_client_read_master_status(client, &status);
        if (result != CTU_OK) {
            (void)fprintf(stderr, "读取失败: %s\n", ctu_error_str(result));
            return 1;
        }
        (void)printf("%s: %s\n", ctu_cli_device_label(device),
                     ctu_master_status_has_fault(&status) ? "存在异常" : "正常");
        (void)printf("  急停=%d 12V异常=%d 24V异常=%d VIN_DC-DC异常=%d AUX异常=%d "
                     "MOTOR异常=%d\n",
                     status.estop, status.rail_12v_fault, status.rail_24v_fault,
                     status.vin_dcdc_fault, status.aux_fault, status.motor_fault);
        (void)printf("  风扇0异常=%d 风扇1异常=%d NTC1断开=%d NTC2断开=%d\n",
                     status.fan0_fault, status.fan1_fault,
                     status.ntc1_disconnected, status.ntc2_disconnected);
    } else {
        ctu_slaver_status_t status;

        result = ctu_client_read_slaver_status(client, &status);
        if (result != CTU_OK) {
            (void)fprintf(stderr, "读取失败: %s\n", ctu_error_str(result));
            return 1;
        }
        (void)printf("%s: %s\n", ctu_cli_device_label(device),
                     ctu_slaver_status_has_fault(&status) ? "存在异常" : "正常");
        (void)printf("  输出位域=0x%02X  24V=%d 12V_ISO=%d LSD1=%d LSD2=%d\n",
                     ctu_slaver_status_output_mask(&status), status.out_24v,
                     status.out_12v, status.out_lsd1, status.out_lsd2);
        (void)printf("  故障: 24V=%d 12V=%d LSD1=%d LSD2=%d AUX=%d MOTOR=%d "
                     "锁存=%d\n",
                     status.fault_24v, status.fault_12v, status.fault_lsd1,
                     status.fault_lsd2, status.fault_aux, status.fault_motor,
                     status.latch_active);
    }

    return 0;
}

static int ctu_cli_cmd_volt(ctu_client_t* client, ctu_device_t device)
{
    if (device == CTU_DEVICE_MASTER) {
        ctu_master_voltage_t voltage;

        if (ctu_client_read_master_voltage(client, &voltage) != CTU_OK) {
            (void)fprintf(stderr, "读取失败\n");
            return 1;
        }
        (void)printf("VIN=%u mV  VIN_DC-DC=%u mV\n", voltage.vin_mv,
                     voltage.vin_dcdc_mv);
    } else {
        ctu_slaver_voltage_t voltage;

        if (ctu_client_read_slaver_voltage(client, &voltage) != CTU_OK) {
            (void)fprintf(stderr, "读取失败\n");
            return 1;
        }
        (void)printf("AUX=%u mV  MOTOR=%u mV  LSD1=%u mV  LSD2=%u mV\n",
                     voltage.aux_mv, voltage.motor_mv, voltage.lsd1_mv,
                     voltage.lsd2_mv);
    }

    return 0;
}

static int ctu_cli_cmd_temp(ctu_client_t* client, ctu_device_t device)
{
    if (device == CTU_DEVICE_MASTER) {
        ctu_master_temp_t temp;

        if (ctu_client_read_master_temp(client, &temp) != CTU_OK) {
            (void)fprintf(stderr, "读取失败\n");
            return 1;
        }
        (void)printf("NTC1=%.2f °C  NTC2=%.2f °C  MCU=%.2f °C\n",
                     (double)temp.ntc1_c, (double)temp.ntc2_c, (double)temp.mcu_c);
    } else {
        ctu_slaver_temp_t temp;

        if (ctu_client_read_slaver_temp(client, &temp) != CTU_OK) {
            (void)fprintf(stderr, "读取失败\n");
            return 1;
        }
        (void)printf("MCU=%.2f °C  VDDA=%u mV\n", (double)temp.mcu_c, temp.vdda_mv);
    }

    return 0;
}

static int ctu_cli_cmd_info(ctu_client_t* client, ctu_device_t device)
{
    ctu_fw_info_t info;

    if (ctu_client_read_info(client, device, &info) != CTU_OK) {
        (void)fprintf(stderr, "读取失败\n");
        return 1;
    }
    (void)printf("App 版本   : v%u\n", info.app_version);
    (void)printf("Boot 版本  : v%u\n", info.meta_version);
    (void)printf("固件大小   : %u B\n", (unsigned)info.fw_size);
    (void)printf("固件校验和 : 0x%08X\n", (unsigned)info.fw_checksum);
    (void)printf("上电次数   : %u\n", info.reboot_counts);
    (void)printf("标志       : 0x%02X (metadata有效=%d 升级待处理=%d 升级完成=%d)\n",
                 info.flags, (info.flags & CTU_FW_FLAG_META_VALID) != 0,
                 (info.flags & CTU_FW_FLAG_UPGRADE_REQ) != 0,
                 (info.flags & CTU_FW_FLAG_UPGRADE_DONE) != 0);

    return 0;
}

static int ctu_cli_cmd_buzzer(ctu_client_t* client, int duty)
{
    ctu_ack_t ack;

    if (ctu_client_set_buzzer_duty(client, (uint8_t)duty, &ack) != CTU_OK) {
        (void)fprintf(stderr, "发送失败\n");
        return 1;
    }
    (void)printf("蜂鸣器占空比 %d%% 已发送（%.1f ms）\n", duty,
                 (double)ack.elapsed_ms);

    return 0;
}

static int ctu_cli_cmd_output(ctu_client_t* client, ctu_cli_args_t* args)
{
    uint8_t mask = 0U;
    ctu_ack_t ack;

    if (args->mask >= 0) {
        mask = (uint8_t)args->mask;
    } else if (args->preserve) {
        ctu_slaver_status_t status;

        if (ctu_client_read_slaver_status(client, &status) != CTU_OK) {
            (void)fprintf(stderr, "读取当前输出失败\n");
            return 1;
        }
        mask = ctu_slaver_status_output_mask(&status);
    }
    if (args->on != NULL) {
        mask = ctu_cli_parse_channels(args->on, mask, true);
    }
    if (args->off != NULL) {
        mask = ctu_cli_parse_channels(args->off, mask, false);
    }

    if (ctu_client_set_outputs(client, mask, (uint16_t)((args->duty < 0) ? 0
                                                        : args->duty),
                               &ack) != CTU_OK) {
        (void)fprintf(stderr, "发送失败\n");
        return 1;
    }
    (void)printf("输出位域 0x%02X，补光 %d 已发送（%.1f ms）\n", mask,
                 (args->duty < 0) ? 0 : args->duty, (double)ack.elapsed_ms);

    return 0;
}

static int ctu_cli_cmd_clear_latch(ctu_client_t* client, ctu_device_t device)
{
    ctu_ack_t ack;

    if (ctu_client_clear_fault_latch(client, device, &ack) != CTU_OK) {
        (void)fprintf(stderr, "发送失败\n");
        return 1;
    }
    (void)printf("%s 清除故障锁存已发送（%.1f ms）\n", ctu_device_name(device),
                 (double)ack.elapsed_ms);

    return 0;
}

static int ctu_cli_cmd_request_upgrade(ctu_client_t* client, ctu_device_t device)
{
    ctu_ack_t ack;

    if (ctu_client_request_upgrade(client, device, &ack) != CTU_OK) {
        (void)fprintf(stderr, "发送失败\n");
        return 1;
    }
    (void)printf("%s 升级请求已确认，板端将复位进入 Bootloader\n",
                 ctu_device_name(device));

    return 0;
}

static int ctu_cli_cmd_upgrade(ctu_client_t* client, ctu_device_t device,
                               const char* filepath)
{
    char reason[160];
    uint32_t size = 0U;
    uint32_t checksum = 0U;
    ctu_error_t result;

    result = ctu_boot_check_file(filepath, &size, &checksum, reason, sizeof(reason));
    (void)printf("固件 %s  %u B  校验和 0x%08X  %s\n", filepath, (unsigned)size,
                 (unsigned)checksum, (result == CTU_OK) ? "预检通过" : reason);
    if (result != CTU_OK) {
        if (result == CTU_ERROR_FILE) {
            char located[512];

            if (ctu_boot_locate_image(filepath, located, sizeof(located)) == CTU_OK) {
                (void)fprintf(stderr,
                              "提示: 已在其它目录找到同名固件，请改用该路径重试:\n"
                              "  ctu_cli -p %s upgrade --device %s --file %s\n",
                              ctu_client_port_name(client), ctu_device_name(device),
                              located);
            }
        }
        return 1;
    }

    result = ctu_client_upgrade(client, device, filepath);
    (void)fprintf(stderr, "\n");
    if (result != CTU_OK) {
        (void)fprintf(stderr, "升级失败: %s\n", ctu_error_str(result));
        return 1;
    }
    (void)printf("%s 升级完成：%s\n", ctu_device_name(device), filepath);

    return 0;
}

static int ctu_cli_cmd_monitor(ctu_cli_args_t* args)
{
    ctu_client_t client;
    ctu_client_config_t config;
    ctu_poller_t poller;
    ctu_poller_config_t poller_config;
    bool verbose = args->verbose;
    uint64_t start_ms;
    uint64_t duration_ms;
    struct timespec now;
    uint32_t interval_ms;

    if ((args->port == NULL) || (args->port[0] == '\0')) {
        (void)fprintf(stderr, "请用 -p/--port 指定串口\n");
        return 2;
    }

    memset(&config, 0, sizeof(config));
    config.timeout_ms = CTU_DEFAULT_TIMEOUT_MS;
    config.log_cb = ctu_cli_log;
    config.user = &verbose;
    if (ctu_client_init(&client, &config) != CTU_OK) {
        return 1;
    }
    if (ctu_client_open(&client, args->port, args->baud) != CTU_OK) {
        (void)fprintf(stderr, "无法打开串口 %s: %s\n", args->port,
                      ctu_client_last_error_text(&client));
        ctu_client_deinit(&client);
        return 1;
    }

    memset(&poller_config, 0, sizeof(poller_config));
    poller_config.device_count = 0U;
    if (args->device != NULL) {
        ctu_device_t device;
        if (!ctu_cli_parse_device(args->device, &device)) {
            (void)fprintf(stderr, "未知设备: %s\n", args->device);
            ctu_client_deinit(&client);
            return 2;
        }
        poller_config.devices[poller_config.device_count++] = device;
    }
    if (ctu_poller_init(&poller, &client, &poller_config) != CTU_OK) {
        ctu_client_deinit(&client);
        return 1;
    }

    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    start_ms = ((uint64_t)now.tv_sec * 1000U) + ((uint64_t)now.tv_nsec / 1000000U);
    duration_ms = (uint64_t)(args->duration * 1000.0);
    interval_ms = (args->interval > 0.0) ? (uint32_t)(args->interval * 1000.0) : 500U;

    (void)printf("开始轮询 %s @ %u（Ctrl+C 停止）\n", args->port,
                 (unsigned)args->baud);

    while (s_ctu_cli_stop == 0) {
        ctu_poll_stats_t stats;
        uint64_t elapsed;

        (void)ctu_poller_poll_once(&poller);
        (void)ctu_poller_get_stats(&poller, &stats);
        (void)printf("tx=%u rx=%u 丢包=%u (%.1f%%) fps=%.1f\n",
                     (unsigned)stats.tx, (unsigned)stats.rx, (unsigned)stats.loss,
                     (double)stats.loss_rate, (double)stats.fps);
        (void)fflush(stdout);

        (void)clock_gettime(CLOCK_MONOTONIC, &now);
        elapsed = (((uint64_t)now.tv_sec * 1000U)
                   + ((uint64_t)now.tv_nsec / 1000000U)) - start_ms;
        if ((duration_ms > 0U) && (elapsed >= duration_ms)) {
            break;
        }
        ctu_cli_sleep_ms(interval_ms);
    }

    (void)printf("已停止\n");
    ctu_client_deinit(&client);

    return 0;
}
