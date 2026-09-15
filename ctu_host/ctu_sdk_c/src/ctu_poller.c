//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_poller.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   周期轮询与统计实现。
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu_poller.h"

#include <string.h>
#include <time.h>

/* Private constants ---------------------------------------------------------*/

#define CTU_POLLER_WINDOW_MS 1000U /**< 接收速率统计窗口 */

/* Private function prototypes -----------------------------------------------*/

static uint64_t ctu_poller_now_ms(void);
static void ctu_poller_refresh_window(ctu_poller_t* poller);
static void ctu_poller_record_ok(ctu_poller_t* poller);
static void ctu_poller_record_error(ctu_poller_t* poller, ctu_device_t device,
                                    uint8_t command, ctu_error_t error);
static ctu_error_t ctu_poller_query_temp(ctu_poller_t* poller, ctu_device_t device,
                                         ctu_poll_sample_t* sample);

/* Exported functions --------------------------------------------------------*/

ctu_error_t ctu_poller_init(ctu_poller_t* poller, ctu_client_t* client,
                            const ctu_poller_config_t* config)
{
    if ((poller == NULL) || (client == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_client_is_initialized(client)) {
        return CTU_ERROR_UNINITIALIZED;
    }

    memset(poller, 0, sizeof(*poller));
    poller->client = client;

    if (config != NULL) {
        poller->config = *config;
    }
    if (poller->config.device_count == 0U) {
        poller->config.devices[0] = CTU_DEVICE_MASTER;
        poller->config.devices[1] = CTU_DEVICE_SLAVER;
        poller->config.device_count = 2U;
    }
    if (poller->config.device_count > 2U) {
        poller->config.device_count = 2U;
    }

    poller->window_start_ms = ctu_poller_now_ms();
    poller->initialized = true;

    return CTU_OK;
}

void ctu_poller_reset_stats(ctu_poller_t* poller)
{
    if (poller == NULL) {
        return;
    }

    memset(&poller->stats, 0, sizeof(poller->stats));
    poller->rx_in_window = 0U;
    poller->window_start_ms = ctu_poller_now_ms();
}

ctu_error_t ctu_poller_poll_once(ctu_poller_t* poller)
{
    if (poller == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!poller->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }

    for (size_t i = 0U; i < poller->config.device_count; i++) {
        (void)ctu_poller_poll_device(poller, poller->config.devices[i]);
    }

    return CTU_OK;
}

ctu_error_t ctu_poller_poll_device(ctu_poller_t* poller, ctu_device_t device)
{
    ctu_poll_sample_t sample;
    ctu_error_t result;

    if (poller == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!poller->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }
    if (!ctu_device_is_valid(device)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    memset(&sample, 0, sizeof(sample));
    sample.device = device;

    /* 1. 状态 */
    poller->stats.tx++;
    if (device == CTU_DEVICE_MASTER) {
        result = ctu_client_read_master_status(poller->client, &sample.master_status);
    } else {
        result = ctu_client_read_slaver_status(poller->client, &sample.slaver_status);
    }
    if (result != CTU_OK) {
        ctu_poller_record_error(poller, device, (uint8_t)CTU_CMD_READ_STATUS, result);
    } else {
        sample.status_valid = true;
        ctu_poller_record_ok(poller);
    }

    /* 2. 电压 */
    poller->stats.tx++;
    if (device == CTU_DEVICE_MASTER) {
        result = ctu_client_read_master_voltage(poller->client, &sample.master_voltage);
    } else {
        result = ctu_client_read_slaver_voltage(poller->client, &sample.slaver_voltage);
    }
    if (result != CTU_OK) {
        ctu_poller_record_error(poller, device, (uint8_t)CTU_CMD_READ_VOLT, result);
    } else {
        sample.voltage_valid = true;
        ctu_poller_record_ok(poller);
    }

    /* 3. 温度 */
    result = ctu_poller_query_temp(poller, device, &sample);
    if (result != CTU_OK) {
        ctu_poller_record_error(poller, device, (uint8_t)CTU_CMD_READ_TEMP, result);
    }

    if ((poller->config.sample_cb != NULL)
        && (sample.status_valid || sample.voltage_valid || sample.temp_valid)) {
        poller->config.sample_cb(&sample, poller->config.user);
    }

    return CTU_OK;
}

ctu_error_t ctu_poller_get_stats(ctu_poller_t* poller, ctu_poll_stats_t* out)
{
    if ((poller == NULL) || (out == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!poller->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }

    ctu_poller_refresh_window(poller);
    *out = poller->stats;

    return CTU_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 读取温度并记入采样
 */
static ctu_error_t ctu_poller_query_temp(ctu_poller_t* poller, ctu_device_t device,
                                         ctu_poll_sample_t* sample)
{
    ctu_error_t result;

    poller->stats.tx++;
    if (device == CTU_DEVICE_MASTER) {
        result = ctu_client_read_master_temp(poller->client, &sample->master_temp);
    } else {
        result = ctu_client_read_slaver_temp(poller->client, &sample->slaver_temp);
    }
    if (result == CTU_OK) {
        sample->temp_valid = true;
        ctu_poller_record_ok(poller);
    }

    return result;
}

/**
 * @brief 读取单调时钟（毫秒）
 */
static uint64_t ctu_poller_now_ms(void)
{
    struct timespec now;

    (void)clock_gettime(CLOCK_MONOTONIC, &now);

    return ((uint64_t)now.tv_sec * 1000U) + ((uint64_t)now.tv_nsec / 1000000U);
}

/**
 * @brief 刷新 1s 统计窗口，更新 fps 与丢包率
 */
static void ctu_poller_refresh_window(ctu_poller_t* poller)
{
    uint64_t now = ctu_poller_now_ms();
    uint64_t elapsed = now - poller->window_start_ms;

    if (elapsed >= CTU_POLLER_WINDOW_MS) {
        poller->stats.fps = ((float)poller->rx_in_window * 1000.0f)
            / (float)elapsed;
        poller->rx_in_window = 0U;
        poller->window_start_ms = now;
    }

    if (poller->stats.tx > 0U) {
        poller->stats.loss_rate =
            ((float)poller->stats.loss * 100.0f) / (float)poller->stats.tx;
    } else {
        poller->stats.loss_rate = 0.0f;
    }
}

/**
 * @brief 记录一次成功应答
 */
static void ctu_poller_record_ok(ctu_poller_t* poller)
{
    poller->stats.rx++;
    poller->rx_in_window++;
}

/**
 * @brief 记录一次失败
 */
static void ctu_poller_record_error(ctu_poller_t* poller, ctu_device_t device,
                                    uint8_t command, ctu_error_t error)
{
    poller->stats.loss++;
    if (poller->config.error_cb != NULL) {
        poller->config.error_cb(device, command, error, poller->config.user);
    }
}
