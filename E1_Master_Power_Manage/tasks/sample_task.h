/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    sample_task.h
 * @brief   ADC 采样任务 — sw_timer 驱动 adc_sample 服务
 */

#ifndef SAMPLE_TASK_H
#define SAMPLE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "srv_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

void sample_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* SAMPLE_TASK_H */
