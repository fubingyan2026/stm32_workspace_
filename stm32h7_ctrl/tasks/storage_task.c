/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    storage_task.c
 * @brief   参数存储任务 — 验证 ring_storage + hal_flash Flash 持久化
 */

#include "storage_task.h"

#include "cmsis_os2.h"
#include "hal_flash.h"
#include "log.h"
#include "ring_storage.h"
#include "ring_storage_port.h"

#define STORAGE_TEST_LEVEL 2
/* 0 = 只初始化 hal_flash + 读 caps
 * 1 = + 读测试 (hal_flash_read)
 * 2 = + ring_storage 完整流程 */

#define TASK_STACK_SIZE 256U
#define TASK_PRIORITY   osPriorityBelowNormal

#define STORAGE_START_ADDR    0x080C0000U
#define STORAGE_AREA_SIZE     (2U * 128U * 1024U)  /* 256KB */
#define STORAGE_FRAME_BUF_SIZE 256U

static osThreadId_t s_task_handle;

/* ring_storage 上下文 */
static ring_storage_context_t s_storage;
static uint8_t s_frame_buf[STORAGE_FRAME_BUF_SIZE];
static uint32_t s_boot_count = 0;

/* ====== ring_storage port 回调 =============================================*/

extern ring_storage_error_t ring_storage_port_read(uint32_t addr,
    uint8_t* buf, size_t size);
extern ring_storage_error_t ring_storage_port_write(uint32_t addr,
    const uint8_t* buf, size_t size);
extern ring_storage_error_t ring_storage_port_erase(uint32_t addr,
    size_t size);
extern void ring_storage_port_lock(void);
extern void ring_storage_port_unlock(void);

static int port_read(uint32_t addr, uint8_t* buf, size_t size)
{
    return (int)ring_storage_port_read(addr, buf, size);
}

static int port_write(uint32_t addr, const uint8_t* buf, size_t size)
{
    return (int)ring_storage_port_write(addr, buf, size);
}

static int port_erase(uint32_t addr, size_t size)
{
    return (int)ring_storage_port_erase(addr, size);
}

static void port_lock(void)    { ring_storage_port_lock(); }
static void port_unlock(void)  { ring_storage_port_unlock(); }

/* ====== 任务入口 ===========================================================*/

static void storage_task_entry(void* argument)
{
    (void)argument;

    LOG_I("storage", "=== Flash driver test START (level=%d) ===", STORAGE_TEST_LEVEL);

    /* Step 0: hal_flash_init + get_caps */
    hal_flash_err_t err = hal_flash_init();
    LOG_I("storage", "hal_flash_init = %d (%s)", (int)err, hal_flash_err_str(err));

    const hal_flash_caps_t* caps = hal_flash_get_caps();
    LOG_I("storage", "Flash caps: base=0x%08lX size=%luKB erase=%luB write_gran=%u",
        (unsigned long)caps->addr,
        (unsigned long)(caps->total_size >> 10),
        (unsigned long)caps->erase_size,
        (unsigned)caps->write_gran);

#if STORAGE_TEST_LEVEL >= 1
    LOG_I("storage", "--- Step 1: hal_flash_read ---");
    uint8_t buf[16];
    err = hal_flash_read(STORAGE_START_ADDR - caps->addr, buf, sizeof(buf));
    LOG_I("storage", "hal_flash_read = %d, data[0]=0x%02X", (int)err, buf[0]);
#endif

#if STORAGE_TEST_LEVEL >= 2
    LOG_I("storage", "--- Step 2: ring_storage ---");

    const ring_storage_port_t port = {
        .read   = port_read,
        .write  = port_write,
        .erase  = port_erase,
        .lock   = port_lock,
        .unlock = port_unlock,
    };
    const ring_storage_config_t cfg = {
        .port              = port,
        .start_addr        = STORAGE_START_ADDR,
        .area_size         = STORAGE_AREA_SIZE,
        .sector_size       = RING_STORAGE_SECTOR_128K,
        .write_gran        = RING_STORAGE_WRITE_GRAN_256,
        .frame_buffer      = s_frame_buf,
        .frame_buffer_size = sizeof(s_frame_buf),
    };

    ring_storage_error_t rs_err = ring_storage_init(&s_storage, &cfg);
    LOG_I("storage", "ring_storage_init = %d", (int)rs_err);
    if (rs_err == RING_STORAGE_OK || rs_err == RING_STORAGE_ERROR_NO_VALID_FRAME) {
        ring_storage_register(&s_storage, "boot_cnt", &s_boot_count,
            sizeof(s_boot_count));

        ring_storage_register(&s_storage, "boot_cnt", &s_boot_count,
            sizeof(s_boot_count));

        LOG_I("storage", "latest_frame_addr=0x%08lX, write_offset=%lu",
            (unsigned long)s_storage.latest_frame_addr,
            (unsigned long)s_storage.write_offset);

        rs_err = ring_storage_load(&s_storage);
        LOG_I("storage", "ring_storage_load = %d, boot_cnt=%lu",
            (int)rs_err, (unsigned long)s_boot_count);

        if (rs_err == RING_STORAGE_OK) {
            s_boot_count++;
        } else {
            s_boot_count = 1;
        }

        rs_err = ring_storage_save(&s_storage);
        LOG_I("storage", "ring_storage_save = %d, boot_cnt=%lu",
            (int)rs_err, (unsigned long)s_boot_count);
    }
#endif

    LOG_I("storage", "=== Flash driver test DONE ===");

    for (;;) {
        osDelay(10000);
    }
}

void storage_task_init(void)
{
    const osThreadAttr_t attr = {
        .name       = "storage_task",
        .stack_size = TASK_STACK_SIZE * 4,
        .priority   = TASK_PRIORITY,
    };
    s_task_handle = osThreadNew(storage_task_entry, NULL, &attr);
}
