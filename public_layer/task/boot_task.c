/**
 * @file    boot_task.c
 * @brief   Boot 主任务实现 — 启动决策 + CAN 升级接收
 */

/* Includes ------------------------------------------------------------------*/
#include "boot_task.h"

#include <string.h>

#include "boot_flash.h"
#include "boot_fsm.h"
#include "boot_transport.h"
#include "drv_can.h"
#include "drv_systick.h"
#include "fsm.h"
#include "log.h"
#include "log_task.h"
#include "main.h" /* core_cm4.h：__disable_irq / __set_MSP */
#include "msg_fifo.h"
#include "sw_timer.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define BOOT_TASK_LOG_ENABLE 1

#if BOOT_TASK_LOG_ENABLE
#define BOOT_TASK_LOG_E(...) LOG_E("boot_task", __VA_ARGS__)
#define BOOT_TASK_LOG_W(...) LOG_W("boot_task", __VA_ARGS__)
#define BOOT_TASK_LOG_I(...) LOG_I("boot_task", __VA_ARGS__)
#define BOOT_TASK_LOG_D(...) LOG_D("boot_task", __VA_ARGS__)
#else
#define BOOT_TASK_LOG_E(...) ((void)0)
#define BOOT_TASK_LOG_W(...) ((void)0)
#define BOOT_TASK_LOG_I(...) ((void)0)
#define BOOT_TASK_LOG_D(...) ((void)0)
#endif

/** 硬件兼容 ID（与上位机工具 hw_id 字段对应；板级可覆写）。
 * 默认 F407/E1_Master 取 0x0002；G474 等经编译定义 BOOT_HW_COMPAT_ID=0x0001U 覆盖。 */
#ifndef BOOT_HW_COMPAT_ID
#define BOOT_HW_COMPAT_ID (0x0002U)
#endif

/** CAN RX 消息队列容量 */
#define BOOT_MSG_FIFO_SIZE 256U

/** 主循环轮询周期 (ms) */
#define BOOT_TASK_PERIOD_MS 1U

/** Bus-Off 轮询周期 (ms)，以主循环周期为 tick 累加 */
#define BOOT_BUSOFF_POLL_MS 100U

/** 升级失败/取消后，等待多少 ms 无新升级会话即回滚到上个版本（可调） */
#define BOOT_ROLLBACK_DELAY_MS 2000U

/** 初始 IDLE（无任何升级会话）等待升级指令：先警告、再回滚（可调） */
#define BOOT_IDLE_WARN_MS (6000U) /**< 等待超时警告 */
#define BOOT_IDLE_ROLLBACK_MS (12000U) /**< 等待超时回滚（仅当存在有效上个版本） */

/** App 分区 Flash 地址范围（用于跳转前向量表合法性校验；板级可覆写）。
 * 默认 F407：AppA 0x08020000 + 128KB；G474 等经编译定义覆盖（如
 * BOOT_APP_FLASH_START=0x08010000U / BOOT_APP_FLASH_END=0x08016000U）。 */
#ifndef BOOT_APP_FLASH_START
#define BOOT_APP_FLASH_START (0x08020000U)
#endif
#ifndef BOOT_APP_FLASH_END
#define BOOT_APP_FLASH_END (0x08040000U) /* 分区起始 + 128KB */
#endif
/** RAM 范围（向量表 SP 校验；默认 128KB，F407 与 G474 相同） */
#ifndef BOOT_RAM_START
#define BOOT_RAM_START (0x20000000U)
#endif
#ifndef BOOT_RAM_END
#define BOOT_RAM_END (0x20020000U) /* 128KB */
#endif

/* Private variables ---------------------------------------------------------*/

/* CAN 消息队列 */
static drv_can_msg_t s_msg_fifo_buffer[BOOT_MSG_FIFO_SIZE];
static msg_fifo_t s_msg_fifo;

/* Flash 分区管理器 */
static boot_flash_context_t s_flash_ctx;
static bool s_flash_inited = false;

/* 状态机实例 */
static fsm_t s_fsm;
static boot_fsm_context_t s_fsm_ctx;

/* 定时器 */
static sw_timer_t s_timer;

/* 目标分区 */
static boot_partition_t s_target_partition = BOOT_PARTITION_A;

/* Bus-Off 轮询降频计数器 */
static uint32_t s_busoff_poll_tick = 0U;

/* 失败/取消回滚状态 */
static bool s_session_started; /**< 本次 boot 会话是否开始过（收到过 START） */
static bool s_rollback_armed; /**< 回滚倒计时是否已启动 */
static uint32_t s_rollback_deadline; /**< 回滚触发时刻（毫秒） */

/* 初始 IDLE 等待升级指令超时（无会话场景） */
static bool s_idle_timer_active; /**< 空闲计时进行中 */
static uint32_t s_idle_start_ms; /**< 进入初始 IDLE 的时刻 */
static bool s_idle_warned; /**< 已发等待超时警告 */
static bool s_idle_rollback_done; /**< 已执行回滚判定（防重复） */

/* Private function prototypes -----------------------------------------------*/

static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg);
static void boot_timer_cb(void* user_data);
static void boot_check_rollback(uint32_t now_ms);
static void boot_rollback(void);
static void boot_rollback_to_prev(void);
static bool boot_partition_is_valid(const boot_metadata_t* meta);

/* 回调函数 */
static uint8_t write_block_cb(void* user_data, uint32_t offset,
    const uint8_t* data, uint32_t len);
static uint8_t verify_block_cb(void* user_data, uint32_t offset,
    const uint8_t* data, uint32_t len);
static uint8_t verify_fw_cb(void* user_data, uint32_t size, uint32_t* checksum);
static uint8_t erase_cb(void* user_data);
static uint8_t set_flag_cb(void* user_data, uint8_t boot_partition,
    uint16_t version, uint32_t fw_size, uint32_t fw_checksum);
static void reset_cb(void* user_data);

/* Exported functions --------------------------------------------------------*/

bool boot_task_try_boot_app(void)
{
    boot_metadata_t meta;

    if (!s_flash_inited) {
        boot_flash_init(&s_flash_ctx);
        s_flash_inited = true;
    }
    boot_flash_read_metadata(&s_flash_ctx, &meta);

    /* 检查 Metadata 有效性 */
    if (meta.magic != BOOT_METADATA_MAGIC) {
        BOOT_TASK_LOG_I("未找到有效 Metadata，进入 Bootloader 模式");
        return false;
    }

    if (meta.upgrade_flag != 0U) {
        BOOT_TASK_LOG_I("升级标志置位，进入 Bootloader 模式 (flag=%u)", meta.upgrade_flag);
        return false;
    }

    BOOT_TASK_LOG_I("Metadata 有效: 分区=%c, 版本=%u, 大小=%u, checksum=0x%08lX",
        (meta.boot_partition == BOOT_PARTITION_A) ? 'A' : 'B',
        meta.version, meta.fw_size, meta.fw_checksum);

    /* 无有效 App（fw_size=0，首烧或未升级过）：留在 Bootloader */
    if (meta.fw_size == 0U) {
        BOOT_TASK_LOG_I("fw_size=0，无已烧写 App，进入 Bootloader 模式");
        return false;
    }

    /* 校验和验证 App 分区 */
    uint32_t calculated_checksum;
#ifdef BOOT_SINGLE_PARTITION
    const boot_partition_t part = BOOT_PARTITION_A;
#else
    boot_partition_t part = (meta.boot_partition == BOOT_PARTITION_A)
        ? BOOT_PARTITION_A
        : BOOT_PARTITION_B;
#endif
    if (boot_flash_compute_checksum(&s_flash_ctx, part,
            meta.fw_size, &calculated_checksum)
        != BOOT_FLASH_OK) {
        BOOT_TASK_LOG_E("分区 %c 校验和计算失败，进入 Bootloader",
            (part == BOOT_PARTITION_A) ? 'A' : 'B');
        return false;
    }

    if (calculated_checksum != meta.fw_checksum) {
        BOOT_TASK_LOG_E("分区 %c 校验和不匹配: 期望 0x%08lX, 计算 0x%08lX, 进入 Bootloader",
            (part == BOOT_PARTITION_A) ? 'A' : 'B', meta.fw_checksum, calculated_checksum);
        return false;
    }

#ifndef BOOT_SINGLE_PARTITION
    /* App 固定链接于 A（0x08020000），只能从 A 运行：
     * 活动分区为 B 时先提升（拷贝 B→A），再从 A 启动——否则跳 B 的内嵌 PC 会落到 A 的
     * 旧地址，取指失败进 Default_Handler。 */
    if (part == BOOT_PARTITION_B) {
        BOOT_TASK_LOG_I("活动分区 B，提升到 A (size=%lu)...", (unsigned long)meta.fw_size);
        if (boot_flash_promote_to_a(&s_flash_ctx, BOOT_PARTITION_B, meta.fw_size)
            != BOOT_FLASH_OK) {
            BOOT_TASK_LOG_E("提升分区 B → A 失败，进入 Bootloader");
            return false;
        }
        /* 提升后 A 即运行槽，更新 metadata 保持一致（下次升级写对侧 B） */
        meta.boot_partition = BOOT_PARTITION_A;
        boot_flash_write_metadata(&s_flash_ctx, &meta);
        part = BOOT_PARTITION_A;
        BOOT_TASK_LOG_I("提升完成，从 A 分区启动");
    }
#endif /* !BOOT_SINGLE_PARTITION */

    /* 跳转到 App */
    BOOT_TASK_LOG_I("校验和验证通过，跳转到分区 %c, 版本=%u",
        (part == BOOT_PARTITION_A) ? 'A' : 'B', meta.version);

    const uint32_t app_addr = boot_flash_partition_addr(part);
    const uint32_t app_sp = *(const volatile uint32_t*)app_addr;
    const uint32_t app_pc = *(const volatile uint32_t*)(app_addr + 4U);

    /* 向量表合法性校验（防跳转空 Flash / 越界指针）。
     * 初始 SP 通常等于 RAM 末尾（_estack，栈向下生长），故允许 == RAM_END。 */
    if (app_sp < BOOT_RAM_START || app_sp > BOOT_RAM_END
        || app_pc < BOOT_APP_FLASH_START || app_pc >= BOOT_APP_FLASH_END) {
        BOOT_TASK_LOG_E("分区 %c 向量表非法 (sp=0x%08lX, pc=0x%08lX)，进入 Bootloader",
            (part == BOOT_PARTITION_A) ? 'A' : 'B',
            (unsigned long)app_sp, (unsigned long)app_pc);
        return false;
    }

    /* 跳转前排空日志（UART DMA 异步，不 flush 则最后几句日志在跳转瞬间被打断丢失） */
    log_task_flush();

/* 恢复外设和时钟到默认状态 */
{
// #include "usart.h"
// #include "fdcan.h"
    /* 关闭 SysTick */
    // SysTick->CTRL = 0;
    // SysTick->LOAD = 0;
    // SysTick->VAL = 0;
    // HAL_FDCAN_DeInit(&hfdcan1);
    // HAL_UART_DeInit(&huart1);
    // HAL_RCC_DeInit();
    HAL_DeInit();
}
    /* 关闭全局中断，设置 MSP 并跳转（App 启动流程会重设 SCB->VTOR） */
    __disable_irq();
    __set_MSP(app_sp);
    ((void (*)(void))app_pc)();

    /* 不应到达此处 */
    return false;
}

uint8_t boot_task_get_state(void)
{
    return boot_fsm_get_state(&s_fsm_ctx);
}

void boot_task_init(void)
{
    boot_flash_error_t flash_err;
    boot_fsm_config_t fsm_config;
    drv_can_error_t can_err;

    BOOT_TASK_LOG_I("初始化 Bootloader...");

    /* 初始化消息队列 */
    msg_fifo_init(&s_msg_fifo, s_msg_fifo_buffer,
        sizeof(s_msg_fifo_buffer), sizeof(drv_can_msg_t));
    BOOT_TASK_LOG_D("消息队列已初始化 (%u x %u 字节)",
        BOOT_MSG_FIFO_SIZE, (uint32_t)sizeof(drv_can_msg_t));

    /* 初始化 Flash（如已在 try_boot_app 中初始化则跳过） */
    if (!s_flash_inited) {
        flash_err = boot_flash_init(&s_flash_ctx);
        if (flash_err != BOOT_FLASH_OK) {
            BOOT_TASK_LOG_E("Flash 初始化失败: err=%d", flash_err);
            return;
        }
        s_flash_inited = true;
    }
    BOOT_TASK_LOG_D("Flash 管理器已初始化");

    /* 读取 Metadata，确定目标升级分区 */
    {
#ifdef BOOT_SINGLE_PARTITION
        s_target_partition = BOOT_PARTITION_A;
        BOOT_TASK_LOG_I("单分区模式, 目标分区 A");
#else
        boot_metadata_t meta;
        boot_flash_read_metadata(&s_flash_ctx, &meta);
        /* “有有效 App” 以 fw_size>0 判定（当前分区确有固件），不能用 upgrade_flag：
         * 0x003 触发的升级会话 upgrade_flag=1，但对侧旧分区仍有效，应写相反分区；
         * 反之全新板（fw_size=0）upgrade_flag=0 也不该误判为”有 App”。 */
        if (meta.magic == BOOT_METADATA_MAGIC && meta.fw_size > 0U
            && meta.boot_partition <= BOOT_PARTITION_B) {
            /* 有有效 App → 升级到相反分区 */
            s_target_partition = (meta.boot_partition == BOOT_PARTITION_A)
                ? BOOT_PARTITION_B
                : BOOT_PARTITION_A;
            BOOT_TASK_LOG_I("当前分区 %c, 目标分区 %c",
                'A' + meta.boot_partition, 'A' + s_target_partition);
        } else {
            /* 无有效 App → 默认写到 A */
            s_target_partition = BOOT_PARTITION_A;
            BOOT_TASK_LOG_I("无有效 App, 目标分区 A");
        }
#endif /* !BOOT_SINGLE_PARTITION */
    }

    /* 初始化 CAN（E1 驱动无参，内部句柄表） */
    can_err = drv_can_init();
    if (can_err != DRV_CAN_OK) {
        BOOT_TASK_LOG_E("CAN 初始化失败: err=%d", can_err);
        return;
    }
    drv_can_register_rx_callback(DRV_CAN_CH_1, can_rx_callback);
    BOOT_TASK_LOG_D("CAN 已初始化 (CAN1, PA11/PA12)");

    /* 初始化状态机 */
    memset(&fsm_config, 0, sizeof(fsm_config));
    fsm_config.write_block_cb = write_block_cb;
    fsm_config.verify_block_cb = verify_block_cb;
    fsm_config.verify_fw_cb = verify_fw_cb;
    fsm_config.erase_cb = erase_cb;
    fsm_config.set_flag_cb = set_flag_cb;
    fsm_config.reset_cb = reset_cb;
    fsm_config.user_data = NULL;
    fsm_config.hw_compat_id = BOOT_HW_COMPAT_ID;
    fsm_config.target_partition = (uint8_t)s_target_partition;

    if (!boot_fsm_init(&s_fsm_ctx, &s_fsm, &fsm_config)) {
        BOOT_TASK_LOG_E("FSM 状态机初始化失败");
        return;
    }
    BOOT_TASK_LOG_D("升级状态机已初始化 (HW_ID=0x%04X)", fsm_config.hw_compat_id);

    /* 创建轮询定时器 */
    sw_timer_init(&s_timer,
        &(sw_timer_config_t) {
            .priority = SW_TIMER_PRIO_NORMAL,
            .callback = boot_timer_cb,
            .user_data = NULL });
    sw_timer_start(&s_timer, BOOT_TASK_PERIOD_MS, 0); /* 0 = 无限重复 */
    BOOT_TASK_LOG_D("轮询定时器已启动 (%u ms 周期)", BOOT_TASK_PERIOD_MS);

    BOOT_TASK_LOG_I("Bootloader 就绪，等待 CAN 升级指令...");
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief CAN RX 中断回调（ISR 上下文）
 */
static void can_rx_callback(drv_can_channel_t ch, const drv_can_msg_t* msg)
{
    (void)ch;
    /* 仅处理来自上位机的消息 */
    if (msg->id == BOOT_CAN_ID_HOST_TO_NODE) {
        msg_fifo_push(&s_msg_fifo, msg);
    }
}

/**
 * @brief 主循环定时器回调（主循环上下文）
 */
static void boot_timer_cb(void* user_data)
{
    drv_can_msg_t msg;
    drv_can_msg_t response;
    (void)user_data;

    /* 消费所有待处理 CAN 消息 */
    while (msg_fifo_pop(&s_msg_fifo, &msg)) {
        boot_fsm_process_msg(&s_fsm_ctx, &msg);

        /* 发送 ACK/NACK 响应 */
        if (boot_fsm_get_response(&s_fsm_ctx, &response)) {
            drv_can_send(DRV_CAN_CH_1, &response);
        }
    }

    /* 更新超时 */
    boot_fsm_tick(&s_fsm_ctx);

    /* 补发 tick 内生成的响应（如块级看门狗超时 NACK） */
    if (boot_fsm_get_response(&s_fsm_ctx, &response)) {
        drv_can_send(DRV_CAN_CH_1, &response);
    }

    /* REBOOT：ACK 已送入 TX 邮箱，等其真正发出（邮箱全部空闲）后再系统复位——
     * 否则复位会打断在途 ACK，上位机收不到 REBOOT 应答。
     * 用 millis() 有界轮询而非固定 delay：8B @1Mbps ≈ 130µs，通常一两个循环即出，
     * 超时（50ms）兜底仍执行复位。 */
    if (boot_fsm_take_reset(&s_fsm_ctx)) {
        const uint32_t wait_start = millis();
        while (!drv_can_tx_all_done(DRV_CAN_CH_1)) {
            if ((uint32_t)(millis() - wait_start) > 50U) {
                BOOT_TASK_LOG_W("等待 REBOOT ACK 发出超时，仍执行复位");
                break;
            }
        }
        reset_cb(NULL);
    }

    /* 失败/取消回滚检查（会话结束后延时无新会话 → 复位跳转上个版本） */
    boot_check_rollback(millis());

    /* CAN Bus-Off 自恢复：每 ~100ms 轮询一次 */
    s_busoff_poll_tick++;
    if (s_busoff_poll_tick >= BOOT_BUSOFF_POLL_MS) {
        s_busoff_poll_tick = 0U;
        if (drv_can_is_bus_off(DRV_CAN_CH_1)) {
            BOOT_TASK_LOG_W("检测到 CAN Bus-Off，自动恢复");
            drv_can_recover(DRV_CAN_CH_1);
        }
    }
}

/* ===== 回调函数实现 ======================================================= */

static uint8_t write_block_cb(void* user_data, uint32_t offset,
    const uint8_t* data, uint32_t len)
{
    (void)user_data;
    (void)data;
    boot_flash_error_t err = boot_flash_write_block(&s_flash_ctx,
        s_target_partition, offset, data, len);
    if (err != BOOT_FLASH_OK) {
        BOOT_TASK_LOG_E("Flash 写入失败: err=%d, offset=%lu, len=%lu", err, offset, len);
    }
    return (uint8_t)err;
}

static uint8_t verify_block_cb(void* user_data, uint32_t offset,
    const uint8_t* data, uint32_t len)
{
    (void)user_data;
    (void)data;
    boot_flash_error_t err = boot_flash_verify_block(&s_flash_ctx,
        s_target_partition, offset, data, len);
    if (err != BOOT_FLASH_OK) {
        BOOT_TASK_LOG_E("Flash 读回校验失败: err=%d, offset=%lu", err, offset);
    }
    return (uint8_t)err;
}

static uint8_t verify_fw_cb(void* user_data, uint32_t size, uint32_t* checksum)
{
    (void)user_data;
    boot_flash_error_t err = boot_flash_compute_checksum(&s_flash_ctx,
        s_target_partition, size, checksum);
    if (err != BOOT_FLASH_OK) {
        BOOT_TASK_LOG_E("Checksum 计算失败: err=%d, size=%lu", err, size);
    }
    return (uint8_t)err;
}

static uint8_t erase_cb(void* user_data)
{
    (void)user_data;
    boot_flash_error_t err = boot_flash_erase_partition(&s_flash_ctx,
        s_target_partition);
    if (err != BOOT_FLASH_OK) {
        BOOT_TASK_LOG_E("分区擦除失败: err=%d, partition=%c",
            err, 'A' + s_target_partition);
    }
    return (uint8_t)err;
}

static uint8_t set_flag_cb(void* user_data, uint8_t boot_partition,
    uint16_t version, uint32_t fw_size, uint32_t fw_checksum)
{
    boot_metadata_t meta;
    (void)user_data;

    memset(&meta, 0, sizeof(meta));
    meta.magic = BOOT_METADATA_MAGIC;
    meta.boot_partition = boot_partition;
    meta.upgrade_flag = 0U; /* 升级完成 */
    meta.version = version;
    meta.fw_size = fw_size;
    meta.fw_checksum = fw_checksum;

    boot_flash_error_t err = boot_flash_write_metadata(&s_flash_ctx, &meta);
    if (err != BOOT_FLASH_OK) {
        BOOT_TASK_LOG_E("Metadata 写入失败: err=%d, part=%c, ver=%u",
            err, 'A' + boot_partition, version);
    } else {
        BOOT_TASK_LOG_I("Metadata 写入成功.");
    }
    return (uint8_t)err;
}

static void reset_cb(void* user_data)
{
    (void)user_data;
    BOOT_TASK_LOG_I("执行系统复位...");
    /* 复位前排空日志，避免"执行系统复位"与回滚提示被复位打断 */
    log_task_flush();
    drv_system_reset();
}

/* ===== 失败/取消回滚 ===================================================== */

/**
 * @brief 回滚检查（主循环 1ms 轮询）
 *
 * 升级会话（收到过 START）结束后回到 IDLE，若 BOOT_ROLLBACK_DELAY_MS 内无新会话，
 * 则执行 boot_rollback()。刚进入 boot 的初始 IDLE（未开始过会话）不回滚，等待主机 START。
 */
static void boot_check_rollback(uint32_t now_ms)
{
    if (boot_fsm_get_state(&s_fsm_ctx) != BOOT_STATE_IDLE) {
        /* 会话进行中或已重新开始：取消回滚倒计时与空闲计时 */
        s_session_started = true;
        s_rollback_armed = false;
        s_idle_timer_active = false;
        s_idle_warned = false;
        s_idle_rollback_done = false;
        return;
    }

    if (!s_session_started) {
        /* 初始 IDLE：尚未开始任何会话——等待升级指令超时处理（先警告，有上个版本则回滚） */
        if (!s_idle_timer_active) {
            s_idle_timer_active = true;
            s_idle_start_ms = now_ms;
        }
        const uint32_t idle_elapsed = (uint32_t)(now_ms - s_idle_start_ms);
        if (idle_elapsed >= BOOT_IDLE_WARN_MS && !s_idle_warned) {
            s_idle_warned = true;
            BOOT_TASK_LOG_W("等待升级指令超时(%ums)，%ums 内仍无指令将回滚到上个版本",
                (unsigned)BOOT_IDLE_WARN_MS,
                (unsigned)(BOOT_IDLE_ROLLBACK_MS - BOOT_IDLE_WARN_MS));
        }
        if (idle_elapsed >= BOOT_IDLE_ROLLBACK_MS && !s_idle_rollback_done) {
            s_idle_rollback_done = true;
            boot_rollback_to_prev();
        }
        return;
    }

    if (!s_rollback_armed) {
        /* 会话结束（失败/取消/全局超时）回到 IDLE：启动回滚倒计时 */
        s_rollback_armed = true;
        s_rollback_deadline = now_ms + BOOT_ROLLBACK_DELAY_MS;
        return;
    }

    /* 已到回滚时刻且仍处 IDLE → 执行回滚 */
    if ((int32_t)(now_ms - s_rollback_deadline) >= 0) {
        boot_rollback();
    }
}

/**
 * @brief 回滚到上个版本
 *
 * 清除 upgrade_flag（保留已提交分区/版本/校验和），随后系统复位；
 * Boot 复位后判定已提交分区有效并跳转上个版本。若从未提交过（无有效 metadata）
 * 则直接复位，由 Boot 重新判定（通常仍进入升级模式）。
 */
static void boot_rollback(void)
{
    BOOT_TASK_LOG_I("升级失败/取消，回滚到上个版本 (delay=%u ms)...",
        (unsigned)BOOT_ROLLBACK_DELAY_MS);

    boot_metadata_t meta;
    if (boot_flash_read_metadata(&s_flash_ctx, &meta) == BOOT_FLASH_OK
        && meta.magic == BOOT_METADATA_MAGIC) {
        meta.upgrade_flag = 0U;
        boot_flash_write_metadata(&s_flash_ctx, &meta);
    }

    s_session_started = false;
    s_rollback_armed = false;
    reset_cb(NULL);
}

/**
 * @brief 初始 IDLE 等待升级指令超时回滚：有有效上个版本则清除 upgrade_flag 并复位跳回，
 *        上个版本无效（首烧/校验坏/向量表非法）则保持 Boot 模式继续等待。
 */
static void boot_rollback_to_prev(void)
{
    boot_metadata_t meta;
    boot_flash_read_metadata(&s_flash_ctx, &meta);

    if (!boot_partition_is_valid(&meta)) {
        BOOT_TASK_LOG_W("上个版本无效 (magic/fw_size/校验和/向量表)，保持 Boot 模式等待升级");
        return; /* 无有效版本可回，保持等待（s_idle_rollback_done 已置位，不重复） */
    }

    BOOT_TASK_LOG_I("等待升级指令超时(%ums)，回滚到上个版本 (分区 %c)...",
        (unsigned)BOOT_IDLE_ROLLBACK_MS,
        (meta.boot_partition == BOOT_PARTITION_A) ? 'A' : 'B');
    meta.upgrade_flag = 0U;
    boot_flash_write_metadata(&s_flash_ctx, &meta);
    s_session_started = false;
    s_rollback_armed = false;
    reset_cb(NULL);
}

/**
 * @brief 判断 metadata 指向的已提交分区是否为可运行的有效固件
 *
 * 与 boot_task_try_boot_app 的跳转前校验一致：magic + fw_size>0 + 分区
 * 32-bit 累加和匹配 + 向量表合法（SP 在 RAM、PC 在 App 分区）。
 * 仅用 magic/fw_size 判断不足——分区数据可能损坏，会导致回滚后复位→校验失败→
 * 再进 boot→再回滚的无限循环。
 */
static bool boot_partition_is_valid(const boot_metadata_t* meta)
{
    if (meta->magic != BOOT_METADATA_MAGIC || meta->fw_size == 0U
#ifdef BOOT_SINGLE_PARTITION
        || meta->boot_partition != BOOT_PARTITION_A) {
#else
        || meta->boot_partition > BOOT_PARTITION_B) {
#endif
        return false;
    }

#ifdef BOOT_SINGLE_PARTITION
    const boot_partition_t part = BOOT_PARTITION_A;
    (void)meta;
#else
    const boot_partition_t part = (meta->boot_partition == BOOT_PARTITION_A)
        ? BOOT_PARTITION_A
        : BOOT_PARTITION_B;
#endif

    /* 整包 32-bit 累加和比对 */
    uint32_t calc = 0U;
    if (boot_flash_compute_checksum(&s_flash_ctx, part, meta->fw_size, &calc) != BOOT_FLASH_OK
        || calc != meta->fw_checksum) {
        return false;
    }

    /* 向量表合法性（初始 SP 允许 == RAM_END，栈向下生长） */
    const uint32_t app_addr = boot_flash_partition_addr(part);
    const uint32_t app_sp = *(const volatile uint32_t*)app_addr;
    const uint32_t app_pc = *(const volatile uint32_t*)(app_addr + 4U);
    if (app_sp < BOOT_RAM_START || app_sp > BOOT_RAM_END
        || app_pc < BOOT_APP_FLASH_START || app_pc >= BOOT_APP_FLASH_END) {
        return false;
    }

    return true;
}
