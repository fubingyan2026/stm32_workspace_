/**
 * @file    boot_task.c
 * @brief   Boot 主任务实现 — 启动决策 + RS485 YMODEM 升级接收
 *
 * Flash 布局（编译期宏 BOOT_FLASH_* 定义，见 CMakeLists）：
 *   BOOT(0x08000000,32K) | AppA(0x08008000,96K) | AppB(0x08020000,96K)
 *   | 保留 24K | Meta(0x0803E000,8K, ring_storage)
 *
 * 运行模型（单链接 A + B 暂存提升）：
 *   新固件 YMODEM 下载到 B 槽 → 校验 → promote_to_a(B→A) → 复位从 A 启动。
 *   upgrade_flag：0=正常；1=下载中（B 可整槽重来）；2=提交中（断电可自愈续提交）。
 */

/* Includes ------------------------------------------------------------------*/
#include "boot_task.h"

#include "boot_flash.h"
#include "boot_ymodem.h"
#include "dev_rs485.h"
#include "drv_systick.h"
#include "log.h"
#include "log_task.h"
#include "main.h"
#include "tim.h"

#define BOOT_LOG_E(tag, ...) LOG_E(tag, __VA_ARGS__)
#define BOOT_LOG_W(tag, ...) LOG_W(tag, __VA_ARGS__)
#define BOOT_LOG_I(tag, ...) LOG_I(tag, __VA_ARGS__)
#define BOOT_LOG_D(tag, ...) LOG_D(tag, __VA_ARGS__)

/* Private constants ---------------------------------------------------------*/

/** @brief App 运行分区起始地址（A 槽，App 固定链接地址） */
#define BOOT_APP_A_ADDR (0x08008000U)

/** @brief RAM 区间（F103RCTx：48KB） */
#define BOOT_RAM_START (0x20000000U)
#define BOOT_RAM_SIZE (48U * 1024U)

/** @brief 文件结束(EOT/EOT)到提交前的安静窗口（保证 ACK 已上总线） */
#define BOOT_COMMIT_GRACE_MS (300U)

/** @brief 会话中途（已开始未完成）长时间无字节活动 → 复位回 App */
#define BOOT_SESSION_IDLE_MS (20000U)

/** @brief 指示灯亮/灭占空比阀值（TIM4 ARR=65535，取半亮） */
#define BOOT_LED_ON_CCR (0x8000U)
#define BOOT_LED_OFF_CCR (0U)

/* Private variables ---------------------------------------------------------*/

static boot_flash_context_t s_flash_ctx;
static boot_metadata_t s_meta;
static boot_ymodem_context_t s_ym;

static uint32_t s_upg_offset; /**< 已写入 B 槽的字节数（最终 fw_size） */
static uint32_t s_upg_checksum; /**< 已写入字节的 32-bit 累加和 */

static uint32_t s_last_byte_tick; /**< 最近一次收到 485 字节的时刻 */
static bool s_commit_started; /**< 提交流程已启动标志 */
static uint32_t s_done_tick; /**< 文件结束时刻 */

static bool s_led_on;
static uint32_t s_led_tick;

/* Private function prototypes -----------------------------------------------*/

static void boot_led_set(bool on);
static void boot_led_poll(uint32_t now_ms);

static bool boot_vector_sane(uint32_t app_addr);
static bool boot_sum_matches(boot_partition_t partition, uint32_t size,
    uint32_t expected);
static bool boot_app_ok(void);
static bool boot_backup_matches_meta(void);

static void boot_jump_to_app(void);
static void boot_reset_to_normal(void);

static uint8_t boot_ym_data_cb(void* user, const uint8_t* data, uint32_t len);
static void boot_ym_tx_cb(void* user, uint8_t byte);

static void boot_do_commit(void);
static void boot_session_abort(void);

/* Exported functions --------------------------------------------------------*/

bool boot_task_try_boot_app(void)
{
    if (boot_flash_init(&s_flash_ctx) != BOOT_FLASH_OK) {
        return false;
    }
    if (boot_flash_read_metadata(&s_flash_ctx, &s_meta) != BOOT_FLASH_OK) {
        return false;
    }
    if (s_meta.magic != BOOT_METADATA_MAGIC) {
        return false;
    }

    /* 情形：上次提交（flag=2）被断电打断 → 续完成提升 */
    if (s_meta.upgrade_flag == 2U) {
        BOOT_LOG_W("boot_task", "未完成提交，从 B 槽续提升");
        if (boot_backup_matches_meta()) {
            if (boot_flash_promote_to_a(&s_flash_ctx, BOOT_PARTITION_B,
                    s_meta.fw_size)
                == BOOT_FLASH_OK) {
                s_meta.upgrade_flag = 0U;
                (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
                boot_reset_to_normal();
                return true; /* 不可达 */
            }
        }
        /* B 与 meta 不符 / 提升失败：退回正常决策 */
        BOOT_LOG_W("boot_task", "B 槽无效，放弃续提交");
        s_meta.upgrade_flag = 0U;
        (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    }

    /* 情形：App 请求升级（flag=1）或下载中断 → 进入升级模式 */
    if (s_meta.upgrade_flag == 1U) {
        BOOT_LOG_W("boot_task", "App 请求升级，进入升级模式");
        return false;
    }

    /* 正常路径：A 有效直接启动 */
    if (boot_app_ok()) {
        BOOT_LOG_I("boot_task", "A 分区校验通过，跳转 App");
        boot_jump_to_app();
        return true; /* 不可达 */
    }

    /* A 无效：若 B 仍保存完整镜像（==meta 记录），自动重建 A */
    if ((s_meta.fw_size >= 16U) && boot_backup_matches_meta()) {
        BOOT_LOG_W("boot_task", "A 分区无效，从 B 槽自动重建");
        if (boot_flash_promote_to_a(&s_flash_ctx, BOOT_PARTITION_B,
                s_meta.fw_size)
            == BOOT_FLASH_OK) {
            s_meta.upgrade_flag = 0U;
            (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
            if (boot_app_ok()) {
                boot_jump_to_app();
                return true; /* 不可达 */
            }
        }
    }

    /* 空片/无有效镜像：进入升级模式常驻等待 */
    BOOT_LOG_W("boot_task", "无有效 App 镜像，常驻升级模式");
    return false;
}

void boot_task_init(void)
{
    static const boot_ymodem_config_t s_ym_cfg = {
        .tx_byte = boot_ym_tx_cb,
        .data_cb = boot_ym_data_cb,
        .user = NULL,
        .header_period_ms = 1000U,
        .packet_timeout_ms = 3000U,
    };

    /* RS485 收发（USART3 + PC5 DE） */
    (void)dev_rs485_init();

    /* 状态指示灯 PWM（TIM4_CH1 → PB6） */
    (void)HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    boot_led_set(false);

    /* 清空暂存槽 B（整槽擦除；B 仅暂存用，丢旧内容不影响 A 启动） */
    if (boot_flash_erase_partition(&s_flash_ctx, BOOT_PARTITION_B)
        != BOOT_FLASH_OK) {
        BOOT_LOG_E("boot_task", "B 槽擦除失败");
        for (;;) {
            boot_led_set(true);
        }
    }

    /* 标记进入升级会话（中断后重启可自动继续接收） */
    s_meta.upgrade_flag = 1U;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);

    s_upg_offset = 0U;
    s_upg_checksum = 0U;
    s_last_byte_tick = 0U;
    s_commit_started = false;
    s_done_tick = 0U;

    boot_ymodem_init(&s_ym, &s_ym_cfg);
    BOOT_LOG_I("boot_task", "升级会话就绪，等待 'C'");
}

void boot_task_poll(void)
{
    const uint32_t now_ms = millis();

    /* 1) 搬运 485 接收字节到 YMODEM 接收器 */
    uint8_t rx_buf[64];
    for (uint32_t round = 0U; round < 32U; round++) {
        const uint32_t got = dev_rs485_rx_read(rx_buf, sizeof(rx_buf));
        if (got == 0U) {
            break;
        }
        for (uint32_t i = 0U; i < got; i++) {
            boot_ymodem_feed(&s_ym, rx_buf[i]);
        }
        s_last_byte_tick = now_ms;
        s_ym.last_rx_ms = now_ms;
        if (got < sizeof(rx_buf)) {
            break;
        }
    }

    /* 2) YMODEM 心跳/超时调度 */
    boot_ymodem_poll(&s_ym, now_ms);

    /* 3) 485 TX 队列排空 */
    dev_rs485_tx_flush();

    /* 4) 指示灯 */
    boot_led_poll(now_ms);

    /* 5) 结束 → 提交窗口 */
    if (boot_ymodem_file_done(&s_ym) && !s_commit_started) {
        s_commit_started = true;
        s_done_tick = now_ms;
        BOOT_LOG_I("boot_task", "文件接收完成: len=%lu", (unsigned long)s_upg_offset);
    }
    if (s_commit_started) {
        if (!dev_rs485_is_tx_busy()
            && (now_ms - s_done_tick >= BOOT_COMMIT_GRACE_MS)) {
            boot_do_commit();
        }
        return;
    }

    /* 6) 异常处理：CAN 中止 / 会话中途超时 → 清标志复位回 App（无有效 App 则回升级态） */
    if (boot_ymodem_aborted(&s_ym)) {
        BOOT_LOG_W("boot_task", "升级会话被中止（CAN/错误）");
        boot_session_abort();
        return;
    }
    if (boot_ymodem_started(&s_ym) && !boot_ymodem_file_done(&s_ym)) {
        if ((s_last_byte_tick != 0U)
            && (now_ms - s_last_byte_tick >= BOOT_SESSION_IDLE_MS)) {
            BOOT_LOG_W("boot_task", "升级会话超时 %ums", (unsigned)BOOT_SESSION_IDLE_MS);
            boot_session_abort();
        }
    }
}

/* Private functions ---------------------------------------------------------*/

/* --- 指示灯（TIM4_CH1 PWM） --- */

static void boot_led_set(bool on)
{
    s_led_on = on;
    TIM4->CCR1 = on ? BOOT_LED_ON_CCR : BOOT_LED_OFF_CCR;
}

static void boot_led_poll(uint32_t now_ms)
{
    if (s_commit_started) {
        boot_led_set(true);
        return;
    }
    /* 未提交：按 250ms 周期快闪指示 bootloader 活动 */
    if ((now_ms - s_led_tick) >= 250U) {
        s_led_tick = now_ms;
        boot_led_set(!s_led_on);
    }
}

/* --- 镜像校验辅助 --- */

static bool boot_vector_sane(uint32_t app_addr)
{
    const uint32_t sp = *(volatile uint32_t*)app_addr;
    const uint32_t pc = *(volatile uint32_t*)(app_addr + 4U);
    if ((sp < BOOT_RAM_START) || (sp >= BOOT_RAM_START + BOOT_RAM_SIZE)) {
        return false;
    }
    if ((pc & 1U) == 0U) {
        return false;
    }
    if ((pc < app_addr) || (pc >= app_addr + BOOT_FLASH_APP_SIZE)) {
        return false;
    }
    return true;
}

/** @brief 计算分区前 size 字节的 32-bit 累加和并与期望值比较 */
static bool boot_sum_matches(boot_partition_t partition, uint32_t size,
    uint32_t expected)
{
    const uint32_t base = boot_flash_partition_addr(partition);
    uint32_t sum = 0U;
    const volatile uint8_t* p = (const volatile uint8_t*)base;
    for (uint32_t i = 0U; i < size; i++) {
        sum += p[i];
    }
    return sum == expected;
}

/** @brief 依据 meta 判定 A 槽镜像是否完整可启动 */
static bool boot_app_ok(void)
{
    if (s_meta.fw_size < 16U || s_meta.fw_size > BOOT_FLASH_APP_SIZE) {
        return false;
    }
    if (!boot_sum_matches(BOOT_PARTITION_A, s_meta.fw_size,
            s_meta.fw_checksum)) {
        return false;
    }
    return boot_vector_sane(boot_flash_partition_addr(BOOT_PARTITION_A));
}

/** @brief B 槽当前内容是否等于 meta 记录的完整镜像（用于自愈/续提交） */
static bool boot_backup_matches_meta(void)
{
    if (s_meta.fw_size < 16U || s_meta.fw_size > BOOT_FLASH_APP_SIZE) {
        return false;
    }
    return boot_sum_matches(BOOT_PARTITION_B, s_meta.fw_size,
        s_meta.fw_checksum);
}

/* --- 跳转 / 复位 --- */

static void boot_jump_to_app(void)
{
    const uint32_t app_addr = boot_flash_partition_addr(BOOT_PARTITION_A);
    const uint32_t sp = *(volatile uint32_t*)app_addr;
    const uint32_t pc = *(volatile uint32_t*)(app_addr + 4U);

    /* 日志 FIFO 排空并等 USART1 DMA 完成后跳转（保证启动日志完整输出） */

    __disable_irq();
    __set_MSP(sp);
    ((void (*)(void))pc)();
    for (;;) {
    }
}

static void boot_reset_to_normal(void)
{
    drv_system_reset();
    for (;;) {
    }
}

/* --- YMODEM 平台回调 --- */

/**
 * @brief YMODEM 数据块回调：写入 B 槽（含读回校验），维护长度/累加和
 * @return 0=成功；非 0=Flash 错误（模块将 CAN 中止）
 */
static uint8_t boot_ym_data_cb(void* user, const uint8_t* data, uint32_t len)
{
    (void)user;

    uint32_t n = len;
    if (s_upg_offset >= BOOT_FLASH_APP_SIZE) {
        return 1U;
    }
    if ((n > BOOT_FLASH_APP_SIZE) || (s_upg_offset + n > BOOT_FLASH_APP_SIZE)) {
        n = BOOT_FLASH_APP_SIZE - s_upg_offset;
    }
    /* 头包声明了文件长度：截掉末块 0x1A 填充 */
    if ((s_ym.file_len != 0U) && (s_upg_offset + n > s_ym.file_len)) {
        if (s_upg_offset >= s_ym.file_len) {
            return 0U; /* 整块填充，跳过 */
        }
        n = s_ym.file_len - s_upg_offset;
    }
    if (n == 0U) {
        return 0U;
    }

    if (boot_flash_write_block(&s_flash_ctx, BOOT_PARTITION_B,
            s_upg_offset, data, n)
        != BOOT_FLASH_OK) {
        return 1U;
    }
    if (boot_flash_verify_block(&s_flash_ctx, BOOT_PARTITION_B,
            s_upg_offset, data, n)
        != BOOT_FLASH_OK) {
        return 1U;
    }

    for (uint32_t i = 0U; i < n; i++) {
        s_upg_checksum += data[i];
    }
    s_upg_offset += n;
    return 0U;
}

/** @brief 发送单字节（ACK/NAK/C/CAN） */
static void boot_ym_tx_cb(void* user, uint8_t byte)
{
    (void)user;
    (void)dev_rs485_send(&byte, 1U);
}

/* --- 提交 / 中止 --- */

/**
 * @brief 提交升级：写提交标志(flag=2) → B→A 提升 → 清标志 → 复位
 *
 * 步骤间任意断电均可在下次上电自愈（flag==1/2 语义见文件头）。
 */
static void boot_do_commit(void)
{
    /* 空文件保护 */
    if (s_upg_offset < 16U || s_upg_offset > BOOT_FLASH_APP_SIZE) {
        BOOT_LOG_E("boot_task", "文件长度非法: %lu", (unsigned long)s_upg_offset);
        boot_session_abort();
        return;
    }

    s_meta.upgrade_flag = 2U;
    s_meta.boot_partition = (uint8_t)BOOT_PARTITION_A;
    s_meta.version = (uint16_t)(s_meta.version + 1U);
    s_meta.fw_size = s_upg_offset;
    s_meta.fw_checksum = s_upg_checksum;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);

    BOOT_LOG_I("boot_task", "提升中 size=%lu crc=0x%08lX",
        (unsigned long)s_upg_offset, (unsigned long)s_upg_checksum);
    if (boot_flash_promote_to_a(&s_flash_ctx, BOOT_PARTITION_B, s_upg_offset)
        != BOOT_FLASH_OK) {
        /* 提升失败（Flash 硬件错误）：清标志复位，回退 A 或进入升级态 */
        BOOT_LOG_E("boot_task", "B→A 提升失败，复位回退");
        s_meta.upgrade_flag = 0U;
        (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
        boot_reset_to_normal();
        return;
    }

    s_meta.upgrade_flag = 0U;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    BOOT_LOG_I("boot_task", "升级成功，复位启动新固件");
    boot_led_set(true);
    drv_system_reset();
    for (;;) {
    }
}

/** @brief 中止升级会话：清标志 → 复位（回 A 或重新进入升级态） */
static void boot_session_abort(void)
{
    s_meta.upgrade_flag = 0U;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    BOOT_LOG_W("boot_task", "中止升级并复位");
    boot_reset_to_normal();
}
