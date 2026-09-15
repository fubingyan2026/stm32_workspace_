/**
 * @file    boot_task.c
 * @brief   Boot 主任务实现 — 启动决策 + RS485 寻址分块升级（方案 B）
 *
 * Flash 布局（编译期宏 BOOT_FLASH_* 定义，见 CMakeLists）：
 *   BOOT(0x08000000,32K) | AppA(0x08008000,96K) | AppB(0x08020000,96K)
 *   | 保留 24K | Meta(0x0803E000,8K, ring_storage)
 *
 * 运行模型（单链接 A + B 暂存提升）：
 *   新固件经寻址分块协议写入 B 槽 → 校验 → promote_to_a(B→A) → 复位从 A 启动。
 *   upgrade_flag：0=正常；1=下载中（B 可整槽重来）；2=提交中（断电可自愈续提交）。
 *
 * 下载状态机统一由 service/boot_session 提供（与 App 侧共用）。
 * 多节点安全：Boot 绝不主动发送（无 'C' 心跳），只在被寻址帧校验通过后回帧。
 */

/* Includes ------------------------------------------------------------------*/
#include "boot_task.h"

#include "boot_flash.h"
#include "boot_session.h"
#include "dev_rs485.h"
#include "drv_led.h"
#include "drv_log_uart.h"
#include "drv_systick.h"
#include "log.h"
#include "log_task.h"
#include "main.h"

#define BOOT_LOG_E(tag, ...) LOG_E(tag, __VA_ARGS__)
#define BOOT_LOG_W(tag, ...) LOG_W(tag, __VA_ARGS__)
#define BOOT_LOG_I(tag, ...) LOG_I(tag, __VA_ARGS__)
#define BOOT_LOG_D(tag, ...) LOG_D(tag, __VA_ARGS__)

/* Private constants ---------------------------------------------------------*/

/** @brief RAM 区间（F103RCTx：48KB） */
#define BOOT_RAM_START (0x20000000U)
#define BOOT_RAM_SIZE (48U * 1024U)

/** @brief TX 排空等待上限（END 应答上总线后再提交） */
#define BOOT_TX_SYNC_MS (300U)

/** @brief 日志排空等待上限（跳转/复位前确保日志 DMA 发完） */
#define BOOT_LOG_SYNC_MS (500U)

/** @brief 指示灯亮度（drv_led 0~1023） */
#define BOOT_LED_ON_DUTY  (256)
#define BOOT_LED_OFF_DUTY (0U)

/** @brief 调试开关：置 1 跳过 App 校验（仅保留向量表 sanity）
 *  - 用途：调试器（J-Link/ST-Link）直接把 App 烧到 A 槽 0x08008000 后，
 *          不依赖 metadata（大小/累加和）也能启动；
 *  - 置 1 后：flag==2 的“续提交”不再执行（避免覆盖调试镜像）；
 *           App 的 0x06 升级请求仍有效（flag==1 照常进升级模式）；
 *  - 默认 0（量产校验完整）；经 CMake 选项 BOOT_SKIP_APP_VERIFY 打开 */
#ifndef BOOT_SKIP_APP_VERIFY
#define BOOT_SKIP_APP_VERIFY (0)
#endif

/** @brief App 镜像签名（App 侧 .app_sig 段，链接器固定在分区 +0x200）
 *  调试模式（跳过 metadata 校验）时用于判定 A 槽是否为“有效固件” */
#define BOOT_APP_SIG_OFFSET (0x200U)
#define BOOT_APP_SIG_MAGIC  (0x41505031U)

/* Private variables ---------------------------------------------------------*/

static boot_flash_context_t s_flash_ctx;
static boot_metadata_t s_meta;

static boot_session_t s_session;
static boot_session_config_t s_session_cfg;

static bool s_led_on;
static uint32_t s_led_tick;

/* Private function prototypes -----------------------------------------------*/

static void boot_led_set(bool on);
static void boot_led_poll(uint32_t now_ms);

static bool boot_vector_sane(uint32_t app_addr);
static bool boot_sum_matches(boot_partition_t partition, uint32_t size,
    uint32_t expected);
static bool boot_is_software_reset(void);
static bool boot_app_ok(void);
static bool boot_backup_matches_meta(void);

static void boot_jump_to_app(void);
static void boot_reset_to_normal(void);
static void boot_log_sync(void);
static void boot_tx_sync(void);

static void boot_tx(const uint8_t* frame, uint32_t len);
static void boot_commit(void* user, uint32_t size, uint32_t checksum);
static void boot_abort(void* user);
static void boot_id(void* user, uint8_t id);

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

#if BOOT_SKIP_APP_VERIFY
    /* 本次复位来源（读取后清标志），用于区分 App 0x06 软复位与上电复位 */
    const bool sw_reset = boot_is_software_reset();
#endif

    /* 情形：上次提交（flag=2）被断电打断 → 续完成提升
     * 调试模式（BOOT_SKIP_APP_VERIFY）：不执行续提交，避免覆盖调试器直烧的 A 槽镜像 */
#if !BOOT_SKIP_APP_VERIFY
    if (s_meta.upgrade_flag == 2U) {
        BOOT_LOG_W("boot_task", "续提交");
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
        BOOT_LOG_W("boot_task", "B 无效");
        s_meta.upgrade_flag = 0U;
        (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    }
#endif /* !BOOT_SKIP_APP_VERIFY */

    /* 情形：App 请求升级（flag=1）或下载中断 → 进入升级模式
     * 调试模式（BOOT_SKIP_APP_VERIFY）：仅当本次为“软件复位”时才认为是
     * App 0x06 的升级请求；上电/引脚复位的残留 flag 一律忽略，保证调试器直烧
     * 的 App 能直接运行，同时保留 485 升级能力。 */
#if BOOT_SKIP_APP_VERIFY
    if (s_meta.upgrade_flag == 1U) {
        if (sw_reset) {
            BOOT_LOG_W("boot_task", "App 请求升级(软复位)");
            return false;
        }
        BOOT_LOG_W("boot_task", "忽略残留 flag=1");
    }
#else
    if (s_meta.upgrade_flag == 1U) {
        BOOT_LOG_W("boot_task", "App 请求升级");
        return false;
    }
#endif

    /* 正常路径：A 有效直接启动 */
    if (boot_app_ok()) {
        BOOT_LOG_I("boot_task", "App.app_sig=0x%08lX",
            (unsigned long)*(volatile uint32_t*)
            (boot_flash_partition_addr(BOOT_PARTITION_A) + BOOT_APP_SIG_OFFSET));
        BOOT_LOG_I("boot_task", "跳转 App");
        boot_jump_to_app();
        return true; /* 不可达 */
    }

    /* A 无效：若 B 仍保存完整镜像（==meta 记录），自动重建 A */
    if ((s_meta.fw_size >= 16U) && boot_backup_matches_meta()) {
        BOOT_LOG_W("boot_task", "从 B 重建 A");
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

    BOOT_LOG_W("boot_task", "无有效 App");
    return false;
}

void boot_task_init(void)
{
    const uint8_t my_id = (uint8_t)(s_meta.reserved & 0xFFU);

    /* RS485 收发（USART3 + PC5 DE） */
    (void)dev_rs485_init();

    /* 状态指示灯（经 drv_led → drv_pwm → TIM4_CH1/PB6） */
    drv_led_init();
    boot_led_set(false);

    /* 标记进入升级会话（中断后重启可自动继续接收） */
    s_meta.upgrade_flag = 1U;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);

    /* 下载状态机（与 App 侧共用 boot_session） */
    s_session_cfg.flash = &s_flash_ctx;
    s_session_cfg.target = BOOT_PARTITION_B;
    s_session_cfg.tx = boot_tx;
    s_session_cfg.on_commit = boot_commit;
    s_session_cfg.on_abort = boot_abort;
    s_session_cfg.on_id = boot_id;
    s_session_cfg.my_id = my_id;
    s_session_cfg.user = NULL;
    boot_session_init(&s_session, &s_session_cfg);

    BOOT_LOG_I("boot_task", "就绪 ID=0x%02X", (unsigned)my_id);
}

void boot_task_poll(void)
{
    const uint32_t now_ms = millis();

    /* 1) 搬运 485 接收字节 → 寻址分块协议解析 */
    uint8_t rx_buf[64];
    for (uint32_t round = 0U; round < 32U; round++) {
        const uint32_t got = dev_rs485_rx_read(rx_buf, sizeof(rx_buf));
        if (got == 0U) {
            break;
        }
        for (uint32_t i = 0U; i < got; i++) {
            boot_session_feed(&s_session, rx_buf[i]);
        }
        if (got < sizeof(rx_buf)) {
            break;
        }
    }

    /* 2) 485 TX 队列排空 */
    dev_rs485_tx_flush();

    /* 3) 指示灯 */
    boot_led_poll(now_ms);
}

/* Private functions ---------------------------------------------------------*/

/* --- 指示灯（TIM4_CH1 PWM） --- */

static void boot_led_set(bool on)
{
    s_led_on = on;
    drv_led_set_duty(DRV_LED_CH_STATUS, on ? BOOT_LED_ON_DUTY : BOOT_LED_OFF_DUTY);
}

static void boot_led_poll(uint32_t now_ms)
{
    /* 未下载：500ms 慢闪；下载/提交中：常亮（由回调控制） */
    if (boot_session_downloading(&s_session)) {
        if ((now_ms - s_led_tick) >= 20U) {
            s_led_tick = now_ms;
            boot_led_set(!s_led_on);
        }
        return;
    }

    if ((now_ms - s_led_tick) >= 500U) {
        s_led_tick = now_ms;
        boot_led_set(!s_led_on);
    }
}

/* --- 镜像校验辅助 --- */

static bool boot_vector_sane(uint32_t app_addr)
{
    const uint32_t sp = *(volatile uint32_t*)app_addr;
    const uint32_t pc = *(volatile uint32_t*)(app_addr + 4U);
    /* 初始 MSP 允许等于 RAM 末尾（_estack = RAM 顶），故上界用 <= */
    if ((sp < BOOT_RAM_START) || (sp > (BOOT_RAM_START + BOOT_RAM_SIZE))) {
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

/** @brief 读取并清除本次复位来源：是否为“软件复位”(NVIC_SystemReset)
 *  @note  F1 RCC_CSR.SFTRSTF；读取后置 RMVF 清标志，避免误判。
 *        用于调试模式区分“App 0x06 请求升级(软复位)”与“上电残留 flag”。 */
static bool boot_is_software_reset(void)
{
    const bool sw = (RCC->CSR & RCC_CSR_SFTRSTF) != 0U;
    RCC->CSR |= RCC_CSR_RMVF; /* 清复位标志 */
    return sw;
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

/** @brief 依据 meta 判定 A 槽镜像是否完整可启动
 *  @note  BOOT_SKIP_APP_VERIFY=1 时仅做向量表 sanity（调试器直烧 A 槽场景），
 *         不比较 fw_size/fw_checksum 与 metadata。 */
static bool boot_app_ok(void)
{
#if BOOT_SKIP_APP_VERIFY
    /* 调试模式（调试器直烧 A 槽）：不看 metadata，但必须
       1) 向量表 sanity（SP/复位向量合法）  2) App 镜像签名魔数正确 —— 防止
       半写入/损坏镜像被误跳转导致死机。签名由 App 的 .app_sig 段提供。 */
    const uint32_t base = boot_flash_partition_addr(BOOT_PARTITION_A);
    if (!boot_vector_sane(base)) {
        BOOT_LOG_W("boot_task", "App 向量无效");
        return false;
    }
    const uint32_t sig = *(volatile uint32_t*)(base + BOOT_APP_SIG_OFFSET);
    if (sig != BOOT_APP_SIG_MAGIC) {
        BOOT_LOG_W("boot_task", "App 签名无效 0x%08lX", (unsigned long)sig);
        return false;
    }
    return true;
#else
    if (s_meta.fw_size < 16U || s_meta.fw_size > BOOT_FLASH_APP_SIZE) {
        return false;
    }
    if (!boot_sum_matches(BOOT_PARTITION_A, s_meta.fw_size,
            s_meta.fw_checksum)) {
        return false;
    }
    return boot_vector_sane(boot_flash_partition_addr(BOOT_PARTITION_A));
#endif
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

/* --- 跳转 / 复位 / 同步 --- */

static void boot_jump_to_app(void)
{
    const uint32_t app_addr = boot_flash_partition_addr(BOOT_PARTITION_A);
    const uint32_t sp = *(volatile uint32_t*)app_addr;
    const uint32_t pc = *(volatile uint32_t*)(app_addr + 4U);

    boot_log_sync();

    __disable_irq();

    /* 跳转前恢复中断使能（清 PRIMASK），否则 App 继承全局屏蔽导致异常；
       同时关 Boot 的 SysTick 并清所有挂起中断，避免误进 Boot 处理函数 */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    for (uint32_t i = 0U; i < 8U; i++) {
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))pc)();
    for (;;) {
        __NOP();
    }
}

static void boot_reset_to_normal(void)
{
    boot_log_sync();
    drv_system_reset();
    for (;;) {
        __NOP();
    }
}

/**
 * @brief 同步排空日志：把 log FIFO 全部发出并等 UART1 DMA 真正发送完成
 * @note  必须按时间等待（DMA 非阻塞），不可仅快速轮询，否则跳转/复位会截断日志。
 *        调用时中断须处于使能状态（依赖 SysTick 计时）。
 */
static void boot_log_sync(void)
{
    const uint32_t t0 = millis();
    for (;;) {
        log_task_poll(); /* FIFO → UART（仅空闲时启动一段 DMA） */
        if ((log_tx_len() == 0U) && !drv_log_uart_is_tx_busy()) {
            break;
        }
        if ((uint32_t)(millis() - t0) > BOOT_LOG_SYNC_MS) {
            break;
        }
    }
}

/** @brief 等 485 TX 排空（END 应答上总线后再提交/复位） */
static void boot_tx_sync(void)
{
    const uint32_t t0 = millis();
    while (dev_rs485_is_tx_busy()) {
        dev_rs485_tx_flush();
        if ((uint32_t)(millis() - t0) > BOOT_TX_SYNC_MS) {
            break;
        }
    }
}

/* --- 会话回调 --- */

static void boot_tx(const uint8_t* frame, uint32_t len)
{
    (void)dev_rs485_send(frame, len);
}

/**
 * @brief 提交升级：写提交标志(flag=2) → B→A 提升 → 清标志 → 复位
 *
 * 步骤间任意断电均可在下次上电自愈（flag==1/2 语义见文件头）。
 */
static void boot_commit(void* user, uint32_t size, uint32_t checksum)
{
    (void)user;

    if (size < 16U || size > BOOT_FLASH_APP_SIZE) {
        boot_abort(NULL);
        return;
    }

    s_meta.upgrade_flag = 2U;
    s_meta.boot_partition = (uint8_t)BOOT_PARTITION_A;
    s_meta.version = (uint16_t)(s_meta.version + 1U);
    s_meta.fw_size = size;
    s_meta.fw_checksum = checksum;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);

    BOOT_LOG_I("boot_task", "提交 %lu/0x%08lX",
        (unsigned long)size, (unsigned long)checksum);
    boot_log_sync();
    boot_tx_sync();

    if (boot_flash_promote_to_a(&s_flash_ctx, BOOT_PARTITION_B, size)
        != BOOT_FLASH_OK) {
        BOOT_LOG_E("boot_task", "提升失败");
        s_meta.upgrade_flag = 0U;
        (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
        boot_reset_to_normal();
        return;
    }

    s_meta.upgrade_flag = 0U;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    BOOT_LOG_I("boot_task", "升级成功");
    boot_led_set(true);
    boot_log_sync();
    drv_system_reset();
    for (;;) {
    }
}

/** @brief 中止升级会话：清标志 → 复位（回 A 或重新进入升级态） */
static void boot_abort(void* user)
{
    (void)user;
    s_meta.upgrade_flag = 0U;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    BOOT_LOG_W("boot_task", "中止");
    boot_reset_to_normal();
}

static void boot_id(void* user, uint8_t id)
{
    (void)user;
    s_meta.reserved = (s_meta.reserved & 0xFFFFFF00U) | (uint32_t)id;
    (void)boot_flash_write_metadata(&s_flash_ctx, &s_meta);
    BOOT_LOG_I("boot_task", "ID=0x%02X", (unsigned)id);
}
