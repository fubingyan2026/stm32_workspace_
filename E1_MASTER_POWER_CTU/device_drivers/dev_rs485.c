/**
 * @file    dev_rs485.c
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-03
 * @brief   RS485 半双工驱动实现（方向控制 DE/RE 封装，底层收发由 drv_uart 承担）
 */

/* Includes ------------------------------------------------------------------*/
#include "dev_rs485.h"

#include "drv_systick.h"
#include "drv_uart.h"
#include "log.h"
#include "main.h"

/* 模块日志开关 ----------------------------------------------------------------*/

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define DEV_RS485_LOG_ENABLE 1

#if DEV_RS485_LOG_ENABLE
#define DEV_RS485_LOG_E(...) LOG_E("dev_rs485", __VA_ARGS__)
#define DEV_RS485_LOG_W(...) LOG_W("dev_rs485", __VA_ARGS__)
#define DEV_RS485_LOG_I(...) LOG_I("dev_rs485", __VA_ARGS__)
#define DEV_RS485_LOG_D(...) LOG_D("dev_rs485", __VA_ARGS__)
#else
#define DEV_RS485_LOG_E(...) ((void)0)
#define DEV_RS485_LOG_W(...) ((void)0)
#define DEV_RS485_LOG_I(...) ((void)0)
#define DEV_RS485_LOG_D(...) ((void)0)
#endif

/* 注：本驱动为通信热路径（send/tx_cplt hook/rx 读取），禁止在热路径打印（自引用回灌）。 */

/* Private constants ---------------------------------------------------------*/

/** @brief 底层通用串口通道（当前仅 USART3/RS485） */
#define RS485_UART_CH (DRV_UART_CH_RS485)

/** @brief DE 建立延时 (us)：EN 拉高到首字节发出前让收发器稳定进入发送态 */
#define DEV_RS485_DE_SETTLE_US (5U)

/* Private variables ---------------------------------------------------------*/

static bool s_initialized;

/* Private function prototypes -----------------------------------------------*/

static void rs485_tx_cplt_hook(drv_uart_channel_t ch);
static void rs485_de_high(void);
static void rs485_de_low(void);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化 RS485 驱动（底层 drv_uart + 方向控制，默认接收方向）
 * @return 操作结果错误码
 */
dev_rs485_error_t dev_rs485_init(void)
{
    if (s_initialized) {
        dev_rs485_deinit();
    }

    /* 默认接收方向：DE/!RE 置低 */
    rs485_de_low();

    if (drv_uart_init() != DRV_UART_OK) {
        s_initialized = false;
        DEV_RS485_LOG_E("底层 drv_uart 初始化失败");
        return DEV_RS485_ERROR_UNINITIALIZED;
    }

    /* 注册 TX 完成 hook：ISR 中在队列排空后释放发送方向 */
    drv_uart_register_tx_cplt_hook(RS485_UART_CH, rs485_tx_cplt_hook);

    s_initialized = true;
    DEV_RS485_LOG_I("RS485 初始化完成 (USART3 + drv_uart)");

    return DEV_RS485_OK;
}

/**
 * @brief 反初始化 RS485 驱动
 */
void dev_rs485_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    drv_uart_deinit_all();
    rs485_de_low();

    s_initialized = false;
    DEV_RS485_LOG_I("RS485 反初始化完成");
}

/**
 * @brief 检查驱动是否已初始化
 * @return true表示已初始化，false表示未初始化
 */
bool dev_rs485_is_initialized(void)
{
    return s_initialized && drv_uart_is_initialized(RS485_UART_CH);
}

/**
 * @brief 非阻塞发送一帧（半双工方向自动控制）
 * @param data 数据指针
 * @param len  数据长度
 * @return 操作结果错误码
 */
dev_rs485_error_t dev_rs485_send(const uint8_t* data, uint32_t len)
{
    if (!data) {
        return DEV_RS485_ERROR_NULL_PTR;
    }

    if (!s_initialized) {
        return DEV_RS485_ERROR_UNINITIALIZED;
    }

    if (len == 0U) {
        return DEV_RS485_OK;
    }

    /* 切到发送方向（DE 高）；若底层空闲会立即开始 DMA，忙则入队待 flush */
    rs485_de_high();

    /* RX→TX 首次驱动总线：等收发器建立发送态（SP3485 DE 建立时间），
       忙/有排队帧时驱动器已在发送态，无需重复等待 */
    if (!drv_uart_is_tx_busy(RS485_UART_CH)
        && drv_uart_tx_pending(RS485_UART_CH) == 0U) {
        delay_us(DEV_RS485_DE_SETTLE_US);
    }

    const drv_uart_error_t err = drv_uart_send(RS485_UART_CH, data, len);
    if (err != DRV_UART_OK) {
        /* 发送失败且既无 DMA 也无排队帧 → 撤销 DE，避免发送方向卡死占用总线 */
        if (!drv_uart_is_tx_busy(RS485_UART_CH)
            && drv_uart_tx_pending(RS485_UART_CH) == 0U) {
            rs485_de_low();
        }
        return DEV_RS485_ERROR_TX_BUSY;
    }

    return DEV_RS485_OK;
}

/**
 * @brief 查询是否正在发送（含底层队列非空判定）
 * @return true表示发送链路忙
 */
bool dev_rs485_is_tx_busy(void)
{
    if (!s_initialized) {
        return false;
    }
    return drv_uart_is_tx_busy(RS485_UART_CH) || drv_uart_tx_pending(RS485_UART_CH) > 0U;
}

/**
 * @brief 排空底层 TX 队列（task 层周期调用）
 */
void dev_rs485_tx_flush(void)
{
    if (!s_initialized) {
        return;
    }

    /* 有帧待发且链路空闲 → 确保发送方向后启动下一帧 */
    if (drv_uart_tx_pending(RS485_UART_CH) > 0U && !drv_uart_is_tx_busy(RS485_UART_CH)) {
        rs485_de_high();
        delay_us(DEV_RS485_DE_SETTLE_US);
        drv_uart_tx_flush(RS485_UART_CH);
    }

    /* flush 未能启动发送（DMA 启动失败/被打断）而队列仍非空 → 先释放方向，
       避免空持总线，下一拍重试时会重新拉高 */
    if (drv_uart_tx_pending(RS485_UART_CH) > 0U && !drv_uart_is_tx_busy(RS485_UART_CH)) {
        rs485_de_low();
    }
}

/**
 * @brief 读取自上次调用以来新收到的字节
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t dev_rs485_rx_read(uint8_t* buf, uint32_t max_len)
{
    if (!buf || max_len == 0U || !s_initialized) {
        return 0;
    }
    return drv_uart_rx_read(RS485_UART_CH, buf, max_len);
}

/**
 * @brief 查询当前可读字节数
 * @return 可读字节数
 */
uint32_t dev_rs485_rx_available(void)
{
    if (!s_initialized) {
        return 0;
    }
    return drv_uart_rx_available(RS485_UART_CH);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief TX DMA 完成 hook（drv_uart ISR 上下文调用）
 * @note  若队列已排空则等 USART TC（移位寄存器发完最后字节）后释放总线方向；
 *        仍有排队帧则保持发送方向，由后续 flush 继续发送。
 */
static void rs485_tx_cplt_hook(drv_uart_channel_t ch)
{
    (void)ch;

    if (!drv_uart_is_tx_busy(RS485_UART_CH) && drv_uart_tx_pending(RS485_UART_CH) == 0U) {
        /* 等 USART TC（移位寄存器发完最后字节）后释放总线方向，避免截断末字节。
           底层 TC 等待由拥有句柄的 drv_uart 提供，避免本驱动耦合具体 UART 句柄 */
        (void)drv_uart_tx_shift_wait(RS485_UART_CH);
        rs485_de_low();
    }
}

static void rs485_de_high(void)
{
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
}

static void rs485_de_low(void)
{
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}
