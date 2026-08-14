/**
 * @file    srv_log_flash.h
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-08-12
 * @brief   警告/错误日志 Flash 持久化服务 — log 落盘钩子 + ring_storage 逐条日志
 * @attention
 *
 * 将 log 模块中 WARN/ERROR 级别的日志行持久化到 Flash，掉电不丢失。
 * 采用"每条日志 = 一个 ring_storage 帧"模型：
 *   - 注册 1 个 KV（key = "logs"），value 为单条记录（固定大小）
 *   - 每次落盘写入一帧（仅一条日志），帧版本号全局单调递增
 *   - 28KB 区域退化为真环形日志：帧小 → 每扇区容纳 30 帧，
 *     写满整个区域才回绕擦除最旧扇区，实际保留 204 条
 *
 * 接线方式（log.c 提供落盘钩子，本服务注册回调）：
 *   log_log(WARN/ERROR) → log_format_output() → srv_log_flash_sink()（ISR 安全入队）
 *   → srv_log_flash_step()（主循环逐条落盘，限流）→ ring_storage_save()（每条一帧）
 *
 * 读取方式：串口（USART1 控制台）发送命令触发打印，见 log_task.c 命令解析。
 *  dump 遍历全部有效帧，按版本号升序（= 时间序）输出；clear 整区擦除。
 */

#ifndef __SRV_LOG_FLASH_H
#define __SRV_LOG_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "log.h"
#include "ring_storage.h" /* RING_STORAGE_SECTOR_4K 等 */

/* Exported constants --------------------------------------------------------*/

/**
 * @brief 总开关（可在 CMake 编译宏中置 0 关闭本服务）
 */
#ifndef SRV_LOG_FLASH_ENABLE
#define SRV_LOG_FLASH_ENABLE 1
#endif

/**
 * @brief Flash 存储区域 — 0x08019000 ~ 0x08020000 (28KB = 7 × 4KB 扇区)
 *
 * @note 保留容量 ≈ 区域大小 / 帧体积 ≈ 204 条。
 *       4KB 逻辑扇区 = G4 单 Bank 一个物理页，擦除 1:1。
 *       起点 0x08019000 为 4KB 对齐；终点 0x08020000 恰为 128KB Flash 边界
 *       （驱动 FLASH_TOTAL_SIZE 按 128KB 配置；若为 512KB 硅片可上移起点扩容）。
 *       代码区由链接脚本限制在前 100KB (0x08000000~0x08018FFF)，无重叠。
 */
#ifndef SRV_LOG_FLASH_AREA_START
#define SRV_LOG_FLASH_AREA_START (0x08019000UL) /**< 28KB 区域起点（4KB 扇区对齐，终点恰为 128KB Flash 边界） */
#endif
#ifndef SRV_LOG_FLASH_AREA_SIZE
#define SRV_LOG_FLASH_AREA_SIZE (28 * 1024U) /**< 28KB = 7 × 4KB 扇区 */
#endif
#ifndef SRV_LOG_FLASH_SECTOR_SIZE
#define SRV_LOG_FLASH_SECTOR_SIZE RING_STORAGE_SECTOR_4K /**< 4KB 扇区 */
#endif

/**
 * @brief 单条记录文本最大长度（含 E (ts) tag: msg\r\n）
 *
 * @note  需容纳中文（UTF-8 每字 3 字节）+ 时间戳/标签前缀。
 *        当前最长真实日志 ~95B，故取 96。超长消息会被干净截断（补 \r\n + UTF-8 边界）。
 *        调整 LINE_MAX 会改变帧体积 → 改变保留容量（MAX_FRAMES 随帧体积自动核算）。
 */
#ifndef SRV_LOG_FLASH_LINE_MAX
#define SRV_LOG_FLASH_LINE_MAX (96U)
#endif

/**
 * @brief 单条日志在 Flash 中的对齐占用（帧头尾 28B + KV 开销 7B + 记录体积）
 * @note  KV 开销 = key_len(1) + key("logs"=4) + val_len(2) = 7B；
 *        对齐到 STM32G4 双字编程（8B）。
 */
#define SRV_LOG_FLASH_FRAME_FLASH_SIZE \
    (((sizeof(srv_log_flash_record_t) + 35U + 7U) / 8U) * 8U)

/**
 * @brief dump 时收集版本号的容量上限（区域可容纳的最大帧数，加 2 余量）
 * @note  编译期常量：随 AREA_SIZE / FRAME_FLASH_SIZE 自动核算。
 */
#define SRV_LOG_FLASH_MAX_FRAMES \
    (SRV_LOG_FLASH_AREA_SIZE / SRV_LOG_FLASH_FRAME_FLASH_SIZE + 2U)

/**
 * @brief 环形日志最大保留条数（稳态上限）
 * @note  每扇区可容纳 floor(扇区/帧) 帧，N 个扇区装满后相邻扇区
 *        边界各重复 1 帧（GC 搬帧），故上限 = N×每扇区帧数 − (N−1)。
 *        当前 4KB 扇区 × 7 = 204 条。编译期常量，随配置自动核算。
 */
#define SRV_LOG_FLASH_MAX_RECORDS                                          \
    ((SRV_LOG_FLASH_AREA_SIZE / SRV_LOG_FLASH_SECTOR_SIZE)                 \
            * (SRV_LOG_FLASH_SECTOR_SIZE / SRV_LOG_FLASH_FRAME_FLASH_SIZE) \
        - ((SRV_LOG_FLASH_AREA_SIZE / SRV_LOG_FLASH_SECTOR_SIZE) - 1U))

/**
 * @brief 待落盘队列缓冲大小（kfifo，需 2 的幂；每条 98B，1024B 可容纳 10 条）
 *
 * @note  sink（可能 ISR 上下文）只做 RAM 入队，主循环 step 逐条落盘。
 *        队列满时丢弃新到的记录（保留队内既有记录），与 log TX 路径语义一致。
 */
#ifndef SRV_LOG_FLASH_PENDING_BUFFER_SIZE
#define SRV_LOG_FLASH_PENDING_BUFFER_SIZE (1024U)
#endif

/**
 * @brief 落盘限流：两次 Flash 写入最小间隔（防高频错误触发连续擦写）
 * @note  每次落盘仅一条日志（~136B），磨损远低于旧整帧快照模型。
 *        高频错误持续时按该周期丢弃队尾多余的记录。
 */
#ifndef SRV_LOG_FLASH_FLUSH_MIN_MS
#define SRV_LOG_FLASH_FLUSH_MIN_MS (200U)
#endif

/**
 * @brief dump 单条记录最大输出字节（颜色码 5 + 文本 LINE_MAX + 复位码 4）
 * @note  文本长度最大 SRV_LOG_FLASH_LINE_MAX（96），颜色码/复位码为固定 ANSI 序列。
 */
#define SRV_LOG_FLASH_DUMP_RECORD_OUT_MAX (5U + SRV_LOG_FLASH_LINE_MAX + 4U)

/**
 * @brief 流式 dump 背压水位：log TX 剩余空间低于该值暂停读取，等排空后下个周期续传
 * @note  须大于 SRV_LOG_FLASH_DUMP_RECORD_OUT_MAX（单条输出），保证单条完整写入；
 *        默认 128B > 105B，留 23B 余量抵消 ISR 并发写 TX 的抖动。
 */
#ifndef SRV_LOG_FLASH_DUMP_WATERMARK
#define SRV_LOG_FLASH_DUMP_WATERMARK (128U)
#endif

/**
 * @brief 流式 dump 停滞超时：输出通道长时间无法排空（如 LOG_OUTPUT_NONE、UART 停发）时中止 dump
 * @note  正常 UART/RTT 输出下 TX 持续排空不会触发；仅防卡死。
 */
#ifndef SRV_LOG_FLASH_DUMP_STALL_MS
#define SRV_LOG_FLASH_DUMP_STALL_MS (2000U)
#endif

/**
 * @brief 存储布局版本（仅作固件版本标记；实际布局以"记录体积"为准）
 *
 * @note  V6：从"整帧快照（40 条/帧）"改为"每条日志一帧"，保留 204 条。
 *        ring_storage 帧本身有 magic/CRC 校验，旧版本（V5，value 3932B）的帧
 *        因 value 长度与当前注册值不匹配不会被加载，dump 时会自动跳过。
 */
#define SRV_LOG_FLASH_MAGIC (0x4C4F474CUL) /**< "LOGL" */
#define SRV_LOG_FLASH_VERSION (6U)

/**
 * @brief ring_storage KV key
 */
#define SRV_LOG_FLASH_KV_KEY "logs"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单条日志记录（作为 ring_storage 单个 KV 的 value）
 * @note  level 单独保存便于过滤/统计；文本为剥离 ANSI 颜色码后的完整日志行
 *        （含时间戳与 CRLF），dump 时按原样输出。
 *        len 为有效文本长度（不含 '\0'），有效记录 len ≥ 2（至少 "\r\n"），
 *        可用于区分当前布局帧与旧布局帧（旧帧 value 长度不匹配，不会被加载）。
 */
typedef struct {
    uint8_t level; /**< 日志级别 (log_level_t) */
    uint8_t len; /**< 有效文本长度（不含 '\0'），文本上限 < 255 */
    char text[SRV_LOG_FLASH_LINE_MAX]; /**< 格式化日志行（含 \r\n，'\0' 结尾） */
} srv_log_flash_record_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化日志 Flash 存储服务
 * @note  调用 hal_flash_init + ring_storage_init + register（单条记录 KV），
 *        并向 log 模块注册落盘回调（需在 log_task_init 之后调用）。
 *        ring_storage 自带帧校验，无需外部布局魔数检查。
 */
void srv_log_flash_init(void);

/**
 * @brief 周期步进（由 log_task 的 sw_timer 调用，主循环上下文）
 * @note  队列有记录且距上次落盘超过 FLUSH_MIN_MS 时，窥视队首 → save 一条
 *        （成功后才出队，失败保留下次重试）。
 *        sink 在 ISR 上下文只入队，所有 Flash 写都发生在这里。
 */
void srv_log_flash_step(void);

/**
 * @brief 打印 Flash 中存储的日志（经 log 输出通道发往 USART1 控制台）
 * @note  遍历全部有效帧，收集版本号后升序排序（= 时间序），逐条输出。
 *        旧布局帧 / 数据 CRC 损坏帧自动跳过。
 */
void srv_log_flash_dump(void);

/**
 * @brief 流式 dump 步进（由 log_task 的 sw_timer 周期调用，背压续传）
 * @note  dump 启动后每次调用按背压水位输出部分记录：log TX 剩余空间不足
 *        一条记录时等待下个周期（排空续传），输出通道停滞超过 STALL_MS 时
 *        打印中止提示并结束，避免卡死。
 */
void srv_log_flash_dump_step(void);

/**
 * @brief 清空已存储的日志（整区擦除 + 丢弃待落盘队列）
 */
void srv_log_flash_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_LOG_FLASH_H */
