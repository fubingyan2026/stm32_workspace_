/**
 * @file    srv_log_flash.c
 * @author  maximillian
 * @version V1.1.0
 * @date    2026-08-12
 * @brief   警告/错误日志 Flash 持久化服务实现
 *
 * 工作流程：
 *   log_log(WARN/ERROR) → 落盘钩子 srv_log_flash_sink（ISR 安全入队 kfifo）
 *   → srv_log_flash_step（主循环逐条落盘，限流）→ ring_storage_save（每条一帧）
 *
 * 逐条落盘模型（V6）：
 *   - 注册 1 个 KV（"logs"），value = 单条记录，每次落盘写入一帧（版本号递增）
 *   - 帧小 → 每扇区容纳 30 帧，写满整个区域才回绕擦最旧扇区 → 保留 204 条
 *   - 帧头/数据 CRC + commit_magic 断电保护，断电最多丢"正在写的那一条"
 *   - sink 可能 ISR 上下文（见 log.c log_format_output）：仅入队 kfifo（SPSC 无锁），
 *     所有 Flash 写都发生在主循环 step，避免在中断里做毫秒级擦写
 */

/* Includes ------------------------------------------------------------------*/
#include "srv_log_flash.h"

#include <string.h>

#include "drv_systick.h"
#include "hal_flash.h"
#include "kfifo.h"
#include "printf.h"
#include "ring_storage_port_hal.h"

/* Private constants ---------------------------------------------------------*/

/**
 * @brief 硬约束：整帧必须 ≤ 扇区一半（ring_storage GC 搬帧后新扇区
 *        需同时容纳旧帧 + 新帧）。单条记录帧体积远小于此，仅作编译期兜底。
 */
_Static_assert(2U * SRV_LOG_FLASH_FRAME_FLASH_SIZE <= SRV_LOG_FLASH_SECTOR_SIZE,
    "srv_log_flash: 帧过大，无法容纳 2 帧/扇区(ring_storage GC 约束)");

/** @brief 本文件日志开关：置 0 屏蔽本文件全部打印 */
#define SRV_LOG_FLASH_LOG_ENABLE 1

#if SRV_LOG_FLASH_LOG_ENABLE
#define SRV_LOG_FLASH_LOG_E(...) LOG_E("log_flash", __VA_ARGS__)
#define SRV_LOG_FLASH_LOG_W(...) LOG_W("log_flash", __VA_ARGS__)
#define SRV_LOG_FLASH_LOG_I(...) LOG_I("log_flash", __VA_ARGS__)
#else
#define SRV_LOG_FLASH_LOG_E(...) ((void)0)
#define SRV_LOG_FLASH_LOG_W(...) ((void)0)
#define SRV_LOG_FLASH_LOG_I(...) ((void)0)
#endif

/** @brief 输出固定字符串到 log 通道（长度由字符串字面量自动计算） */
#define SRV_LOG_FLASH_LOG_STR(s) \
    log_write((const uint8_t*)(s), (uint32_t)(sizeof(s) - 1U))

/** @brief 仅保存 WARN 及以上级别（ERROR=1, WARN=2） */
#define SRV_LOG_FLASH_MIN_LEVEL LOG_LEVEL_WARN

/** @brief 单条记录字节数（kfifo 队列元素大小，98B） */
#define SRV_LOG_FLASH_RECORD_SIZE ((uint32_t)sizeof(srv_log_flash_record_t))

/* Private types -------------------------------------------------------------*/

/**
 * @brief dump 收集的帧元信息（用于按版本排序后逐个读取）
 */
typedef struct {
    uint32_t version; /**< 帧版本号（单调递增，作时间序） */
    uint32_t frame_addr; /**< 帧 Flash 起始地址 */
} srv_log_flash_frame_t;

/* Private variables ---------------------------------------------------------*/

/** @brief ring_storage 帧序列化缓冲区（需 ≥ 最大帧大小） */
static uint8_t s_frame_buf[sizeof(srv_log_flash_record_t) + 64];

/** @brief ring_storage 上下文 */
static ring_storage_context_t s_rs;

/** @brief 单条记录缓冲（ring_storage 的 KV value；sink 不入这里，step 落盘前窥视进这里） */
static srv_log_flash_record_t s_record;

/** @brief 待落盘队列（kfifo SPSC：ISR sink 生产，主循环 step 消费） */
static kfifo_t s_pending;
static uint8_t s_pending_buf[SRV_LOG_FLASH_PENDING_BUFFER_SIZE];

/** @brief 上次成功落盘时间戳 (millis) */
static uint32_t s_last_save_ms = 0;

/** @brief 初始化标志 */
static bool s_initialized = false;

/** @brief dump 收集的帧元信息数组 + 计数 */
static srv_log_flash_frame_t s_dump_frames[SRV_LOG_FLASH_MAX_FRAMES];
static uint32_t s_dump_count = 0;

/** @brief 流式 dump 进行中标志 */
static bool s_dump_active = false;

/** @brief 流式 dump 当前输出索引（指向 s_dump_frames） */
static uint32_t s_dump_index = 0;

/** @brief 流式 dump 最近一次成功输出时间戳 (millis)，用于停滞检测 */
static uint32_t s_dump_last_write_ms = 0;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 剥离 ANSI 颜色转义序列，写入记录文本
 * @param src      原始日志行（含颜色码）
 * @param len      原始日志行长度
 * @param dst      目标文本缓冲
 * @param dst_cap  目标缓冲容量
 * @return 剥离后的有效文本长度
 */
static uint16_t srv_log_flash_strip_ansi(const char* src, uint16_t len,
    char* dst, uint16_t dst_cap);

/**
 * @brief log 模块落盘回调（WARN/ERROR → RAM 待落盘队列，ISR 安全）
 * @param level 日志级别
 * @param line  完整格式化日志行（含 ANSI 颜色码与 CRLF）
 * @param len   日志行长度
 */
static void srv_log_flash_sink(log_level_t level, const char* line, uint16_t len);

/**
 * @brief 记录级别对应的 ANSI 颜色（dump 输出时恢复与实时日志一致的颜色）
 * @param level 日志级别
 * @return ANSI 颜色序列
 */
static const char* srv_log_flash_level_color(log_level_t level);

/**
 * @brief 帧遍历回调：统计有效帧数量
 * @param version    帧版本号
 * @param frame_addr 帧地址
 * @param user_arg   指向 uint32_t 计数的指针
 * @return true 继续遍历
 */
static bool srv_log_flash_count_cb(uint32_t version, uint32_t frame_addr,
    void* user_arg);

/**
 * @brief 帧遍历回调：收集 {version, frame_addr} 到 dump 数组
 * @param version    帧版本号
 * @param frame_addr 帧地址
 * @param user_arg   未使用（遍历中直接写文件静态数组）
 * @return true 继续遍历
 */
static bool srv_log_flash_collect_cb(uint32_t version, uint32_t frame_addr,
    void* user_arg);

/**
 * @brief 对帧数组按 version 升序排序（处理 32 位回绕）
 * @param frames 帧数组
 * @param n      帧数量
 */
static void srv_log_flash_sort_frames(srv_log_flash_frame_t* frames, uint32_t n);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化日志 Flash 存储服务
 */
void srv_log_flash_init(void)
{
#if SRV_LOG_FLASH_ENABLE
    if (s_initialized) {
        return;
    }

    /* hal_flash 幂等，仅首次真正初始化（检测 G4 扇区大小） */
    if (hal_flash_init() != HAL_FLASH_OK) {
        SRV_LOG_FLASH_LOG_E("hal_flash_init 失败，日志落盘功能不可用");
        return;
    }

    /* 待落盘队列初始化（SPSC 无锁，sink 可 ISR 入队） */
    kfifo_init(&s_pending, s_pending_buf, sizeof(s_pending_buf), NULL);

    const ring_storage_config_t cfg = {
        .port = ring_storage_port_hal(),
        .start_addr = SRV_LOG_FLASH_AREA_START,
        .area_size = SRV_LOG_FLASH_AREA_SIZE,
        .sector_size = SRV_LOG_FLASH_SECTOR_SIZE,
        .write_gran = RING_STORAGE_WRITE_GRAN_64, /* STM32G4 双字编程 */
        .frame_buffer = s_frame_buf,
        .frame_buffer_size = (uint16_t)sizeof(s_frame_buf),
    };

    if (ring_storage_init(&s_rs, &cfg) != RING_STORAGE_OK) {
        SRV_LOG_FLASH_LOG_E("ring_storage_init 失败 (addr=0x%08lX, size=%lu)",
            (unsigned long)SRV_LOG_FLASH_AREA_START,
            (unsigned long)SRV_LOG_FLASH_AREA_SIZE);
        return;
    }

    if (ring_storage_register(&s_rs, SRV_LOG_FLASH_KV_KEY,
            &s_record, (uint16_t)sizeof(s_record))
        != RING_STORAGE_OK) {
        SRV_LOG_FLASH_LOG_E("ring_storage_register 失败");
        return;
    }

    /* 统计当前已存储的记录条数（旧布局帧 value 长度不匹配，不会被加载/计数） */
    uint32_t count = 0;
    (void)ring_storage_foreach_frame(&s_rs, srv_log_flash_count_cb, &count);

    /* 注册 log 落盘回调 */
    log_set_flash_sink_cb(srv_log_flash_sink);
    s_initialized = true;

    SRV_LOG_FLASH_LOG_I("初始化完成: 区域=0x%08lX, 已存 %lu 条",
        (unsigned long)SRV_LOG_FLASH_AREA_START,
        (unsigned long)count);
#endif /* SRV_LOG_FLASH_ENABLE */
}

/**
 * @brief 周期步进（由 log_task 的 sw_timer 调用，主循环上下文）
 */
void srv_log_flash_step(void)
{
#if SRV_LOG_FLASH_ENABLE
    if (!s_initialized || kfifo_len(&s_pending) == 0) {
        return;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - s_last_save_ms) < SRV_LOG_FLASH_FLUSH_MIN_MS) {
        return; /* 限流：防高频错误触发连续擦写 */
    }

    /* 窥视队首到 KV value（不弹出），落盘成功后才出队，失败保留下次重试 */
    if (kfifo_peek(&s_pending, (uint8_t*)&s_record,
            SRV_LOG_FLASH_RECORD_SIZE, 0)
        != SRV_LOG_FLASH_RECORD_SIZE) {
        return; /* 数据不完整，等下一个周期 */
    }

    const ring_storage_error_t err = ring_storage_save(&s_rs);
    if (err == RING_STORAGE_OK) {
        (void)kfifo_skip(&s_pending, SRV_LOG_FLASH_RECORD_SIZE);
        s_last_save_ms = now;
    } else {
        SRV_LOG_FLASH_LOG_E("Flash 日志保存失败 (%d)", (int)err);
        /* 保持队首，下个周期重试 */
    }
#endif /* SRV_LOG_FLASH_ENABLE */
}

/**
 * @brief 启动 Flash 日志流式 dump（收集+排序+打印头，逐条输出交给 dump_step 背压续传）
 */
void srv_log_flash_dump(void)
{
#if SRV_LOG_FLASH_ENABLE
    if (!s_initialized) {
        SRV_LOG_FLASH_LOG_STR("(log_flash 未初始化)\r\n");
        return;
    }

    /* 收集全部有效帧的元信息（帧头 magic/CRC + commit_magic 已校验） */
    s_dump_count = 0;
    (void)ring_storage_foreach_frame(&s_rs, srv_log_flash_collect_cb, NULL);

    /* 按版本升序排序（版本单调递增 = 时间顺序，处理 32 位回绕） */
    srv_log_flash_sort_frames(s_dump_frames, s_dump_count);

    if (s_dump_count == 0) {
        SRV_LOG_FLASH_LOG_STR("(无记录)\r\n");
        return;
    }

    /* 屏蔽实时日志，保证 dump 输出连续；dump_step 完成/中止或 clear 时恢复 */
    (void)log_hold_output(true);

    char header[64];
    int n = snprintf_(header, sizeof(header),
        "===== Flash 日志 (WARN/ERROR) %lu/%lu 条 =====\r\n",
        (unsigned long)s_dump_count, (unsigned long)SRV_LOG_FLASH_MAX_RECORDS);
    if (n > 0) {
        log_write((const uint8_t*)header, (uint32_t)n);
    }

    /* 初始化流式输出状态：由 dump_step 按背压逐条续传 */
    s_dump_index = 0;
    s_dump_active = true;
    s_dump_last_write_ms = millis();
#endif /* SRV_LOG_FLASH_ENABLE */
}

/**
 * @brief 流式 dump 步进（由 log_task 的 sw_timer 周期调用，背压续传）
 * @note  仅在 dump 启动后生效。每次调用在 log TX 缓冲空间充足时批量输出记录，
 *        不足则等待下一 tick；输出通道停滞超过 STALL_MS 时中止，防止卡死。
 */
void srv_log_flash_dump_step(void)
{
#if SRV_LOG_FLASH_ENABLE
    if (!s_initialized || !s_dump_active) {
        return;
    }

    const uint32_t now = millis();

    /* 停滞保护：输出通道（如 LOG_OUTPUT_NONE）长时间无法排空时中止 */
    if ((uint32_t)(now - s_dump_last_write_ms) > SRV_LOG_FLASH_DUMP_STALL_MS) {
        SRV_LOG_FLASH_LOG_STR("===== 结束（输出超时中止） =====\r\n");
        s_dump_active = false;
        (void)log_hold_output(false); /* 恢复实时日志 */
        return;
    }

    /* 背压：log TX 剩余空间不足一条记录时等待，下个周期排空后继续 */
    while (s_dump_index < s_dump_count
        && log_tx_avail() >= SRV_LOG_FLASH_DUMP_WATERMARK) {
        /* 重置为无效态：旧布局帧 value 长度不匹配不会被加载，靠 len==0 区分 */
        memset(&s_record, 0, sizeof(s_record));

        if (ring_storage_load_frame(&s_rs, s_dump_frames[s_dump_index].frame_addr)
            != RING_STORAGE_OK) {
            s_dump_index++; /* 数据 CRC 损坏帧，跳过 */
            continue;
        }

        if (s_record.len > 0 && s_record.len <= SRV_LOG_FLASH_LINE_MAX) {
            /* 单条记录组装后单次写入，避免与 ISR 日志并发写 TX 时被拆断 */
            char out[SRV_LOG_FLASH_DUMP_RECORD_OUT_MAX];
            uint16_t o = 0;

            const char* color = srv_log_flash_level_color((log_level_t)s_record.level);
            const uint16_t clen = (uint16_t)strlen(color);
            (void)memcpy(out + o, color, clen);
            o += clen;

            (void)memcpy(out + o, s_record.text, s_record.len);
            o += s_record.len;

            const uint16_t rlen = (uint16_t)strlen(LOG_COLOR_RESET);
            (void)memcpy(out + o, LOG_COLOR_RESET, rlen);
            o += rlen;

            log_write((const uint8_t*)out, o);
        }

        s_dump_index++;
        s_dump_last_write_ms = now;
    }

    if (s_dump_index >= s_dump_count) {
        SRV_LOG_FLASH_LOG_STR("===== 结束 =====\r\n");
        s_dump_active = false;
        (void)log_hold_output(false); /* 恢复实时日志 */
    }
#endif /* SRV_LOG_FLASH_ENABLE */
}

/**
 * @brief 清空已存储的日志（整区擦除 + 丢弃待落盘队列）
 */
void srv_log_flash_clear(void)
{
#if SRV_LOG_FLASH_ENABLE
    if (!s_initialized) {
        return;
    }

    /* 中止进行中的流式 dump（避免 dump_step 读取已擦除区域）并恢复实时日志 */
    s_dump_active = false;
    (void)log_hold_output(false);

    /* 丢弃待落盘队列中的记录 */
    kfifo_reset(&s_pending);

    /* 整区擦除（含最新帧），等价于清空全部历史 */
    const ring_storage_error_t err = ring_storage_wipe(&s_rs);
    if (err != RING_STORAGE_OK) {
        SRV_LOG_FLASH_LOG_E("清空 Flash 日志失败 (%d)", (int)err);
        return;
    }

    s_last_save_ms = millis();
    SRV_LOG_FLASH_LOG_STR("Flash 日志已清空\r\n");
#endif /* SRV_LOG_FLASH_ENABLE */
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 剥离 ANSI 颜色转义序列，写入记录文本
 */
static uint16_t srv_log_flash_strip_ansi(const char* src, uint16_t len,
    char* dst, uint16_t dst_cap)
{
    uint16_t di = 0;
    uint16_t i = 0;

    while (i < len && di < dst_cap - 1) {
        const uint8_t c = (uint8_t)src[i];

        if (c == 0x1B && (i + 1) < len && src[i + 1] == '[') {
            /* 跳过 CSI 序列: ESC [ ... m */
            i += 2;
            while (i < len && src[i] != 'm') {
                i++;
            }
            if (i < len) {
                i++; /* 跳过结束符 'm' */
            }
        } else {
            dst[di++] = (char)c;
            i++;
        }
    }

    dst[di] = '\0';
    return di;
}

/**
 * @brief 记录级别对应的 ANSI 颜色（dump 输出时恢复与实时日志一致的颜色）
 */
static const char* srv_log_flash_level_color(log_level_t level)
{
    switch (level) {
    case LOG_LEVEL_ERROR:
        return LOG_COLOR_RED;
    case LOG_LEVEL_WARN:
        return LOG_COLOR_YELLOW;
    default:
        return LOG_COLOR_RESET;
    }
}

/**
 * @brief log 模块落盘回调（WARN/ERROR → RAM 待落盘队列）
 * @note  预留 2 字节强制补 \r\n：长消息（尤其中文 UTF-8）被截断时仍以换行结尾，
 *        避免 dump 时相邻记录粘连；截断处回退到完整多字节字符边界，避免乱码。
 *        仅在 ISR 上下文做 RAM 入队（kfifo SPSC put），不做任何 Flash 操作。
 */
static void srv_log_flash_sink(log_level_t level, const char* line, uint16_t len)
{
    if (level > SRV_LOG_FLASH_MIN_LEVEL) {
        return; /* 仅持久化 WARN / ERROR */
    }

    if (!s_initialized || line == NULL) {
        return;
    }

    srv_log_flash_record_t rec;

    /* 剥离 ANSI 颜色码；预留 2 字节给强制换行 */
    uint16_t n = srv_log_flash_strip_ansi(line, len, rec.text,
        (uint16_t)(SRV_LOG_FLASH_LINE_MAX - 2));

    /* 末尾是 UTF-8 续字节（0x80~0xBF）说明多字节字符被截断，回退到完整字符边界 */
    while (n > 0 && ((uint8_t)rec.text[n - 1] & 0xC0U) == 0x80U) {
        n--;
    }

    /* 确保以 \r\n 结尾（短行源文本已含换行则跳过） */
    if (n < 2 || rec.text[n - 2] != '\r' || rec.text[n - 1] != '\n') {
        rec.text[n++] = '\r';
        rec.text[n++] = '\n';
    }
    rec.text[n] = '\0';

    rec.len = (uint8_t)n;
    rec.level = (uint8_t)level;

    /* 入队（ISR 安全：kfifo SPSC put，仅写 in 索引）。
       队列满时丢弃新记录（保留队内既有记录，与 log TX 路径语义一致）。 */
    (void)kfifo_put(&s_pending, (const unsigned char*)&rec, SRV_LOG_FLASH_RECORD_SIZE);
}

/**
 * @brief 帧遍历回调：统计有效帧数量
 */
static bool srv_log_flash_count_cb(uint32_t version, uint32_t frame_addr,
    void* user_arg)
{
    (void)version;
    (void)frame_addr;
    (*(uint32_t*)user_arg)++;
    return true;
}

/**
 * @brief 帧遍历回调：收集 {version, frame_addr} 到 dump 数组
 */
static bool srv_log_flash_collect_cb(uint32_t version, uint32_t frame_addr,
    void* user_arg)
{
    (void)user_arg;
    if (s_dump_count < SRV_LOG_FLASH_MAX_FRAMES) {
        s_dump_frames[s_dump_count].version = version;
        s_dump_frames[s_dump_count].frame_addr = frame_addr;
        s_dump_count++;
    }
    return true;
}

/**
 * @brief 对帧数组按 version 升序排序（处理 32 位回绕）
 */
static void srv_log_flash_sort_frames(srv_log_flash_frame_t* frames, uint32_t n)
{
    for (uint32_t i = 1; i < n; i++) {
        const srv_log_flash_frame_t key = frames[i];
        uint32_t j = i;
        while (j > 0 && (int32_t)(frames[j - 1].version - key.version) > 0) {
            frames[j] = frames[j - 1];
            j--;
        }
        frames[j] = key;
    }
}
