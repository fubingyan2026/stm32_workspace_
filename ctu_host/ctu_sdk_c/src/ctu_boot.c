//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_boot.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   Boot 固件升级实现（SELECT/START/DATA/END，逐块应答重试）。
 */

/* Includes ------------------------------------------------------------------*/
#include "ctu/ctu_boot.h"

#include <errno.h>
#include <glob.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Private constants ---------------------------------------------------------*/

#define CTU_BOOT_DEFAULT_INVITE_DELAY_MS 500U   /**< 默认邀请后延时 */
#define CTU_BOOT_DEFAULT_SELECT_TIMEOUT_MS 2000U /**< 默认 SELECT 超时 */
#define CTU_BOOT_DEFAULT_BLOCK_TIMEOUT_MS 2000U /**< 默认 DATA 超时 */
#define CTU_BOOT_DEFAULT_END_TIMEOUT_MS 30000U  /**< 默认 END 超时 */
#define CTU_BOOT_DEFAULT_MAX_RETRIES 8U         /**< 默认重试次数 */
#define CTU_BOOT_SUM_CHUNK 512U                 /**< 累加和计算块大小 */
#define CTU_BOOT_HEADER_LEN 16U                 /**< 向量表校验长度 */
#define CTU_BOOT_LOCATE_PATH_MAX 1024U          /**< 固件定位所用路径缓冲长度 */
#define CTU_BOOT_LOCATE_MAX_LEVEL 4U            /**< 向上查找的最大目录层数 */

/* Private function prototypes -----------------------------------------------*/

static void ctu_boot_log(const ctu_boot_context_t* ctx, const char* level,
                         const char* text);
static void ctu_boot_phase(const ctu_boot_context_t* ctx, const char* text);
static void ctu_boot_progress(const ctu_boot_context_t* ctx, float fraction);
static uint32_t ctu_boot_sum_file(FILE* file, uint32_t* out_checksum);
static ctu_error_t ctu_boot_wait_reply(const ctu_boot_context_t* ctx,
                                       ctu_transport_t* transport, uint8_t command,
                                       uint32_t timeout_ms);
static ctu_error_t ctu_boot_send_frame(ctu_transport_t* transport,
                                       const uint8_t* frame, size_t length);
static ctu_error_t ctu_boot_step_error_only(ctu_boot_context_t* ctx,
                                            ctu_transport_t* transport,
                                            uint8_t command, const uint8_t* frame,
                                            size_t length, uint32_t timeout_ms,
                                            const char* name);
static void ctu_boot_send_abort(const ctu_boot_context_t* ctx,
                                ctu_transport_t* transport);
static uint64_t ctu_boot_now_ms(void);
static void ctu_boot_delay_ms(uint32_t delay_ms);

/* Exported functions --------------------------------------------------------*/

ctu_error_t ctu_boot_init(ctu_boot_context_t* ctx, const ctu_boot_config_t* config)
{
    if (ctx == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    memset(ctx, 0, sizeof(*ctx));
    if (config != NULL) {
        ctx->config = *config;
    }
    if (ctx->config.invite_delay_ms == 0U) {
        ctx->config.invite_delay_ms = CTU_BOOT_DEFAULT_INVITE_DELAY_MS;
    }
    if (ctx->config.select_timeout_ms == 0U) {
        ctx->config.select_timeout_ms = CTU_BOOT_DEFAULT_SELECT_TIMEOUT_MS;
    }
    if (ctx->config.block_timeout_ms == 0U) {
        ctx->config.block_timeout_ms = CTU_BOOT_DEFAULT_BLOCK_TIMEOUT_MS;
    }
    if (ctx->config.end_timeout_ms == 0U) {
        ctx->config.end_timeout_ms = CTU_BOOT_DEFAULT_END_TIMEOUT_MS;
    }
    if (ctx->config.max_retries == 0U) {
        ctx->config.max_retries = CTU_BOOT_DEFAULT_MAX_RETRIES;
    }

    ctx->initialized = true;

    return CTU_OK;
}

ctu_error_t ctu_boot_check_image(const uint8_t* data, size_t length,
                                 char* reason, size_t capacity)
{
    uint32_t stack_pointer;
    uint32_t reset_vector;

    if (data == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    if (length < CTU_BOOT_HEADER_LEN) {
        if ((reason != NULL) && (capacity > 0U)) {
            (void)snprintf(reason, capacity, "文件过小，不是有效固件");
        }
        return CTU_ERROR_IMAGE;
    }

    stack_pointer = ctu_protocol_u32_le(&data[0]);
    reset_vector = ctu_protocol_u32_le(&data[4]);

    if ((stack_pointer < CTU_BOOT_RAM_BASE) || (stack_pointer > CTU_BOOT_RAM_END)) {
        if ((reason != NULL) && (capacity > 0U)) {
            (void)snprintf(reason, capacity,
                           "向量表栈顶 0x%08X 不在 RAM 范围（疑似非本 App 固件）",
                           (unsigned int)stack_pointer);
        }
        return CTU_ERROR_IMAGE;
    }
    if (((reset_vector & 0x01U) == 0U)
        || (reset_vector < CTU_BOOT_APP_VECT_ADDR)
        || (reset_vector >= CTU_BOOT_APP_END_ADDR)) {
        if ((reason != NULL) && (capacity > 0U)) {
            (void)snprintf(reason, capacity,
                           "复位向量 0x%08X 不在 AppA 区间 [0x%08X, 0x%08X)",
                           (unsigned int)reset_vector,
                           (unsigned int)CTU_BOOT_APP_VECT_ADDR,
                           (unsigned int)CTU_BOOT_APP_END_ADDR);
        }
        return CTU_ERROR_IMAGE;
    }

    if ((reason != NULL) && (capacity > 0U)) {
        reason[0] = '\0';
    }

    return CTU_OK;
}

ctu_error_t ctu_boot_check_file(const char* filepath, uint32_t* out_size,
                                uint32_t* out_checksum, char* reason,
                                size_t capacity)
{
    FILE* file;
    uint8_t header[CTU_BOOT_HEADER_LEN];
    long file_size;
    uint32_t checksum = 0U;
    ctu_error_t result;

    if (filepath == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    file = fopen(filepath, "rb");
    if (file == NULL) {
        if ((reason != NULL) && (capacity > 0U)) {
            const char* text = strerror(errno);

            (void)snprintf(reason, capacity, "无法打开文件: %s",
                           (text != NULL) ? text : "unknown error");
        }
        return CTU_ERROR_FILE;
    }

    if ((fseek(file, 0, SEEK_END) != 0) || ((file_size = ftell(file)) < 0)
        || (fseek(file, 0, SEEK_SET) != 0)) {
        if ((reason != NULL) && (capacity > 0U)) {
            (void)snprintf(reason, capacity, "文件定位失败: %s", strerror(errno));
        }
        (void)fclose(file);
        return CTU_ERROR_FILE;
    }

    if ((uint32_t)file_size > CTU_FIRMWARE_MAX_SIZE) {
        if ((reason != NULL) && (capacity > 0U)) {
            (void)snprintf(reason, capacity,
                           "超出 App 分区容量 96KB（当前 %ldB）", file_size);
        }
        (void)fclose(file);
        return CTU_ERROR_IMAGE;
    }

    if (fread(header, 1U, sizeof(header), file) != sizeof(header)) {
        if ((reason != NULL) && (capacity > 0U)) {
            (void)snprintf(reason, capacity,
                           "文件过小或读取失败（%ld B），不是有效固件", file_size);
        }
        (void)fclose(file);
        return CTU_ERROR_FILE;
    }

    result = ctu_boot_check_image(header, sizeof(header), reason, capacity);
    if (result != CTU_OK) {
        (void)fclose(file);
        return result;
    }

    if (fseek(file, 0, SEEK_SET) != 0) {
        (void)fclose(file);
        return CTU_ERROR_FILE;
    }
    (void)ctu_boot_sum_file(file, &checksum);
    (void)fclose(file);

    if (out_size != NULL) {
        *out_size = (uint32_t)file_size;
    }
    if (out_checksum != NULL) {
        *out_checksum = checksum;
    }

    return CTU_OK;
}

ctu_error_t ctu_boot_locate_image(const char* name, char* out, size_t capacity)
{
    char pattern[CTU_BOOT_LOCATE_PATH_MAX];
    char cwd[CTU_BOOT_LOCATE_PATH_MAX];
    const char* base;
    bool found = false;

    if ((name == NULL) || (out == NULL) || (capacity == 0U)) {
        return CTU_ERROR_NULL_PTR;
    }
    out[0] = '\0';

    /* 已带路径：只检查可用性 */
    if (strchr(name, '/') != NULL) {
        if (access(name, R_OK) != 0) {
            return CTU_ERROR_FILE;
        }
        (void)snprintf(out, capacity, "%s", name);
        return CTU_OK;
    }

    base = name;
    if (access(base, R_OK) == 0) { /* 当前目录就有 */
        (void)snprintf(out, capacity, "%s", base);
        return CTU_OK;
    }
    if (getcwd(cwd, sizeof(cwd)) == NULL) {
        return CTU_ERROR_FILE;
    }

    /* 从当前目录逐级向上，在 <up>* 的 build 产物目录中按文件名查找 */
    for (uint32_t level = 1U; (level <= CTU_BOOT_LOCATE_MAX_LEVEL) && !found; level++) {
        static const char* const sub_patterns[] = {
            "*/build/*/",   /* 例: <proj>/build/RelWithDebInfo/ */
            "*/build/",     /* 例: <proj>/build/ */
            "build/*/",     /* 例: ./build/RelWithDebInfo/ */
        };
        char prefix[CTU_BOOT_LOCATE_PATH_MAX] = "";

        for (uint32_t i = 0U; i < level; i++) {
            (void)strncat(prefix, "../", sizeof(prefix) - strlen(prefix) - 1U);
        }

        for (size_t s = 0U; s < (sizeof(sub_patterns) / sizeof(sub_patterns[0])); s++) {
            glob_t matches;

            (void)snprintf(pattern, sizeof(pattern), "%s%s%s", prefix,
                           sub_patterns[s], base);
            if (glob(pattern, 0, NULL, &matches) != 0) {
                continue;
            }
            for (size_t m = 0U; m < matches.gl_pathc; m++) {
                const char* candidate = matches.gl_pathv[m];

                if (strstr(candidate, "/CMakeFiles/") != NULL) {
                    continue; /* 跳过 CMake 内部产物 */
                }
                if (access(candidate, R_OK) != 0) {
                    continue;
                }
                /* 优先 RelWithDebInfo / Release，其次第一个匹配 */
                if (!found || (strstr(candidate, "RelWithDebInfo") != NULL)) {
                    (void)snprintf(out, capacity, "%s", candidate);
                    found = true;
                }
                if (strstr(candidate, "RelWithDebInfo") != NULL) {
                    break;
                }
            }
            globfree(&matches);
            if (found) {
                break;
            }
        }
    }

    return found ? CTU_OK : CTU_ERROR_FILE;
}

bool ctu_boot_is_canceled(const ctu_boot_context_t* ctx)
{
    if ((ctx == NULL) || (ctx->cancel == NULL)) {
        return false;
    }

    return (*ctx->cancel) != 0;
}

ctu_error_t ctu_boot_invite(ctu_boot_context_t* ctx, ctu_transport_t* transport)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    size_t length;

    if ((ctx == NULL) || (transport == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctx->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }

    length = ctu_protocol_build_upgrade(ctx->addr, frame, sizeof(frame));
    if (length == 0U) {
        return CTU_ERROR_PROTOCOL;
    }

    return ctu_boot_send_frame(transport, frame, length);
}

ctu_error_t ctu_boot_transfer(ctu_boot_context_t* ctx, ctu_transport_t* transport,
                              const char* filepath)
{
    FILE* file;
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    uint8_t chunk[CTU_BOOT_DATA_MAX];
    uint32_t size = 0U;
    uint32_t checksum = 0U;
    uint32_t offset = 0U;
    uint16_t block = 0U;
    uint32_t total_blocks;
    ctu_error_t result;
    char reason[160];

    if ((ctx == NULL) || (transport == NULL) || (filepath == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctx->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }
    if (!ctu_transport_is_open(transport)) {
        return CTU_ERROR_NOT_OPEN;
    }

    /* 1. 预检固件 */
    result = ctu_boot_check_file(filepath, &size, &checksum, reason, sizeof(reason));
    if (result != CTU_OK) {
        ctu_boot_log(ctx, "error", reason);
        return result;
    }
    ctx->fw_size = size;
    ctx->fw_checksum = checksum;

    file = fopen(filepath, "rb");
    if (file == NULL) {
        return CTU_ERROR_FILE;
    }

    /* 2. 0x06 邀请（App→复位进 Boot / Boot→SELECT，幂等） */
    ctu_boot_phase(ctx, "发送 0x06 邀请 / 等待 Boot");
    result = ctu_boot_invite(ctx, transport);
    if (result != CTU_OK) {
        ctu_boot_log(ctx, "error", "0x06 邀请发送失败");
        (void)fclose(file);
        return result;
    }
    ctu_boot_delay_ms(ctx->config.invite_delay_ms);
    (void)ctu_transport_flush_input(transport);

    /* 3. SELECT */
    ctu_boot_phase(ctx, "选中设备并进入升级会话");
    {
        size_t length = ctu_protocol_build_boot_select(ctx->addr, frame, sizeof(frame));
        result = ctu_boot_step_error_only(ctx, transport, 0x06U, frame, length,
                                          ctx->config.select_timeout_ms, "SELECT");
    }
    if (result != CTU_OK) {
        ctu_boot_log(ctx, "error", "SELECT 失败：未收到 Boot 应答");
        (void)fclose(file);
        return result;
    }
    ctu_boot_log(ctx, "info", "已选中设备并进入升级会话");

    /* 4. START */
    ctu_boot_phase(ctx, "擦除暂存区并开始下载");
    {
        size_t length = ctu_protocol_build_boot_start(ctx->addr, size, checksum,
                                                      frame, sizeof(frame));
        result = ctu_boot_step_error_only(ctx, transport, 0x08U, frame, length,
                                          ctx->config.select_timeout_ms, "START");
    }
    if (result != CTU_OK) {
        ctu_boot_send_abort(ctx, transport);
        (void)fclose(file);
        return result;
    }

    /* 5. DATA 分块传输 */
    ctu_boot_phase(ctx, "传输固件数据");
    total_blocks = (size + CTU_BOOT_DATA_MAX - 1U) / CTU_BOOT_DATA_MAX;
    if (fseek(file, 0, SEEK_SET) != 0) {
        (void)fclose(file);
        ctu_boot_send_abort(ctx, transport);
        return CTU_ERROR_FILE;
    }

    while (offset < size) {
        size_t want = size - offset;
        size_t got;
        uint16_t crc16;
        ctu_error_t block_result = CTU_ERROR_UPGRADE;
        uint8_t attempt;

        if (want > CTU_BOOT_DATA_MAX) {
            want = CTU_BOOT_DATA_MAX;
        }
        got = fread(chunk, 1U, want, file);
        if (got != want) {
            ctu_boot_log(ctx, "error", "读取固件数据失败");
            (void)fclose(file);
            ctu_boot_send_abort(ctx, transport);
            return CTU_ERROR_FILE;
        }
        if (ctu_boot_is_canceled(ctx)) {
            ctu_boot_log(ctx, "warn", "升级被中止");
            (void)fclose(file);
            ctu_boot_send_abort(ctx, transport);
            return CTU_ERROR_CANCELED;
        }

        crc16 = ctu_protocol_crc16_xmodem(chunk, got);
        {
            size_t length = ctu_protocol_build_boot_data(ctx->addr, block, crc16,
                                                         chunk, got, frame,
                                                         sizeof(frame));
            for (attempt = 0U; attempt < ctx->config.max_retries; attempt++) {
                if (ctu_boot_is_canceled(ctx)) {
                    (void)fclose(file);
                    ctu_boot_send_abort(ctx, transport);
                    return CTU_ERROR_CANCELED;
                }
                if (ctu_boot_send_frame(transport, frame, length) != CTU_OK) {
                    continue;
                }
                block_result = ctu_boot_wait_reply(ctx, transport, 0x09U,
                                                   ctx->config.block_timeout_ms);
                if (block_result == CTU_OK) {
                    break;
                }
                ctu_boot_log(ctx, "warn", "块应答异常，重发");
            }
        }
        if (block_result != CTU_OK) {
            ctu_boot_log(ctx, "error", "数据块传输失败");
            (void)fclose(file);
            ctu_boot_send_abort(ctx, transport);
            return CTU_ERROR_UPGRADE;
        }

        offset += (uint32_t)got;
        block++;
        ctu_boot_progress(ctx, (float)offset / (float)size);

        if ((block % 8U) == 0U) {
            char message[64];
            (void)snprintf(message, sizeof(message), "块 %u/%u",
                           (unsigned int)block, (unsigned int)total_blocks);
            ctu_boot_log(ctx, "tx", message);
        }
    }
    (void)fclose(file);
    ctu_boot_log(ctx, "info", "数据全部写入");

    /* 6. END（板端校验并提交） */
    ctu_boot_phase(ctx, "提交固件（校验→提升/写入暂存槽）");
    {
        size_t length = ctu_protocol_build_boot_end(ctx->addr, frame, sizeof(frame));
        if (ctu_boot_send_frame(transport, frame, length) != CTU_OK) {
            return CTU_ERROR_IO;
        }
        result = ctu_boot_wait_reply(ctx, transport, 0x0AU, ctx->config.end_timeout_ms);
    }
    if (result != CTU_OK) {
        ctu_boot_log(ctx, "error", "END 未被确认，升级失败");
        return result;
    }

    ctu_boot_log(ctx, "info", "升级完成");
    ctu_boot_progress(ctx, 1.0f);

    return CTU_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 读取单调时钟（毫秒）
 */
static uint64_t ctu_boot_now_ms(void)
{
    struct timespec now;

    (void)clock_gettime(CLOCK_MONOTONIC, &now);

    return ((uint64_t)now.tv_sec * 1000U) + ((uint64_t)now.tv_nsec / 1000000U);
}

/**
 * @brief 毫秒级延时
 */
static void ctu_boot_delay_ms(uint32_t delay_ms)
{
    struct timespec request;

    request.tv_sec = (time_t)(delay_ms / 1000U);
    request.tv_nsec = (long)((delay_ms % 1000U) * 1000000U);

    while ((nanosleep(&request, &request) != 0) && (errno == EINTR)) {
        /* 被信号打断则继续剩余时间 */
    }
}

/**
 * @brief 发送一帧（内部使用）
 */
static ctu_error_t ctu_boot_send_frame(ctu_transport_t* transport,
                                       const uint8_t* frame, size_t length)
{
    if (length == 0U) {
        return CTU_ERROR_PROTOCOL;
    }

    return ctu_transport_write(transport, frame, length, 0U);
}

/**
 * @brief 累加和计算（返回字节数，checksum 由出参给出）
 */
static uint32_t ctu_boot_sum_file(FILE* file, uint32_t* out_checksum)
{
    uint8_t buffer[CTU_BOOT_SUM_CHUNK];
    uint32_t checksum = 0U;
    uint32_t total = 0U;
    size_t count;

    while ((count = fread(buffer, 1U, sizeof(buffer), file)) > 0U) {
        for (size_t i = 0U; i < count; i++) {
            checksum += buffer[i];
        }
        total += (uint32_t)count;
    }

    if (out_checksum != NULL) {
        *out_checksum = checksum & 0xFFFFFFFFU;
    }

    return total;
}

/**
 * @brief 等待指定命令的应答（忽略总线上其它设备帧）
 * @return CTU_OK 收到且 err=0；CTU_ERROR_DEVICE/CTU_ERROR_STATE 表示 err!=0
 */
static ctu_error_t ctu_boot_wait_reply(const ctu_boot_context_t* ctx,
                                       ctu_transport_t* transport, uint8_t command,
                                       uint32_t timeout_ms)
{
    uint64_t deadline = ctu_boot_now_ms() + (uint64_t)timeout_ms;
    uint8_t expected = (uint8_t)(command | CTU_PROTOCOL_REPLY_FLAG);

    while (ctu_boot_now_ms() < deadline) {
        uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
        size_t length = 0U;
        uint64_t remaining = deadline - ctu_boot_now_ms();
        ctu_error_t result = ctu_transport_read_frame(transport, frame, sizeof(frame),
                                                      &length, (uint32_t)remaining);
        const uint8_t* payload;

        if (result == CTU_ERROR_TIMEOUT) {
            return CTU_ERROR_TIMEOUT;
        }
        if (result != CTU_OK) {
            return result;
        }

        payload = ctu_protocol_frame_payload(frame, length);
        if (ctu_protocol_frame_addr(frame, length) != ctx->addr) {
            continue; /* 其它设备的帧 */
        }
        if (ctu_protocol_frame_cmd(frame, length) == CTU_PROTOCOL_ERR_CMD) {
            return CTU_ERROR_DEVICE;
        }
        if (ctu_protocol_frame_cmd(frame, length) != expected) {
            continue;
        }
        if (payload == NULL) {
            return CTU_ERROR_PROTOCOL;
        }
        if ((ctu_protocol_frame_payload_len(frame, length) < 2U)
            || (payload[1] != (uint8_t)CTU_BOOT_ERR_NONE)) {
            return CTU_ERROR_STATE;
        }
        return CTU_OK;
    }

    return CTU_ERROR_TIMEOUT;
}

/**
 * @brief 发送并按次重试“只回错误码”的命令
 */
static ctu_error_t ctu_boot_step_error_only(ctu_boot_context_t* ctx,
                                            ctu_transport_t* transport,
                                            uint8_t command, const uint8_t* frame,
                                            size_t length, uint32_t timeout_ms,
                                            const char* name)
{
    ctu_error_t result = CTU_ERROR_UPGRADE;

    if (length == 0U) {
        return CTU_ERROR_PROTOCOL;
    }

    for (uint8_t attempt = 0U; attempt < ctx->config.max_retries; attempt++) {
        if (ctu_boot_is_canceled(ctx)) {
            return CTU_ERROR_CANCELED;
        }
        if (ctu_boot_send_frame(transport, frame, length) != CTU_OK) {
            continue;
        }
        result = ctu_boot_wait_reply(ctx, transport, command, timeout_ms);
        if (result == CTU_OK) {
            return CTU_OK;
        }
        if ((result == CTU_ERROR_DEVICE) || (result == CTU_ERROR_STATE)) {
            ctu_boot_log(ctx, "error", name);
            return result;
        }
        ctu_boot_log(ctx, "warn", "应答超时，重试");
    }

    return result;
}

/**
 * @brief 发送 ABORT 结束升级会话
 */
static void ctu_boot_send_abort(const ctu_boot_context_t* ctx,
                                ctu_transport_t* transport)
{
    uint8_t frame[CTU_PROTOCOL_MAX_FRAME];
    size_t length = ctu_protocol_build_boot_abort(ctx->addr, frame, sizeof(frame));

    (void)ctu_boot_send_frame(transport, frame, length);
    ctu_boot_log(ctx, "warn", "已发送 ABORT 中止");
}

/**
 * @brief 日志回调包装
 */
static void ctu_boot_log(const ctu_boot_context_t* ctx, const char* level,
                         const char* text)
{
    if ((ctx == NULL) || (ctx->config.log_cb == NULL) || (text == NULL)) {
        return;
    }

    ctx->config.log_cb(level, text, ctx->config.user);
}

/**
 * @brief 阶段回调包装
 */
static void ctu_boot_phase(const ctu_boot_context_t* ctx, const char* text)
{
    if ((ctx == NULL) || (ctx->config.phase_cb == NULL) || (text == NULL)) {
        return;
    }

    ctx->config.phase_cb(text, ctx->config.user);
}

/**
 * @brief 进度回调包装
 */
static void ctu_boot_progress(const ctu_boot_context_t* ctx, float fraction)
{
    if ((ctx == NULL) || (ctx->config.progress_cb == NULL)) {
        return;
    }
    if (fraction < 0.0f) {
        fraction = 0.0f;
    }
    if (fraction > 1.0f) {
        fraction = 1.0f;
    }

    ctx->config.progress_cb(fraction, ctx->config.user);
}
