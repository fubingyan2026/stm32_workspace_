//
// Created by E1 on 2026-09-15.
//

/**
 * @file    ctu_transport.c
 * @author  E1
 * @version V1.0.0
 * @date    2026-09-15
 * @brief   串口传输层实现（Linux termios + poll）。
 *
 * 采用 O_NONBLOCK + poll() 的同步阻塞风格：调用方 write() 后按需
 * read_frame(timeout) 等待应答，不使用任何线程与动态内存。
 */

/* Includes ------------------------------------------------------------------*/

#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE /* 启用 B460800/B921600、cfmakeraw、ptsname 等 */
#endif

#include "ctu/ctu_transport.h"

#include <string.h>

#if defined(__unix__) || defined(__linux__)
#define CTU_TRANSPORT_POSIX 1
#else
#define CTU_TRANSPORT_POSIX 0
#endif

#if CTU_TRANSPORT_POSIX

#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* Private constants ---------------------------------------------------------*/

#define CTU_TRANSPORT_DEFAULT_WRITE_TIMEOUT_MS 500U /**< 默认写超时 */
#define CTU_TRANSPORT_DEFAULT_POLL_SLICE_MS 2U      /**< 默认轮询切片 */
#define CTU_TRANSPORT_RX_CHUNK 256U                 /**< 单次读取块大小 */

/* Private types -------------------------------------------------------------*/

/**
 * @brief 波特率映射表项
 */
typedef struct {
    uint32_t baud;   /**< 数值波特率 */
    speed_t speed;   /**< termios 速度常量 */
} ctu_transport_baud_map_t;

/* Private variables ---------------------------------------------------------*/

static const ctu_transport_baud_map_t s_ctu_baud_table[] = {
    { 9600U, B9600 },
    { 19200U, B19200 },
    { 38400U, B38400 },
    { 57600U, B57600 },
    { 115200U, B115200 },
    { 230400U, B230400 },
    { 460800U, B460800 },
    { 921600U, B921600 },
};

/* Private function prototypes -----------------------------------------------*/

static bool ctu_transport_lookup_speed(uint32_t baud, speed_t* speed);
static ctu_error_t ctu_transport_configure(ctu_transport_t* transport, uint32_t baud);
static uint64_t ctu_transport_now_ms(void);
static void ctu_transport_record_error(ctu_transport_t* transport, int error_code);
static void ctu_transport_resolve_port(const char* port, char* out, size_t capacity);

/* Exported functions --------------------------------------------------------*/

ctu_error_t ctu_transport_init(ctu_transport_t* transport,
                               const ctu_transport_config_t* config)
{
    if (transport == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    memset(transport, 0, sizeof(*transport));
    transport->fd = -1;

    if (config != NULL) {
        transport->config = *config;
    }
    if (transport->config.write_timeout_ms == 0U) {
        transport->config.write_timeout_ms = CTU_TRANSPORT_DEFAULT_WRITE_TIMEOUT_MS;
    }
    if (transport->config.poll_slice_ms == 0U) {
        transport->config.poll_slice_ms = CTU_TRANSPORT_DEFAULT_POLL_SLICE_MS;
    }

    ctu_protocol_parser_init(&transport->parser);
    transport->initialized = true;

    return CTU_OK;
}

void ctu_transport_deinit(ctu_transport_t* transport)
{
    if (transport == NULL) {
        return;
    }

    ctu_transport_close(transport);
    transport->initialized = false;
}

bool ctu_transport_is_initialized(const ctu_transport_t* transport)
{
    return (transport != NULL) && transport->initialized;
}

ctu_error_t ctu_transport_open(ctu_transport_t* transport, const char* port,
                               uint32_t baud)
{
    int fd;
    speed_t speed;
    char resolved[CTU_PORT_NAME_MAX];
    size_t name_len;

    if ((transport == NULL) || (port == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!transport->initialized) {
        return CTU_ERROR_UNINITIALIZED;
    }
    if (!ctu_transport_lookup_speed(baud, &speed)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    /* 允许简写设备名（如 "ttyUSB0" → "/dev/ttyUSB0"） */
    ctu_transport_resolve_port(port, resolved, sizeof(resolved));
    name_len = strlen(resolved);
    if ((name_len == 0U) || (name_len >= sizeof(transport->port))) {
        return CTU_ERROR_INVALID_PARAM;
    }

    errno = 0;
    fd = open(resolved, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        ctu_transport_record_error(transport, errno);
        return CTU_ERROR_PORT;
    }

    transport->fd = fd;
    transport->baud = baud;
    memcpy(transport->port, resolved, name_len);
    transport->port[name_len] = '\0';
    transport->opened = true;
    transport->rx_frames = 0U;
    transport->last_errno = 0;
    transport->last_error[0] = '\0';
    ctu_protocol_parser_init(&transport->parser);

    if (ctu_transport_configure(transport, baud) != CTU_OK) {
        ctu_transport_record_error(transport, errno);
        ctu_transport_close(transport);
        return CTU_ERROR_PORT;
    }

    return CTU_OK;
}

void ctu_transport_close(ctu_transport_t* transport)
{
    if (transport == NULL) {
        return;
    }

    if (transport->fd >= 0) {
        (void)close(transport->fd);
        transport->fd = -1;
    }
    transport->opened = false;
    ctu_protocol_parser_init(&transport->parser);
}

bool ctu_transport_is_open(const ctu_transport_t* transport)
{
    return (transport != NULL) && transport->opened && (transport->fd >= 0);
}

const char* ctu_transport_port_name(const ctu_transport_t* transport)
{
    if ((transport == NULL) || (transport->port[0] == '\0')) {
        return "";
    }
    return transport->port;
}

int ctu_transport_last_errno(const ctu_transport_t* transport)
{
    return (transport == NULL) ? 0 : transport->last_errno;
}

const char* ctu_transport_last_error_text(const ctu_transport_t* transport)
{
    if (transport == NULL) {
        return "";
    }

    return transport->last_error;
}

ctu_error_t ctu_transport_write(ctu_transport_t* transport, const uint8_t* data,
                                size_t length, uint32_t timeout_ms)
{
    size_t written = 0U;
    uint32_t wait_ms;
    uint64_t deadline_ms;

    if ((transport == NULL) || (data == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_transport_is_open(transport)) {
        return CTU_ERROR_NOT_OPEN;
    }

    wait_ms = (timeout_ms == 0U) ? transport->config.write_timeout_ms : timeout_ms;
    deadline_ms = ctu_transport_now_ms() + (uint64_t)wait_ms;

    while (written < length) {
        ssize_t count = (ssize_t)write(transport->fd, &data[written], length - written);

        if (count > 0) {
            written += (size_t)count;
            continue;
        }
        if ((count < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK)
            && (errno != EINTR)) {
            return CTU_ERROR_IO;
        }

        /* 写缓冲暂满：等待可写直到超时 */
        if (ctu_transport_now_ms() >= deadline_ms) {
            return CTU_ERROR_TIMEOUT;
        }
        {
            struct pollfd pfd;
            pfd.fd = transport->fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            (void)poll(&pfd, 1, (int)transport->config.poll_slice_ms);
        }
    }

    return CTU_OK;
}

ctu_error_t ctu_transport_flush_input(ctu_transport_t* transport)
{
    if (transport == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_transport_is_open(transport)) {
        return CTU_ERROR_NOT_OPEN;
    }

    (void)tcflush(transport->fd, TCIFLUSH);
    ctu_protocol_parser_init(&transport->parser);

    return CTU_OK;
}

ctu_error_t ctu_transport_read_frame(ctu_transport_t* transport, uint8_t* frame_out,
                                     size_t capacity, size_t* frame_len,
                                     uint32_t timeout_ms)
{
    uint8_t chunk[CTU_TRANSPORT_RX_CHUNK];
    uint64_t deadline_ms;

    if ((transport == NULL) || (frame_out == NULL) || (frame_len == NULL)) {
        return CTU_ERROR_NULL_PTR;
    }
    if (!ctu_transport_is_open(transport)) {
        return CTU_ERROR_NOT_OPEN;
    }

    /* 解析缓存里可能已有完整帧（上次读入的多帧） */
    if (ctu_protocol_parser_pop(&transport->parser, frame_out, capacity, frame_len)) {
        transport->rx_frames++;
        return CTU_OK;
    }

    deadline_ms = ctu_transport_now_ms() + (uint64_t)timeout_ms;

    while (true) {
        struct pollfd pfd;
        int ready;
        ssize_t count;

        if (ctu_transport_now_ms() >= deadline_ms) {
            return CTU_ERROR_TIMEOUT;
        }

        pfd.fd = transport->fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        ready = poll(&pfd, 1, (int)transport->config.poll_slice_ms);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            return CTU_ERROR_IO;
        }
        if (ready == 0) {
            continue;
        }

        count = (ssize_t)read(transport->fd, chunk, sizeof(chunk));
        if (count > 0) {
            ctu_error_t result = ctu_protocol_parser_feed(&transport->parser, chunk,
                                                          (size_t)count);
            if (result != CTU_OK) {
                return result;
            }
            if (ctu_protocol_parser_pop(&transport->parser, frame_out, capacity,
                                        frame_len)) {
                transport->rx_frames++;
                return CTU_OK;
            }
        } else if ((count < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK)
                   && (errno != EINTR)) {
            return CTU_ERROR_IO;
        }
    }
}

bool ctu_transport_baud_is_supported(uint32_t baud)
{
    speed_t speed;

    return ctu_transport_lookup_speed(baud, &speed);
}

ctu_error_t ctu_transport_foreach_port(ctu_port_cb_t callback, void* user)
{
    static const char* const patterns[] = {
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyS*",
    };

    if (callback == NULL) {
        return CTU_ERROR_NULL_PTR;
    }

    for (size_t i = 0U; i < (sizeof(patterns) / sizeof(patterns[0])); i++) {
        glob_t result;

        if (glob(patterns[i], 0, NULL, &result) == 0) {
            for (size_t j = 0U; j < result.gl_pathc; j++) {
                callback(result.gl_pathv[j], user);
            }
            globfree(&result);
        }
    }

    return CTU_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 记录失败原因（errno + 可读描述，权限问题附提示）
 */
static void ctu_transport_record_error(ctu_transport_t* transport, int error_code)
{
    const char* reason;

    if (transport == NULL) {
        return;
    }

    transport->last_errno = error_code;
    reason = strerror(error_code);

    if ((error_code == EACCES) || (error_code == EPERM)) {
        (void)snprintf(transport->last_error, sizeof(transport->last_error),
                       "%s（权限不足：可将用户加入 dialout 组，或用 sudo 运行）",
                       (reason != NULL) ? reason : "permission denied");
    } else {
        (void)snprintf(transport->last_error, sizeof(transport->last_error), "%s",
                       (reason != NULL) ? reason : "unknown error");
    }
}

/**
 * @brief 解析串口设备名：不以 '/' 开头时尝试补 "/dev/" 前缀
 * @param port     输入名称（如 "/dev/ttyUSB0" 或 "ttyUSB0"）
 * @param out      输出缓冲
 * @param capacity 输出容量
 */
static void ctu_transport_resolve_port(const char* port, char* out, size_t capacity)
{
    if ((port == NULL) || (out == NULL) || (capacity == 0U)) {
        return;
    }

    if (port[0] == '/') {
        (void)snprintf(out, capacity, "%s", port);
        return;
    }

    /* 简写名：优先 /dev/<name>（存在则采用），否则按原样尝试 */
    char candidate[CTU_PORT_NAME_MAX];

    (void)snprintf(candidate, sizeof(candidate), "/dev/%s", port);
    if (access(candidate, F_OK) == 0) {
        (void)snprintf(out, capacity, "%s", candidate);
    } else {
        (void)snprintf(out, capacity, "%s", port);
    }
}

/**
 * @brief 读取单调时钟（毫秒）
 */
static uint64_t ctu_transport_now_ms(void)
{
    struct timespec now;

    (void)clock_gettime(CLOCK_MONOTONIC, &now);

    return ((uint64_t)now.tv_sec * 1000U) + ((uint64_t)now.tv_nsec / 1000000U);
}

/**
 * @brief 查询波特率对应的 termios 速度常量
 */
static bool ctu_transport_lookup_speed(uint32_t baud, speed_t* speed)
{
    for (size_t i = 0U; i < (sizeof(s_ctu_baud_table) / sizeof(s_ctu_baud_table[0])); i++) {
        if (s_ctu_baud_table[i].baud == baud) {
            if (speed != NULL) {
                *speed = s_ctu_baud_table[i].speed;
            }
            return true;
        }
    }

    return false;
}

/**
 * @brief 配置串口为 8N1 原始模式
 */
static ctu_error_t ctu_transport_configure(ctu_transport_t* transport, uint32_t baud)
{
    struct termios tty;
    speed_t speed;

    if (tcgetattr(transport->fd, &tty) != 0) {
        return CTU_ERROR_PORT;
    }
    if (!ctu_transport_lookup_speed(baud, &speed)) {
        return CTU_ERROR_INVALID_PARAM;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(tcflag_t)CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(tcflag_t)PARENB;
    tty.c_cflag &= ~(tcflag_t)CSTOPB;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    (void)cfsetispeed(&tty, speed);
    (void)cfsetospeed(&tty, speed);

    if (tcsetattr(transport->fd, TCSANOW, &tty) != 0) {
        return CTU_ERROR_PORT;
    }
    (void)tcflush(transport->fd, TCIOFLUSH);

    return CTU_OK;
}

#else /* !CTU_TRANSPORT_POSIX */

ctu_error_t ctu_transport_init(ctu_transport_t* transport,
                               const ctu_transport_config_t* config)
{
    (void)config;
    if (transport == NULL) {
        return CTU_ERROR_NULL_PTR;
    }
    memset(transport, 0, sizeof(*transport));
    transport->fd = -1;
    return CTU_ERROR_NOT_SUPPORTED;
}

void ctu_transport_deinit(ctu_transport_t* transport)
{
    (void)transport;
}

bool ctu_transport_is_initialized(const ctu_transport_t* transport)
{
    (void)transport;
    return false;
}

ctu_error_t ctu_transport_open(ctu_transport_t* transport, const char* port,
                               uint32_t baud)
{
    (void)transport;
    (void)port;
    (void)baud;
    return CTU_ERROR_NOT_SUPPORTED;
}

void ctu_transport_close(ctu_transport_t* transport)
{
    (void)transport;
}

bool ctu_transport_is_open(const ctu_transport_t* transport)
{
    (void)transport;
    return false;
}

const char* ctu_transport_port_name(const ctu_transport_t* transport)
{
    (void)transport;
    return "";
}

int ctu_transport_last_errno(const ctu_transport_t* transport)
{
    (void)transport;
    return 0;
}

const char* ctu_transport_last_error_text(const ctu_transport_t* transport)
{
    (void)transport;
    return "";
}

ctu_error_t ctu_transport_write(ctu_transport_t* transport, const uint8_t* data,
                                size_t length, uint32_t timeout_ms)
{
    (void)transport;
    (void)data;
    (void)length;
    (void)timeout_ms;
    return CTU_ERROR_NOT_SUPPORTED;
}

ctu_error_t ctu_transport_flush_input(ctu_transport_t* transport)
{
    (void)transport;
    return CTU_ERROR_NOT_SUPPORTED;
}

ctu_error_t ctu_transport_read_frame(ctu_transport_t* transport, uint8_t* frame_out,
                                     size_t capacity, size_t* frame_len,
                                     uint32_t timeout_ms)
{
    (void)transport;
    (void)frame_out;
    (void)capacity;
    (void)frame_len;
    (void)timeout_ms;
    return CTU_ERROR_NOT_SUPPORTED;
}

bool ctu_transport_baud_is_supported(uint32_t baud)
{
    (void)baud;
    return false;
}

ctu_error_t ctu_transport_foreach_port(ctu_port_cb_t callback, void* user)
{
    (void)callback;
    (void)user;
    return CTU_ERROR_NOT_SUPPORTED;
}

#endif /* CTU_TRANSPORT_POSIX */
