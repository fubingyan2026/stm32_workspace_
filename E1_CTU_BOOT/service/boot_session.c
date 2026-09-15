/**
 * @file    boot_session.c
 * @brief   升级会话实现（Boot/App 共用下载状态机）
 *
 * 见 boot_session.h 的流程说明。此处只做三件事：
 *   1) START：校验大小并擦除暂存槽；
 *   2) DATA ：按 blk×248 偏移写入并回读校验，累计 32 位累加和；
 *   3) END  ：长度核对后回调 on_commit，交由调用方决定提交策略。
 */

/* Includes ------------------------------------------------------------------*/
#include "boot_session.h"

#include <string.h>

/* 本模块日志/进度输出默认关闭（Boot 32K 分区余量极小）：
 * App 侧经 CMake 定义 BOOT_SESSION_LOG_ENABLE=1 打开（含每 10% 进度行）。 */
#ifndef BOOT_SESSION_LOG_ENABLE
#define BOOT_SESSION_LOG_ENABLE (0)
#endif

#if BOOT_SESSION_LOG_ENABLE
#include "log.h"
#define BOOT_SESSION_LOG_E(...) LOG_E("boot_session", __VA_ARGS__)
#define BOOT_SESSION_LOG_I(...) LOG_I("boot_session", __VA_ARGS__)
#else
#define BOOT_SESSION_LOG_E(...) ((void)0)
#define BOOT_SESSION_LOG_I(...) ((void)0)
#endif

/* Private function prototypes -----------------------------------------------*/

static uint8_t session_tx(void* user, const uint8_t* frame, uint32_t len);
static uint8_t session_start(void* user, uint32_t size, uint32_t checksum);
static uint8_t session_data(void* user, uint16_t blk, const uint8_t* data,
    uint32_t len);
static void session_end(void* user, uint32_t size, uint32_t checksum);
static void session_abort(void* user);
static void session_id(void* user, uint8_t id);

/* Exported functions --------------------------------------------------------*/

void boot_session_init(boot_session_t* session, const boot_session_config_t* cfg)
{
    if (!session || !cfg) {
        return;
    }

    memset(session, 0, sizeof(*session));
    session->cfg = cfg;

    session->proto_cfg.tx = session_tx;
    session->proto_cfg.on_start = session_start;
    session->proto_cfg.on_data = session_data;
    session->proto_cfg.on_end = session_end;
    session->proto_cfg.on_abort = session_abort;
    session->proto_cfg.on_id = (cfg->on_id != NULL) ? session_id : NULL;
    session->proto_cfg.user = session;
    session->proto_cfg.my_id = cfg->my_id;

    boot_proto_init(&session->proto, &session->proto_cfg);
}

void boot_session_feed(boot_session_t* session, uint8_t byte)
{
    if (!session || !session->cfg) {
        return;
    }
    boot_proto_feed(&session->proto, byte);
}

bool boot_session_downloading(const boot_session_t* session)
{
    return (session != NULL) && session->proto.downloading;
}

/* Private functions ---------------------------------------------------------*/

static uint8_t session_tx(void* user, const uint8_t* frame, uint32_t len)
{
    boot_session_t* s = (boot_session_t*)user;
    if (!s || !s->cfg || !s->cfg->tx) {
        return 1U;
    }
    s->cfg->tx(frame, len);
    return 0U;
}

static uint8_t session_start(void* user, uint32_t size, uint32_t checksum)
{
    boot_session_t* s = (boot_session_t*)user;

    if (!s || !s->cfg || !s->cfg->flash) {
        return BOOT_PROTO_ERR_STATE;
    }
    if (size < 16U || size > BOOT_FLASH_APP_SIZE) {
        return BOOT_PROTO_ERR_SIZE;
    }
    if (boot_flash_erase_partition(s->cfg->flash, s->cfg->target)
        != BOOT_FLASH_OK) {
        BOOT_SESSION_LOG_E("擦除暂存槽失败");
        return BOOT_PROTO_ERR_FLASH;
    }

    s->upg_offset = 0U;
    s->upg_checksum = 0U;
    BOOT_SESSION_LOG_I("START %lu/0x%08lX",
        (unsigned long)size, (unsigned long)checksum);
    return BOOT_PROTO_ERR_NONE;
}

static uint8_t session_data(void* user, uint16_t blk, const uint8_t* data,
    uint32_t len)
{
    boot_session_t* s = (boot_session_t*)user;
    const uint32_t offset = (uint32_t)blk * BOOT_PROTO_DATA_MAX;

    if (!s || !s->cfg || !s->cfg->flash) {
        return BOOT_PROTO_ERR_STATE;
    }
    if (len == 0U || (offset + len) > BOOT_FLASH_APP_SIZE) {
        return BOOT_PROTO_ERR_SIZE;
    }
    if (boot_flash_write_block(s->cfg->flash, s->cfg->target, offset, data, len)
        != BOOT_FLASH_OK) {
        return BOOT_PROTO_ERR_FLASH;
    }
    if (boot_flash_verify_block(s->cfg->flash, s->cfg->target, offset, data,
            len)
        != BOOT_FLASH_OK) {
        return BOOT_PROTO_ERR_FLASH;
    }

    for (uint32_t i = 0U; i < len; i++) {
        s->upg_checksum += data[i];
    }
    s->upg_offset = offset + len;
    return BOOT_PROTO_ERR_NONE;
}

static void session_end(void* user, uint32_t size, uint32_t checksum)
{
    boot_session_t* s = (boot_session_t*)user;
    (void)checksum;

    if (!s || !s->cfg) {
        return;
    }
    if (size != s->upg_offset) {
        BOOT_SESSION_LOG_E("END 长度不符 (%lu != %lu)",
            (unsigned long)size, (unsigned long)s->upg_offset);
        if (s->cfg->on_abort) {
            s->cfg->on_abort(s->cfg->user);
        }
        return;
    }
    if (s->cfg->on_commit) {
        s->cfg->on_commit(s->cfg->user, s->upg_offset, s->upg_checksum);
    }
}

static void session_abort(void* user)
{
    boot_session_t* s = (boot_session_t*)user;
    if (s && s->cfg && s->cfg->on_abort) {
        s->cfg->on_abort(s->cfg->user);
    }
}

static void session_id(void* user, uint8_t id)
{
    boot_session_t* s = (boot_session_t*)user;
    if (s && s->cfg && s->cfg->on_id) {
        s->cfg->on_id(s->cfg->user, id);
    }
}
