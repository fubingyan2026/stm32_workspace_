/**
 * @file    boot_proto.h
 * @brief   Bootloader RS485 寻址分块升级协议（方案 B，多节点/半双工安全）
 * @attention
 *
 * 复用兄弟工程 z 帧信封（CRC8 覆盖 z..payload，初值 0xFF，多项式 0x8C）：
 *   [ 'z' ][ cmd ][ len ][ payload... ][ CRC8 ][ '\n' ]
 *
 * 关键规则（多节点 485 半双工）：
 *   - 接收端【绝不主动发送】任何字节（取消 YMODEM 的 'C' 心跳），只在被寻址
 *     且帧校验通过后才回 1 帧 → 不会出现多节点同时驱动总线；
 *   - payload[0] = 目标设备 ID（0x01=MASTER / 0x02=SLAVER / 0x00=广播）；
 *     非本机 ID 的帧静默丢弃；本机 ID 未定(0)时接受广播或任意 ID 并学习；
 *   - 固件数据以定长块传输，每块带 16 位块号 + CRC16（信封 CRC8 之外再校验）。
 *
 * 命令（下行 cmd / 上行 cmd|0x80）：
 *   0x06 SELECT  下行 [id][0x01]              上行 [id][err]
 *   0x08 START   下行 [id][size u32][sum u32] 上行 [id][err]
 *   0x09 DATA    下行 [id][blk u16][crc16 u16][data...] 上行 [id][err][blk u16]
 *   0x0A END     下行 [id]                    上行 [id][err] 后提交并复位
 *   0x0B ABORT   下行 [id]                    上行 [id][err] 后放弃并复位
 *
 * 错误码：0x00=OK / 0x01=帧长非法 / 0x02=未选中或状态错 / 0x03=块号错 /
 *         0x04=CRC16 错 / 0x05=Flash 错 / 0x06=长度或容量错
 */

#ifndef __BOOT_PROTO_H
#define __BOOT_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/** @brief 数据块最大长度（决定块偏移步长，收发两端一致）
 *  取 248：≤255 可装进 1 字节 len，且为 4 的倍数（F1 Flash 需字对齐） */
#define BOOT_PROTO_DATA_MAX (248U)

/** @brief 协议命令码 */
typedef enum {
    BOOT_PROTO_CMD_SELECT = 0x06,
    BOOT_PROTO_CMD_START = 0x08,
    BOOT_PROTO_CMD_DATA = 0x09,
    BOOT_PROTO_CMD_END = 0x0A,
    BOOT_PROTO_CMD_ABORT = 0x0B,
} boot_proto_cmd_t;

/** @brief 设备 ID */
#define BOOT_PROTO_ID_MASTER    (0x01U)
#define BOOT_PROTO_ID_SLAVER    (0x02U)
#define BOOT_PROTO_ID_BROADCAST (0x00U)

/** @brief 错误码 */
typedef enum {
    BOOT_PROTO_ERR_NONE = 0x00,
    BOOT_PROTO_ERR_BAD_LEN = 0x01,
    BOOT_PROTO_ERR_STATE = 0x02,
    BOOT_PROTO_ERR_SEQ = 0x03,
    BOOT_PROTO_ERR_CRC = 0x04,
    BOOT_PROTO_ERR_FLASH = 0x05,
    BOOT_PROTO_ERR_SIZE = 0x06,
} boot_proto_err_t;

/* Exported types ------------------------------------------------------------*/

/** @brief 整帧发送回调（经 dev_rs485，需自带半双工方向控制） */
typedef uint8_t (*boot_proto_tx_cb_t)(const uint8_t* frame, uint32_t len);

/** @brief START 回调：擦除暂存槽并准备接收；返回错误码 */
typedef uint8_t (*boot_proto_start_cb_t)(void* user, uint32_t size, uint32_t checksum);

/** @brief DATA 回调：写入第 blk 块（按 blk*DATA_MAX 偏移）；返回错误码 */
typedef uint8_t (*boot_proto_data_cb_t)(void* user, uint16_t blk,
    const uint8_t* data, uint32_t len);

/** @brief END 回调：校验并提交（内部会复位，可不返回） */
typedef void (*boot_proto_end_cb_t)(void* user, uint32_t size, uint32_t checksum);

/** @brief ABORT 回调：放弃升级并复位 */
typedef void (*boot_proto_abort_cb_t)(void* user);

/** @brief 学习到本机 ID 时回调（用于持久化到 metadata.reserved） */
typedef void (*boot_proto_id_cb_t)(void* user, uint8_t id);

/** @brief 协议配置 */
typedef struct {
    boot_proto_tx_cb_t tx;            /**< 整帧发送（必填） */
    boot_proto_start_cb_t on_start;   /**< START（必填） */
    boot_proto_data_cb_t on_data;     /**< DATA（必填） */
    boot_proto_end_cb_t on_end;       /**< END（必填） */
    boot_proto_abort_cb_t on_abort;   /**< ABORT（必填） */
    boot_proto_id_cb_t on_id;         /**< 学习 ID（可空） */
    void* user;                       /**< 用户指针 */
    uint8_t my_id;                    /**< 本机 ID；0=未定（接受广播/任意并学习） */
} boot_proto_config_t;

/** @brief 协议上下文 */
typedef struct {
    const boot_proto_config_t* cfg;

    /* 帧解析：最大帧 = DATA_MAX + 5(负载头) + 5(z/cmd/len/crc/foot) */
    uint8_t rx_buf[BOOT_PROTO_DATA_MAX + 10U];
    uint16_t rx_cnt;
    uint16_t rx_total;   /**< 本帧已收字节数（z,cmd,len,payload,crc,foot） */
    uint8_t rx_state;

    /* 会话 */
    uint8_t my_id;       /**< 本机 ID（可由广播/任意 ID 学习后更新） */
    bool selected;
    bool downloading;
    uint32_t fw_size;
    uint32_t fw_sum;
    uint32_t rx_bytes;
    uint16_t expected_blk;
} boot_proto_context_t;

/* Exported functions prototypes ---------------------------------------------*/

/** @brief 初始化协议 */
void boot_proto_init(boot_proto_context_t* ctx, const boot_proto_config_t* cfg);

/** @brief 喂入一个接收字节（主循环按字节流调用） */
void boot_proto_feed(boot_proto_context_t* ctx, uint8_t byte);

/** @brief 是否已被寻址选中 */
bool boot_proto_selected(const boot_proto_context_t* ctx);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_PROTO_H */
