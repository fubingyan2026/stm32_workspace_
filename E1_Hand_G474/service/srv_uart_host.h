/**
 * @file    srv_uart_host.h
 * @brief   USART1 主机二进制定长帧协议服务（20 字节帧，沿用 docs/uart_protocol.md 格式）
 *
 * 帧格式：head(4B 0x1400AA55) + can_id(4B 小端) + data[8] + crc(4B，
 * CRC16_CCITT_FALSE 覆盖 can_id+data 共 12 字节，低字节在前，高 2 字节为 0)。
 * 帧解析与打包复用共享中间件 protocol_parser / protocol_packer。
 *
 * 命令定义（can_id 小端）：
 *   TX（主机→设备）：
 *     0x000001D0  MIT 控制      data = juxie 载荷 Byte[1..8]（pos16 大端 + vel/kp/kd/tq 12bit 打包，同 juxie §4.1）
 *     0x000001D1  电机配置      data[0]=param, data[1..2]=int16 值（param 见 srv_juxie_motor.h）
 *     0x000001D2  电机标零      下发 SDO 写 CAN 0x601 / 0x2531=1，当前角度置零
 *     0x000001E0  读电机反馈    data[0]=电机ID(=1)
 *     0x000001E1  读电机状态    data[0]=电机ID(=1)
 *     0x000000E2  读 Mz         忽略数据
 *     0x000000E3  流式控制      data[0]=0/1, data[1]=间隔 ms(1~1000, 默认 100)
 *     0x000000E4  Mz 标零       触发扭矩传感器通道 1 标零（写 0x4604=1.0）
 *   RX（设备→主机）：
 *     0x00E00100  电机反馈      pos(2B 0.01°) + speed(2B rpm) + iq(2B mA) + tq(2B 0.01Nm)（小端）
 *     0x00E10100  电机状态      err(2B) + temp(2B 0.1℃) + mode(1B) + status(1B) + 预留(2B)
 *     0x00E20000  Mz            mz(4B float32 小端) + seq(2B) + flags(1B) + 预留(1B)
 *     0x00E30000  流帧          mz(4B float32 小端) + pos(2B 0.01°) + tq(2B 0.01Nm)
 *     0x00FF0000  ACK/NAK       data[0]=1/0, data[1]=命令字节回显
 *
 * 日志已切换为 SEGGER RTT，USART1 专属上位机交互命令端口（帧内字节不会被日志打断）。
 */

#ifndef __SRV_UART_HOST_H
#define __SRV_UART_HOST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化主机协议服务（复位帧组装状态与流式配置）
 */
void srv_uart_host_init(void);

/**
 * @brief 周期步进（can_task 1ms 定时器调用）：
 *        读取 USART1 RX，组装并解析 20 字节帧，分发命令，发送应答与流式帧
 */
void srv_uart_host_step(void);

#ifdef __cplusplus
}
#endif

#endif /* __SRV_UART_HOST_H */
