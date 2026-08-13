/**
 * @file    srv_ht_test_mode.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-13
 * @brief   苇熠(HT) 伺服执行器测试模式选择宏
 *
 * can_task / srv_can 按此宏接线：激活哪一个 HT 测试模块参与 CAN 收发。
 * 两个模块的源文件始终编译（不激活者不参与收发），切换只需改这一个宏。
 */

#ifndef __SRV_HT_TEST_MODE_H
#define __SRV_HT_TEST_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 苇熠测试模式选择
 *        1 = srv_ht_torque_test — 位置模式往复耐久测试（±180°，到达即反向，累计在线 30 天）
 *        0 = srv_ht_temp_test   — 速度模式原测试（正转/停留/反转/停留，累计在线 24h）
 * @note  两个测试模块共享 CAN 总线（设备地址寻址），同一时刻只允许激活一个；
 *        本默认值可在编译命令行用 -DSRV_HT_TEST_MODE_TORQUE=1 覆盖
 */
#ifndef SRV_HT_TEST_MODE_TORQUE
#define SRV_HT_TEST_MODE_TORQUE 1
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SRV_HT_TEST_MODE_H */
