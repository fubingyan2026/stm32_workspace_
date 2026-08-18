/**
 * @file    srv_motor_test_select.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-15
 * @brief   CAN1(FDCAN1) 电机测试模块统一选择头文件
 *
 * 由 srv_ht_test_mode.h 重命名而来，把「CAN1 上运行哪一个电机测试模块」的选择
 * 统一收敛到此处：一个枚举 + 一个选择宏。can_task / srv_can 只按该枚举接线。
 *
 * 可选模块（同一时刻只激活一个，源文件始终编译）：
 *   - SRV_MOTOR_TEST_HT_TORQUE = 0 苇熠(HT) 位置模式往复耐久测试（srv_ht_torque_test）
 *   - SRV_MOTOR_TEST_HT_TEMP   = 1 苇熠(HT) 速度模式原测试（srv_ht_temp_test）
 *   - SRV_MOTOR_TEST_TONGZHI   = 2 良志(ODrive) 位置模式往复耐久测试（srv_tongzhi_torque_test）
 *
 * 注意：PA430(Motorevo) 测试走 FDCAN2 独立总线，由 srv_pa430_torque_test.h 的
 * SRV_PA430_TORQUE_TEST_ENABLE 单独控制，不参与本选择。
 */

#ifndef __SRV_MOTOR_TEST_SELECT_H
#define __SRV_MOTOR_TEST_SELECT_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- 枚举：表示当前运行的 CAN1 测试模块 --- */

/**
 * @brief CAN1 电机测试模块选择枚举（运行时表示/switch 用）
 */
typedef enum {
    SRV_MOTOR_TEST_SEL_HT_TORQUE = 0, /* 苇熠(HT) 位置模式往复耐久（srv_ht_torque_test） */
    SRV_MOTOR_TEST_SEL_HT_TEMP = 1, /* 苇熠(HT) 速度模式原测试（srv_ht_temp_test） */
    SRV_MOTOR_TEST_SEL_TONGZHI = 2, /* 良志(ODrive) 位置模式往复（srv_tongzhi_torque_test） */
    SRV_MOTOR_TEST_SEL_NUM
} srv_motor_test_sel_t;

/* --- 预处理选择值（#if 比较用，值须与枚举一致） --- */

#define SRV_MOTOR_TEST_HT_TORQUE 0
#define SRV_MOTOR_TEST_HT_TEMP 1
#define SRV_MOTOR_TEST_TONGZHI 2

/* --- 当前选择（命令行 -DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_HT_TORQUE 等可覆盖） --- */

/**
 * @brief 当前激活的 CAN1 测试模块（默认良志 TONGZHI；可用编译宏覆盖）
 * @note  命令行覆盖必须使用上面的宏名或整型字面量（枚举名不是预处理器符号，无法用于 #if）
 */
#ifndef SRV_MOTOR_TEST_SELECT
#define SRV_MOTOR_TEST_SELECT SRV_MOTOR_TEST_HT_TORQUE
#endif

/* --- 便捷判定宏（供 #if 使用） --- */

#define SRV_MOTOR_TEST_IS_HT_TORQUE (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_HT_TORQUE)
#define SRV_MOTOR_TEST_IS_HT_TEMP (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_HT_TEMP)
#define SRV_MOTOR_TEST_IS_TONGZHI (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_TONGZHI)

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_TEST_SELECT_H */
