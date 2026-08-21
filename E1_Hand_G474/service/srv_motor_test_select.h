/**
 * @file    srv_motor_test_select.h
 * @author  maximillian
 * @version V1.0.0
 * @date    2026-08-15
 * @brief   CAN1/CAN2(FDCAN1/FDCAN2) 电机测试模块统一选择头文件
 *
 * 由 srv_ht_test_mode.h 重命名而来，把「CAN1 上运行哪一个电机测试模块」与
 * 「CAN2 上运行哪一个电机测试模块」的选择统一收敛到此处：每路一个枚举 + 一个
 * 选择宏。can_task / srv_can 只按该枚举接线。
 *
 * CAN1 可选模块（SRV_MOTOR_TEST_SELECT，同一时刻只激活一个，源文件始终编译）：
 *   - SRV_MOTOR_TEST_HT_TORQUE = 0 苇熠(HT) 位置模式往复耐久测试（srv_ht_torque_test）
 *   - SRV_MOTOR_TEST_HT_TEMP   = 1 苇熠(HT) 速度模式原测试（srv_ht_temp_test）
 *   - SRV_MOTOR_TEST_TONGZHI   = 2 良志(ODrive) 位置模式往复耐久测试（srv_tongzhi_torque_test）
 *
 * CAN2 可选模块（SRV_MOTOR_TEST_SELECT_CAN2，与 CAN1 并行运行、互不干扰）：
 *   - SRV_MOTOR_TEST_HT_CAN2 = 0 苇熠(HT) 速度模式往复耐久 CAN2 版（srv_ht_can2_torque_test）
 *   - SRV_MOTOR_TEST_PA430   = 1 Motorevo(PA430) MIT 力位混合来回测试（srv_pa430_torque_test）
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

/**
 * @brief CAN2 电机测试模块选择枚举（运行时表示/switch 用）
 */
typedef enum {
    SRV_MOTOR_TEST_SEL_HT_CAN2 = 0, /* 苇熠(HT) 速度模式往复耐久 CAN2 版（srv_ht_can2_torque_test） */
    SRV_MOTOR_TEST_SEL_PA430 = 1, /* Motorevo(PA430) MIT 力位混合来回（srv_pa430_torque_test） */
    SRV_MOTOR_TEST_SEL_CAN2_NUM
} srv_motor_test_can2_sel_t;

/* --- 预处理选择值（#if 比较用，值须与枚举一致） --- */

//can1的选项电机
#define SRV_MOTOR_TEST_HT_TORQUE 0
#define SRV_MOTOR_TEST_HT_TEMP 1
#define SRV_MOTOR_TEST_TONGZHI 2

//can2的选项电机
#define SRV_MOTOR_TEST_HT_CAN2 0
#define SRV_MOTOR_TEST_PA430 1

/* --- 当前选择（命令行 -DSRV_MOTOR_TEST_SELECT=SRV_MOTOR_TEST_HT_TORQUE 等可覆盖） --- */

/**
 * @brief 当前激活的 CAN1 测试模块（默认良志 TONGZHI；可用编译宏覆盖）
 * @note  命令行覆盖必须使用上面的宏名或整型字面量（枚举名不是预处理器符号，无法用于 #if）
 */
#ifndef SRV_MOTOR_TEST_SELECT
#define SRV_MOTOR_TEST_SELECT SRV_MOTOR_TEST_HT_TORQUE
#endif

/**
 * @brief 当前激活的 CAN2 测试模块（默认苇熠 HT_CAN2；可用编译宏覆盖）
 * @note  命令行覆盖必须使用上面的宏名或整型字面量（枚举名不是预处理器符号，无法用于 #if）
 */
#ifndef SRV_MOTOR_TEST_SELECT_CAN2
#define SRV_MOTOR_TEST_SELECT_CAN2 SRV_MOTOR_TEST_HT_CAN2
#endif

/* --- 便捷判定宏（供 #if 使用） --- */

#define SRV_MOTOR_TEST_IS_HT_TORQUE (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_HT_TORQUE)
#define SRV_MOTOR_TEST_IS_HT_TEMP (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_HT_TEMP)
#define SRV_MOTOR_TEST_IS_TONGZHI (SRV_MOTOR_TEST_SELECT == SRV_MOTOR_TEST_TONGZHI)
#define SRV_MOTOR_TEST_IS_HT_CAN2 (SRV_MOTOR_TEST_SELECT_CAN2 == SRV_MOTOR_TEST_HT_CAN2)
#define SRV_MOTOR_TEST_IS_PA430 (SRV_MOTOR_TEST_SELECT_CAN2 == SRV_MOTOR_TEST_PA430)

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_TEST_SELECT_H */
