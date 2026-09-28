/**
 * @file    srv_motor_test_select.h
 * @author  maximillian
 * @version V2.0.0
 * @date    2026-09-28
 * @brief   CAN 总线测试模块统一选择头文件（单一枚举，两条通道自由选择）
 *
 * 所有可在 CAN 总线上运行的测试模块收敛到**一个枚举** srv_motor_test_sel_t，
 * 不再区分 CAN1/CAN2。FDCAN1 与 FDCAN2 各有自己的选择宏：
 *   - SRV_MOTOR_TEST_SELECT        ：CAN1(FDCAN1) 选择
 *   - SRV_MOTOR_TEST_SELECT_CAN2   ：CAN2(FDCAN2) 选择
 * 两者都从同一个枚举中取值，因此任何模块都能放到任意一条通道上执行。
 *
 * 统一枚举（预处理值须与枚举一致）：
 *   - SRV_MOTOR_TEST_NONE      = 0 该通道不运行任何测试
 *   - SRV_MOTOR_TEST_HT_TORQUE = 1 苇熠(HT) 位置模式多圈往复耐久（srv_ht_torque_test）
 *   - SRV_MOTOR_TEST_HT_TEMP   = 2 苇熠(HT) 速度模式正/停/反/停（srv_ht_temp_test）
 *   - SRV_MOTOR_TEST_HT_CAN2   = 3 苇熠(HT) 速度模式多圈往复（srv_ht_can2_torque_test）
 *   - SRV_MOTOR_TEST_TONGZHI   = 4 良志(ODrive) 位置模式梯形往复（srv_tongzhi_torque_test）
 *   - SRV_MOTOR_TEST_TZ_TEMP   = 5 良志(ODrive) 速度/位置模式耐久（srv_tz_temp_test，**多实例**）
 *   - SRV_MOTOR_TEST_PA430     = 6 Motorevo(PA430) MIT 力位混合来回（srv_pa430_torque_test）
 *
 * 通道绑定：
 *   - 单例模块（HT_TORQUE/HT_TEMP/HT_CAN2/TONGZHI/PA430）内部收发统一走各自的
 *     XXX_CAN_CH 宏，其值由"哪条通道选择了它"决定（选在 CAN1 → DRV_CAN_CH_1，
 *     否则 DRV_CAN_CH_2）。单例不能同时在两条通道上选择（本头文件编译期报错）。
 *   - TZ_TEMP 为多实例服务，每实例经 config.can_ch 绑定通道，两条通道可同时各跑一个实例。
 *
 * srv_can.c 仍使用 SRV_MOTOR_TEST_IS_HT_*（兼容宏，按 CAN1 选择判定）解析旧 0x100 协议。
 */

#ifndef __SRV_MOTOR_TEST_SELECT_H
#define __SRV_MOTOR_TEST_SELECT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h" /* DRV_CAN_CH_1 / DRV_CAN_CH_2 */

/* --- 统一枚举：所有 CAN 总线测试模块（不再分 CAN1/CAN2） --- */

typedef enum {
    SRV_MOTOR_TEST_NONE = 0, /* 该通道不运行任何测试 */
    SRV_MOTOR_TEST_HT_TORQUE = 1, /* 苇熠(HT) 位置模式多圈往复耐久（srv_ht_torque_test） */
    SRV_MOTOR_TEST_HT_TEMP = 2, /* 苇熠(HT) 速度模式正/停/反/停（srv_ht_temp_test） */
    SRV_MOTOR_TEST_HT_CAN2 = 3, /* 苇熠(HT) 速度模式多圈往复（srv_ht_can2_torque_test） */
    SRV_MOTOR_TEST_TONGZHI = 4, /* 良志(ODrive) 位置模式梯形往复（srv_tongzhi_torque_test） */
    SRV_MOTOR_TEST_TZ_TEMP = 5, /* 良志(ODrive) 速度/位置耐久（srv_tz_temp_test，多实例） */
    SRV_MOTOR_TEST_PA430 = 6, /* Motorevo(PA430) MIT 力位混合来回（srv_pa430_torque_test） */
    SRV_MOTOR_TEST_PA430_TEMP = 7, /* Motorevo(PA430) MIT 速度模式 正/停/反/停 24H（srv_pa430_temp_test） */
    SRV_MOTOR_TEST_NUM
} srv_motor_test_sel_t;

/* --- 预处理选择值（#if 比较用，值须与枚举一致） --- */

#define SRV_MOTOR_TEST_NONE 0
#define SRV_MOTOR_TEST_HT_TORQUE 1
#define SRV_MOTOR_TEST_HT_TEMP 2
#define SRV_MOTOR_TEST_HT_CAN2 3
#define SRV_MOTOR_TEST_TONGZHI 4
#define SRV_MOTOR_TEST_TZ_TEMP 5
#define SRV_MOTOR_TEST_PA430 6
#define SRV_MOTOR_TEST_PA430_TEMP 7

/* --- 两条通道各自的选择（改这里切换；默认 CAN1=TZ_TEMP，CAN2=TZ_TEMP） --- */

/** @brief CAN1(FDCAN1) 当前测试模块 */
#ifndef SRV_MOTOR_TEST_SELECT
#define SRV_MOTOR_TEST_SELECT SRV_MOTOR_TEST_PA430_TEMP
#endif

/** @brief CAN2(FDCAN2) 当前测试模块 */
#ifndef SRV_MOTOR_TEST_SELECT_CAN2
#define SRV_MOTOR_TEST_SELECT_CAN2 SRV_MOTOR_TEST_NONE
#endif

/* --- 判定宏：某通道是否选择了某模块 --- */

#define SRV_MOTOR_TEST_CH1_IS(v) (SRV_MOTOR_TEST_SELECT == (v))
#define SRV_MOTOR_TEST_CH2_IS(v) (SRV_MOTOR_TEST_SELECT_CAN2 == (v))

/* --- 单例模块冲突检查：同一条模块不能在两条通道同时选择 --- */

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TORQUE) && SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_HT_TORQUE)
#error "SRV_MOTOR_TEST_HT_TORQUE 为单例，不能在 CAN1 与 CAN2 同时选择"
#endif
#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TEMP) && SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_HT_TEMP)
#error "SRV_MOTOR_TEST_HT_TEMP 为单例，不能在 CAN1 与 CAN2 同时选择"
#endif
#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_CAN2) && SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_HT_CAN2)
#error "SRV_MOTOR_TEST_HT_CAN2 为单例，不能在 CAN1 与 CAN2 同时选择"
#endif
#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_TONGZHI) && SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_TONGZHI)
#error "SRV_MOTOR_TEST_TONGZHI 为单例，不能在 CAN1 与 CAN2 同时选择"
#endif
#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_PA430) && SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_PA430)
#error "SRV_MOTOR_TEST_PA430 为单例，不能在 CAN1 与 CAN2 同时选择"
#endif
#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_PA430_TEMP) && SRV_MOTOR_TEST_CH2_IS(SRV_MOTOR_TEST_PA430_TEMP)
#error "SRV_MOTOR_TEST_PA430_TEMP 为单例，不能在 CAN1 与 CAN2 同时选择"
#endif

/* --- 单例模块的通道绑定：选在 CAN1 → CH_1，否则 CH_2 --- */

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TORQUE)
#define SRV_HT_TORQUE_CAN_CH DRV_CAN_CH_1
#else
#define SRV_HT_TORQUE_CAN_CH DRV_CAN_CH_2
#endif

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TEMP)
#define SRV_HT_TEMP_CAN_CH DRV_CAN_CH_1
#else
#define SRV_HT_TEMP_CAN_CH DRV_CAN_CH_2
#endif

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_CAN2)
#define SRV_HT_CAN2_CAN_CH DRV_CAN_CH_1
#else
#define SRV_HT_CAN2_CAN_CH DRV_CAN_CH_2
#endif

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_TONGZHI)
#define SRV_TONGZHI_CAN_CH DRV_CAN_CH_1
#else
#define SRV_TONGZHI_CAN_CH DRV_CAN_CH_2
#endif

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_PA430)
#define SRV_PA430_CAN_CH DRV_CAN_CH_1
#else
#define SRV_PA430_CAN_CH DRV_CAN_CH_2
#endif

#if SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_PA430_TEMP)
#define SRV_PA430_TEMP_CAN_CH DRV_CAN_CH_1
#else
#define SRV_PA430_TEMP_CAN_CH DRV_CAN_CH_2
#endif

/* --- 兼容宏（srv_can.c 旧 0x100 协议按 CAN1 选择判定苇熠模块） --- */

#define SRV_MOTOR_TEST_IS_HT_TORQUE SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TORQUE)
#define SRV_MOTOR_TEST_IS_HT_TEMP SRV_MOTOR_TEST_CH1_IS(SRV_MOTOR_TEST_HT_TEMP)

#ifdef __cplusplus
}
#endif

#endif /* __SRV_MOTOR_TEST_SELECT_H */
