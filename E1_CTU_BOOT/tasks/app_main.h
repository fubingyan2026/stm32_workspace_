/**
 * @file    app_main.h
 * @brief   应用入口（Bootloader）
 */

#ifndef __APP_MAIN_H
#define __APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bootloader 主入口（CubeMX main() 在 USER CODE 区调用）
 * @note  不会返回：先尝试跳 App，失败则进入 RS485 YMODEM 升级模式
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_MAIN_H */
