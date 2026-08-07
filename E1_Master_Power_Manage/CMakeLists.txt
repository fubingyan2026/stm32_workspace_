cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME E1_Master_Power_Manage)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ==============================================================================
# 双镜像目标
#   - ${CMAKE_PROJECT_NAME} : App 镜像（主控电源固件，链接于 0x08020000 分区）
#   - E1_Boot               : Boot 镜像（启动决策 + CAN 升级接收，链接于 0x08000000）
# 分区布局见 docs/boot_upgrade.md。
# ==============================================================================
add_executable(${CMAKE_PROJECT_NAME})

set(E1_BOOT_TARGET E1_Boot)
add_executable(${E1_BOOT_TARGET})

# Add STM32CubeMX generated sources (attach 到 App 与 Boot 两个目标)
add_subdirectory(cmake/stm32cubemx)

# Add middleware (platform-independent static library via public_layer junction)
add_subdirectory(../public_layer/m_middlewares m_middlewares)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 批量收集应用层源代码（middleware 已由 m_middlewares 库管理）
aux_source_directory(${CMAKE_SOURCE_DIR}/tasks TASKS)
aux_source_directory(${CMAKE_SOURCE_DIR}/device_drivers DEV_DRV)
aux_source_directory(${CMAKE_SOURCE_DIR}/../public_layer/device_drivers/hal_flash DEV_DRV_HAL_FLASH)

aux_source_directory(${CMAKE_SOURCE_DIR}/service SERVICE)
aux_source_directory(${CMAKE_SOURCE_DIR}/applications APP)

# ============================ App 目标 =====================================
# Boot 任务/服务已移至 public_layer/service/boot 与 public_layer/task，
# 仅由 Boot 目标显式编译，App 的 aux_source_directory 不会收录。

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${DEV_DRV}
    ${DEV_DRV_HAL_FLASH}
    ${SERVICE}
    ${APP}
    ${TASKS}
    # 共享信号服务（public_layer/service，App 的 LED/蜂鸣器使用）
    ${CMAKE_SOURCE_DIR}/../public_layer/service/srv_signal.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    tasks
    device_drivers
    service
    applications
    ../public_layer/device_drivers/hal_flash
    ../public_layer/service
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    HAL_FLASH_CHIP_STM32F4
    # mpaland_printf: STM32F407 FPv4-SP 无硬件双精度，禁用浮点输出
    PRINTF_DISABLE_SUPPORT_FLOAT
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
    # App 向量表重定位到 0x08020000（system_stm32f4xx.c 于 SystemInit 内设置 SCB->VTOR）
    USER_VECT_TAB_ADDRESS
    VECT_TAB_OFFSET=0x00020000U
)

# App 链接脚本与 map（App 分区 0x08020000）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-T,${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m_middlewares

    # Add user defined libraries
)

# 生成 App Bin/Hex
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -Oihex $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -Obinary $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating hex || bin file for App"
)

# ============================ Boot 目标 ====================================
# Boot 专属源（显式列表）：升级服务/boot 任务取自 public_layer（跨工程复用），
# 其余为 E1 最小驱动集 + srv_signal + led_task；Core 由 stm32cubemx 挂接。
set(BOOT_SOURCES
    ${CMAKE_SOURCE_DIR}/../public_layer/service/boot/boot_transport.c
    ${CMAKE_SOURCE_DIR}/../public_layer/service/boot/boot_fsm.c
    ${CMAKE_SOURCE_DIR}/../public_layer/service/boot/boot_flash.c
    ${CMAKE_SOURCE_DIR}/../public_layer/task/boot_task.c
    ${CMAKE_SOURCE_DIR}/../public_layer/service/srv_signal.c
    ${CMAKE_SOURCE_DIR}/tasks/app_main.c
    ${CMAKE_SOURCE_DIR}/tasks/log_task.c
    ${CMAKE_SOURCE_DIR}/tasks/led_task.c
    ${CMAKE_SOURCE_DIR}/device_drivers/drv_systick.c
    ${CMAKE_SOURCE_DIR}/device_drivers/drv_log_uart.c
    ${CMAKE_SOURCE_DIR}/device_drivers/drv_can.c
    ${CMAKE_SOURCE_DIR}/device_drivers/drv_led.c
)

target_sources(${E1_BOOT_TARGET} PRIVATE
    ${BOOT_SOURCES}
    ${DEV_DRV_HAL_FLASH}
)

target_include_directories(${E1_BOOT_TARGET} PRIVATE
    tasks
    device_drivers
    service
    applications
    ../public_layer/device_drivers/hal_flash
    ../public_layer/service
    ../public_layer/service/boot
    ../public_layer/task
)

target_compile_definitions(${E1_BOOT_TARGET} PRIVATE
    E1_BUILD_BOOT
    HAL_FLASH_CHIP_STM32F4
    PRINTF_DISABLE_SUPPORT_FLOAT
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
)

# Boot 链接脚本与 map（Boot 区 0x08000000）
target_link_options(${E1_BOOT_TARGET} PRIVATE
    -Wl,-T,${CMAKE_SOURCE_DIR}/STM32F407XX_BOOT.ld
    -Wl,-Map=${E1_BOOT_TARGET}.map
)

target_link_libraries(${E1_BOOT_TARGET}
    stm32cubemx
    m_middlewares
)

# 生成 Boot Bin/Hex
add_custom_command(
    TARGET ${E1_BOOT_TARGET}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} $<TARGET_FILE:${E1_BOOT_TARGET}> -Oihex $<TARGET_FILE_DIR:${E1_BOOT_TARGET}>/${E1_BOOT_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} $<TARGET_FILE:${E1_BOOT_TARGET}> -Obinary $<TARGET_FILE_DIR:${E1_BOOT_TARGET}>/${E1_BOOT_TARGET}.bin
    COMMENT "Generating hex || bin file for Boot"
)
