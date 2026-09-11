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
set(CMAKE_PROJECT_NAME E1_SLAVER_POWER_CTU)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ==============================================================================
# E1_SLAVER_POWER_CTU 分层固件（STM32F103RCT7 副电源管理模块）
#   - Boot 适配：App 链接 AppA 分区 0x08008000（96K），Bootloader 0x08000000(32K)
#     485 升级：0x1F 命令 → 写共享 metadata(0x0803E000) upgrade_flag=1 → 复位进 Boot
#     布局/契约见 ../E1_CTU_BOOT/docs/boot_485_ymodem.md
#   - 通信：RS485 主机查询应答式；日志：USART1
#   - 共享层：../public_layer/m_middlewares（sw_timer/msg_fifo/protocol_parser 等）
#             ../public_layer/service/srv_signal（状态灯效引擎）
#             ../E1_CTU_BOOT/service/boot_flash + hal_flash（升级跳转的 metadata 读写，
#             boot 相关代码统一由 E1_CTU_BOOT 工程持有）
#   - log_task 为工程内本地副本（UART 输出，见 tasks/log_task.c）
# ==============================================================================

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources (attach 到目标)
add_subdirectory(cmake/stm32cubemx)

# Add middleware (platform-independent static library via public_layer)
add_subdirectory(../public_layer/m_middlewares m_middlewares)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 批量收集分层源代码
aux_source_directory(${CMAKE_SOURCE_DIR}/tasks TASKS)
aux_source_directory(${CMAKE_SOURCE_DIR}/device_drivers DEV_DRV)
aux_source_directory(${CMAKE_SOURCE_DIR}/service SERVICE)
aux_source_directory(${CMAKE_SOURCE_DIR}/applications APP)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${TASKS}
    ${DEV_DRV}
    ${SERVICE}
    ${APP}
    # 共享服务（public_layer）：状态灯效引擎
    ${CMAKE_SOURCE_DIR}/../public_layer/service/srv_signal.c
    # 共享 boot 栈（metadata/升级跳转；ring_storage/crc 已在 m_middlewares 库内）
    # Boot 相关代码统一由 E1_CTU_BOOT 工程持有（不再取 public_layer/service）
    ${CMAKE_SOURCE_DIR}/../E1_CTU_BOOT/service/boot_flash.c
    ${CMAKE_SOURCE_DIR}/../public_layer/device_drivers/hal_flash/hal_flash.c
    ${CMAKE_SOURCE_DIR}/../public_layer/device_drivers/hal_flash/drv_stm32f1_flash.c
    ${CMAKE_SOURCE_DIR}/../public_layer/device_drivers/hal_flash/ring_storage_port.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    tasks
    device_drivers
    service
    applications
    ../public_layer/service
    ../E1_CTU_BOOT/service
    ../public_layer/device_drivers/hal_flash
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    # mpaland_printf: STM32F103 无硬件 FPU，禁用浮点/指数格式化
    PRINTF_DISABLE_SUPPORT_FLOAT
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
    # Flash 抽象层选型（升级 metadata 读写）
    HAL_FLASH_CHIP_STM32F1
    # Boot 适配：向量表偏移（main() 在 HAL_Init 前执行 SCB->VTOR=FLASH_BASE|VECT_TAB_OFFSET；
    #            App 链接于 A 分区 0x08008000，布局见 ../E1_CTU_BOOT/docs/boot_485_ymodem.md）
    VECT_TAB_OFFSET=0x00008000U
    # 分区布局（与 E1_CTU_BOOT 一致，见 ../E1_CTU_BOOT/docs/boot_485_ymodem.md）
    BOOT_FLASH_BOOT_SIZE=0x8000U
    BOOT_FLASH_APP_SIZE=0x18000U
    BOOT_FLASH_META_SIZE=0x2000U
    BOOT_META_OFFSET=0x3E000U
    BOOT_META_SECTOR_SIZE=0x800U
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m_middlewares

    # Add user defined libraries
)

# 生成 App Bin/Hex（objcopy 取编译器同目录，保证无 PATH 依赖）
get_filename_component(TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
set(TOOLCHAIN_OBJCOPY "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy")
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${TOOLCHAIN_OBJCOPY} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -Oihex $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${TOOLCHAIN_OBJCOPY} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -Obinary $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating hex || bin file for App"
)
