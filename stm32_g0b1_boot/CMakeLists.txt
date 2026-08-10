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
set(CMAKE_PROJECT_NAME stm32_g0_boot)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# 批量收集源代码目录
aux_source_directory(${CMAKE_SOURCE_DIR}/tasks TASKS)
aux_source_directory(${CMAKE_SOURCE_DIR}/device_drivers DEV_DRV)
aux_source_directory(${CMAKE_SOURCE_DIR}/../public_layer/device_drivers/hal_flash DEV_DRV_HAL_FLASH)

# Middleware 通过 add_subdirectory 引用为独立静态库
add_subdirectory(../public_layer/m_middlewares m_middlewares)

# mpaland_printf 在该项目中禁用浮点输出（STM32G0B1 M0+ 无硬件双精度）
target_compile_definitions(m_middlewares PRIVATE
    PRINTF_DISABLE_SUPPORT_FLOAT
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
)

# Boot 服务/任务复用 public_layer 共享实现（本工程不维护本地副本）。
# 板级分区参数经下方 target_compile_definitions 覆写为 G0 布局。
set(PUBLIC_BOOT_SRCS
    ${CMAKE_SOURCE_DIR}/../public_layer/service/boot/boot_transport.c
    ${CMAKE_SOURCE_DIR}/../public_layer/service/boot/boot_fsm.c
    ${CMAKE_SOURCE_DIR}/../public_layer/service/boot/boot_flash.c
    ${CMAKE_SOURCE_DIR}/../public_layer/task/boot_task.c
    ${CMAKE_SOURCE_DIR}/../public_layer/task/log_task.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${DEV_DRV}
    ${DEV_DRV_HAL_FLASH}

    ${TASKS}
    ${PUBLIC_BOOT_SRCS}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    tasks
    device_drivers
    ../public_layer/device_drivers/hal_flash
    ../public_layer/service
    ../public_layer/service/boot
    ../public_layer/task
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    HAL_FLASH_CHIP_STM32G0
    # G0 板级分区布局（覆写 public_layer boot_flash.h 的 F407 默认值）
    # Boot 64K (0x08000000) | App 56K (0x08010000) | Metadata 8K (0x0801E000)
    BOOT_FLASH_BOOT_SIZE=0x10000U
    BOOT_FLASH_APP_SIZE=0xE000U
    BOOT_FLASH_META_SIZE=0x2000U
    # G0 硬件兼容 ID 与 App 向量表范围（覆写 public_layer boot_task.c 的 F407 默认值）
    BOOT_HW_COMPAT_ID=0x0003U
    BOOT_APP_FLASH_START=0x08010000U
    BOOT_APP_FLASH_END=0x0801E000U
    # G0 RAM 144K（0x20000000-0x20024000），覆写 boot_task.c 默认 128K 上界
    BOOT_RAM_START=0x20000000U
    BOOT_RAM_END=0x20024000U
    # G0 CAN 升级 ID（覆写 public_layer boot_transport.h 默认 0x701/0x702）
    BOOT_CAN_ID_HOST_TO_NODE=0x701U
    BOOT_CAN_ID_NODE_TO_HOST=0x702U
    # 单分区模式：无 A/B 双槽位，App 仅一个分区（省 Flash，去 promote/分区切换逻辑）
    BOOT_SINGLE_PARTITION
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # 通用中间件（平台无关，静态分配）
    m_middlewares
)

# 生成Bin文件
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Oihex ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Obinary ${CMAKE_PROJECT_NAME}.bin
        COMMENT "Generating hex || bin file for STM32"
)
