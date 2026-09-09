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
set(CMAKE_PROJECT_NAME E1_CTU_BOOT)

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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # --- 本地 Boot 业务层 ---
    device_drivers/drv_uart.c
    device_drivers/dev_rs485.c
    device_drivers/drv_systick.c
    device_drivers/drv_log_uart.c
    tasks/app_main.c
    tasks/boot_task.c
    tasks/log_task.c
    service/boot_ymodem.c
    # --- public_layer 共享 Flash/boot 栈 ---
    ../public_layer/device_drivers/hal_flash/hal_flash.c
    ../public_layer/device_drivers/hal_flash/drv_stm32f1_flash.c
    ../public_layer/device_drivers/hal_flash/ring_storage_port.c
    ../public_layer/service/boot/boot_flash.c
    ../public_layer/m_middlewares/Third_Party/ring_storage/ring_storage.c
    ../public_layer/m_middlewares/Third_Party/ring_storage/rs_crc32.c
    ../public_layer/m_middlewares/Third_Party/mpaland_printf/printf.c
    ../public_layer/m_middlewares/log/log.c
    ../public_layer/m_middlewares/algorithm/crc.c
    ../public_layer/m_middlewares/utils/kfifo.c
    ../public_layer/m_middlewares/framework/msg_fifo.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # --- 本地 ---
    device_drivers
    tasks
    service
    # --- 共享 public_layer ---
    ../public_layer/service/boot
    ../public_layer/device_drivers/hal_flash
    ../public_layer/m_middlewares
    ../public_layer/m_middlewares/log
    ../public_layer/m_middlewares/utils
    ../public_layer/m_middlewares/framework
    ../public_layer/m_middlewares/algorithm
    ../public_layer/m_middlewares/Third_Party/ring_storage
    ../public_layer/m_middlewares/Third_Party/mpaland_printf
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Flash 抽象层选型：STM32F1 驱动
    HAL_FLASH_CHIP_STM32F1
    # 日志格式化（m_middlewares log 使用 mpaland_printf）：STM32F103 无 FPU，
    # 且日志不使用浮点/指数，裁剪 printf 体积
    PRINTF_DISABLE_SUPPORT_FLOAT
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
    # 分区布局（F103RCTx，见 docs/boot_485_ymodem.md）
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

    # Add user defined libraries
)

# 生成 Boot Hex/Bin（objcopy 取编译器同目录，保证无 PATH 依赖）
get_filename_component(TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
set(TOOLCHAIN_OBJCOPY "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy")
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${TOOLCHAIN_OBJCOPY} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -Oihex $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${TOOLCHAIN_OBJCOPY} $<TARGET_FILE:${CMAKE_PROJECT_NAME}> -Obinary $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating hex || bin file for Boot"
)
