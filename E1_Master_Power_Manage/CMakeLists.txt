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

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${DEV_DRV}
    ${DEV_DRV_HAL_FLASH}
    ${SERVICE}
    ${TASKS}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    tasks
    device_drivers
    service
    ../public_layer/device_drivers/hal_flash
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    HAL_FLASH_CHIP_STM32F4
    # mpaland_printf: STM32F407 FPv4-SP 无硬件双精度，禁用浮点输出
    PRINTF_DISABLE_SUPPORT_FLOAT
    PRINTF_DISABLE_SUPPORT_EXPONENTIAL
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m_middlewares

    # Add user defined libraries
)

# 生成Bin文件
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Oihex ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Obinary ${CMAKE_PROJECT_NAME}.bin
        COMMENT "Generating hex || bin file for STM32"
)