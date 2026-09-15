cmake_minimum_required(VERSION 3.16)
project(ctu_sdk_c LANGUAGES C VERSION 1.0.0)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(NOT UNIX)
    message(FATAL_ERROR "ctu_sdk_c 目前仅支持 Linux / POSIX")
endif()

option(CTU_BUILD_CLI "构建 ctu_cli 命令行工具" ON)
option(CTU_BUILD_EXAMPLES "构建示例程序" ON)
option(CTU_WARNINGS_AS_ERRORS "将编译告警视为错误" OFF)
option(CTU_SANITIZE "启用 ASan/UBSan（仅调试用）" OFF)

set(CTU_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wshadow)
if(CTU_WARNINGS_AS_ERRORS)
    list(APPEND CTU_WARNING_FLAGS -Werror)
endif()

set(CTU_SOURCES
    src/ctu.c
    src/ctu_error.c
    src/ctu_types.c
    src/ctu_protocol.c
    src/ctu_transport.c
    src/ctu_boot.c
    src/ctu_client.c
    src/ctu_poller.c
)

# ---- 随 SDK 一起发布的共享中间件（协议解析 + kfifo）----
# 说明：为保证 SDK 可独立部署（不依赖仓库其余部分），中间件以副本形式随包提供，
# 副本与 public_layer 原件字节一致，来源见 middleware/README.md。
set(CTU_MIDDLEWARE_PROTOCOL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/middleware/protocol_tools")
set(CTU_MIDDLEWARE_UTILS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/middleware/utils")

if(NOT EXISTS "${CTU_MIDDLEWARE_PROTOCOL_DIR}/protocol_parser.c")
    message(FATAL_ERROR "缺少中间件副本：${CTU_MIDDLEWARE_PROTOCOL_DIR}/protocol_parser.c")
endif()

add_library(ctu_middleware OBJECT
    ${CTU_MIDDLEWARE_PROTOCOL_DIR}/protocol_parser.c
    ${CTU_MIDDLEWARE_UTILS_DIR}/kfifo.c
)
target_include_directories(ctu_middleware PUBLIC
    ${CTU_MIDDLEWARE_PROTOCOL_DIR}
    ${CTU_MIDDLEWARE_UTILS_DIR}
)
# 中间件为外部原件副本，按自身规范维护，这里仅做基本告警检查，不参与本 SDK 的严格 -Werror
target_compile_options(ctu_middleware PRIVATE -Wall -Wextra)

add_library(ctu_sdk STATIC ${CTU_SOURCES} $<TARGET_OBJECTS:ctu_middleware>)
target_include_directories(ctu_sdk PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CTU_MIDDLEWARE_PROTOCOL_DIR}>
    $<BUILD_INTERFACE:${CTU_MIDDLEWARE_UTILS_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_options(ctu_sdk PRIVATE ${CTU_WARNING_FLAGS})
# POSIX/GNU 接口：clock_gettime、nanosleep、cfmakeraw、B460800、ptsname_r 等
target_compile_definitions(ctu_sdk PRIVATE _GNU_SOURCE)

# 目标名 ctu::sdk 便于下游 find_package/add_subdirectory 使用
add_library(ctu::sdk ALIAS ctu_sdk)

if(CTU_SANITIZE)
    target_compile_options(ctu_sdk PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(ctu_sdk PUBLIC -fsanitize=address,undefined)
endif()

if(CTU_BUILD_CLI)
    add_executable(ctu_cli tools/ctu_cli.c)
    target_link_libraries(ctu_cli PRIVATE ctu_sdk)
    target_compile_options(ctu_cli PRIVATE ${CTU_WARNING_FLAGS})
    target_compile_definitions(ctu_cli PRIVATE _GNU_SOURCE)
endif()

if(CTU_BUILD_EXAMPLES)
    add_executable(ctu_example_monitor examples/monitor.c)
    target_link_libraries(ctu_example_monitor PRIVATE ctu_sdk)
    target_compile_options(ctu_example_monitor PRIVATE ${CTU_WARNING_FLAGS})
    target_compile_definitions(ctu_example_monitor PRIVATE _GNU_SOURCE)

    add_executable(ctu_example_upgrade examples/upgrade.c)
    target_link_libraries(ctu_example_upgrade PRIVATE ctu_sdk)
    target_compile_options(ctu_example_upgrade PRIVATE ${CTU_WARNING_FLAGS})
    target_compile_definitions(ctu_example_upgrade PRIVATE _GNU_SOURCE)
endif()

install(TARGETS ctu_sdk ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
