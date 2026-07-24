# m_middlewares — 通用中间件静态库
# 平台无关、静态分配、无 malloc
# 主工程 add_subdirectory() + target_link_libraries(... m_middlewares) 即可引用

add_library(m_middlewares STATIC)

# ========== 收集源码 ==========

# 框架原语
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/utils       _MID_utils)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/framework   _MID_framework)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/log         _MID_log)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/key_base    _MID_key_base)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/protocol_tools _MID_proto)

# 算法
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/algorithm           _MID_algorithm)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/algorithm/pll       _MID_pll)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/algorithm/controller _MID_ctrl)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/algorithm/filter    _MID_filter)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/algorithm/math      _MID_math)

# 第三方库
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/SEGGER_RTT/RTT          _MID_rtt)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/CmBacktrace/cm_backtrace _MID_cmbt)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/lwmem/src/lwmem          _MID_lwmem)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/ring_storage             _MID_ring)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/mpaland_printf            _MID_printf)

# ========== 源文件加入目标 ==========
target_sources(m_middlewares PRIVATE
    ${_MID_utils}
    ${_MID_framework}
    ${_MID_log}
    ${_MID_key_base}
    ${_MID_proto}

    ${_MID_algorithm}
    ${_MID_pll}
    ${_MID_ctrl}
    ${_MID_filter}
    ${_MID_math}

    ${_MID_rtt}
    ${_MID_cmbt}
    ${_MID_lwmem}
    ${_MID_ring}
    ${_MID_printf}

    # 汇编异常处理文件
    ${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/CmBacktrace/cm_backtrace/fault_handler/gcc/cmb_fault.S
)

# ========== 公开头文件路径（PUBLIC → 传播给链接该库的父项目） ==========
target_include_directories(m_middlewares PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/log>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/protocol_tools>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm/pll>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm/controller>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm/filter>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/algorithm/math>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/framework>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/CmBacktrace/cm_backtrace>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/SEGGER_RTT/RTT/config>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/SEGGER_RTT/RTT>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/lwmem/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/mpaland_printf>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Third_Party/ring_storage>
)
