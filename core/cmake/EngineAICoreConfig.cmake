
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was EngineAICoreConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set(EngineAICore_DIR_NAME "core")
set(SDK_FRAMEWORK_MODE ON CACHE BOOL "Running in SDK framework mode" FORCE)
set(EngineAICore_VERSION )

# 使用 PACKAGE_INIT 提供的宏进行路径重定位
# PACKAGE_PREFIX_DIR 由 PACKAGE_INIT 自动计算为安装前缀
set_and_check(EngineAICore_LIB_DIR "${PACKAGE_PREFIX_DIR}/native_sdk/core/lib")
set_and_check(EngineAICore_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/native_sdk/core/include")

# 计算 SDK 根目录
get_filename_component(EngineAICore_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(EngineAICore_SDK_ROOT "${EngineAICore_CMAKE_DIR}" PATH)

set(EngineAICore_LIBRARY_DIRS "${EngineAICore_LIB_DIR}")

# SDK 检测路径（供 AutoRegisterRunners 使用）
set(SDK_CORE_DIR_NAME "${EngineAICore_DIR_NAME}")
set(SDK_DETECTION_INCLUDE_PATH "${EngineAICore_INCLUDE_DIR}")

# 构建 include 目录列表
set(EngineAICore_INCLUDE_DIRS "${EngineAICore_INCLUDE_DIR}")

# 递归扫描所有头文件并添加其所有父目录层级
file(GLOB_RECURSE _all_headers "${EngineAICore_INCLUDE_DIR}/*.h" "${EngineAICore_INCLUDE_DIR}/*.hpp")
foreach(_header ${_all_headers})
    # 计算相对于 include/ 的路径
    file(RELATIVE_PATH _rel_path "${EngineAICore_INCLUDE_DIR}" "${_header}")
    get_filename_component(_rel_dir "${_rel_path}" DIRECTORY)

    # 添加所有父目录层级
    # 例如：data/data_store/data_store.h -> 添加 data/ 和 data/data_store/
    while(_rel_dir)
        list(APPEND EngineAICore_INCLUDE_DIRS "${EngineAICore_INCLUDE_DIR}/${_rel_dir}")
        get_filename_component(_rel_dir "${_rel_dir}" DIRECTORY)
    endwhile()
endforeach()

list(REMOVE_DUPLICATES EngineAICore_INCLUDE_DIRS)
unset(_all_headers)
unset(_header)
unset(_rel_path)
unset(_rel_dir)

# 导入已编译的核心库目标
# 此文件由 install(EXPORT) 自动生成，包含：
# - 所有库的导入目标（如 EngineAICore::src_core_math）
# - 完整的依赖传递关系（PUBLIC/PRIVATE链接）
# - 头文件路径（通过生成表达式自动计算）
# - 编译特性和定义

# 1. 加载原始 Targets 文件（包含硬编码路径）
include("${CMAKE_CURRENT_LIST_DIR}/EngineAICoreTargets.cmake")

# 2. 运行时重定位：修正所有 IMPORTED 目标的库文件位置
# 动态发现所有导出的 EngineAICore 目标，避免硬编码列表
get_property(_imported_targets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
set(_engineai_imported_targets "")
foreach(_target ${_imported_targets})
    if(_target MATCHES "^EngineAICore::src_.+$")
        list(APPEND _engineai_imported_targets "${_target}")
    endif()
endforeach()

foreach(_full_target ${_engineai_imported_targets})
    if(TARGET ${_full_target})
        # 获取目标类型
        get_target_property(_target_type ${_full_target} TYPE)

        # 只处理非 INTERFACE 库（SHARED_LIBRARY, STATIC_LIBRARY）
        if(NOT _target_type STREQUAL "INTERFACE_LIBRARY")
            # 处理按配置导入的位置（Release/RelWithDebInfo/Debug）
            get_target_property(_configs ${_full_target} IMPORTED_CONFIGURATIONS)
            if(_configs)
                foreach(_cfg ${_configs})
                    string(TOUPPER "${_cfg}" _cfg_upper)
                    get_target_property(_old_location ${_full_target} "IMPORTED_LOCATION_${_cfg_upper}")
                    if(_old_location)
                        get_filename_component(_lib_name "${_old_location}" NAME)
                        set_target_properties(${_full_target} PROPERTIES
                            "IMPORTED_LOCATION_${_cfg_upper}" "${EngineAICore_SDK_ROOT}/lib/${_lib_name}"
                        )
                    endif()
                endforeach()
            endif()

            # 同时设置不带配置后缀的版本（用于兼容性）
            get_target_property(_old_location ${_full_target} IMPORTED_LOCATION)
            if(_old_location)
                get_filename_component(_lib_name "${_old_location}" NAME)
                set_target_properties(${_full_target} PROPERTIES
                    IMPORTED_LOCATION "${EngineAICore_SDK_ROOT}/lib/${_lib_name}"
                )
            endif()
        endif()
    endif()
endforeach()

unset(_imported_targets)
unset(_engineai_imported_targets)
unset(_target)
unset(_full_target)
unset(_target_type)
unset(_configs)
unset(_cfg)
unset(_cfg_upper)
unset(_old_location)
unset(_lib_name)

# 为所有导入的核心库目标设置 include 目录并创建别名
# 处理常规库文件
file(GLOB _lib_files
    "${EngineAICore_SDK_ROOT}/lib/libsrc_*.so"
    "${EngineAICore_SDK_ROOT}/lib/libsrc_*.a"
    "${EngineAICore_SDK_ROOT}/lib/libsrc_*.dylib"
)

set(_all_module_names "")

foreach(_lib_file ${_lib_files})
    get_filename_component(_lib_name "${_lib_file}" NAME_WE)
    string(REGEX REPLACE "^libsrc_" "" _module_name "${_lib_name}")

    # 构建目标名：common_base -> EngineAICore::src_common_base
    set(_full_target "EngineAICore::src_${_module_name}")

    if(TARGET ${_full_target})
        # 设置 include 目录到导入的目标
        set_property(TARGET ${_full_target} APPEND PROPERTY
            INTERFACE_INCLUDE_DIRECTORIES "${EngineAICore_INCLUDE_DIRS}"
        )

        list(APPEND _all_module_names ${_module_name})
    endif()
endforeach()

# 处理 INTERFACE 库（header-only，没有对应的 .so 文件）
# 从 EngineAICoreTargets.cmake 中提取所有已定义的目标
if(NOT _engineai_imported_targets)
    get_property(_engineai_imported_targets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)
endif()
foreach(_target ${_engineai_imported_targets})
    if(_target MATCHES "^EngineAICore::src_(.+)$")
        set(_module_name "${CMAKE_MATCH_1}")
        list(FIND _all_module_names "${_module_name}" _found_idx)
        if(_found_idx EQUAL -1)
            # 这是一个 INTERFACE 库，添加到列表
            list(APPEND _all_module_names ${_module_name})
            # 设置 include 目录
            set_property(TARGET ${_target} APPEND PROPERTY
                INTERFACE_INCLUDE_DIRECTORIES "${EngineAICore_INCLUDE_DIRS}"
            )
        endif()
    endif()
endforeach()

# 为所有模块创建 src:: 别名
foreach(_module_name ${_all_module_names})
    set(_full_target "EngineAICore::src_${_module_name}")

    # 创建 src:: 别名，正确处理多级命名空间
    # 规则：只替换前两个下划线，保留后续的下划线
    # 例如：core_estimator_base_state_estimator -> core::estimator::base_state_estimator
    string(REGEX REPLACE "^([^_]+)_([^_]+)_(.+)$" "\\1::\\2::\\3" _alias_name "${_module_name}")
    # 处理只有两段的情况（如 common_base）
    if(_alias_name STREQUAL _module_name)
        string(REGEX REPLACE "^([^_]+)_(.+)$" "\\1::\\2" _alias_name "${_module_name}")
    endif()

    if(NOT TARGET src::${_alias_name})
        add_library(src::${_alias_name} ALIAS ${_full_target})
        message(STATUS "Created alias: src::${_alias_name} -> ${_full_target}")
    endif()
endforeach()

unset(_lib_files)
unset(_lib_file)
unset(_lib_name)
unset(_module_name)
unset(_full_target)
unset(_alias_name)

check_required_components(EngineAICore)

message(STATUS "EngineAI Core SDK loaded: ${EngineAICore_VERSION}")
