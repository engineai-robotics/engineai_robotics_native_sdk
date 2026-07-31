
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was EngineAICoreConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

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

include(CMakeFindDependencyMacro)
find_dependency(Boost REQUIRED COMPONENTS thread)

set(EngineAICore_DIR_NAME "core")
set(SDK_FRAMEWORK_MODE ON CACHE BOOL "Running in SDK framework mode" FORCE)
set(EngineAICore_VERSION )

# Relocate paths using the macros provided by PACKAGE_INIT.
set_and_check(EngineAICore_LIB_DIR "${PACKAGE_PREFIX_DIR}/core/lib")
set_and_check(EngineAICore_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/core/include")

# Compute the SDK root directory (used for library relocation)
get_filename_component(EngineAICore_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(EngineAICore_SDK_ROOT "${EngineAICore_CMAKE_DIR}" PATH)

# SDK detection directory name (used by AutoRegisterRunners)
set(SDK_CORE_DIR_NAME "${EngineAICore_DIR_NAME}")

# ----------------------------------------------------------------------------
# Build the include directory list: the include/ root plus every parent directory level of each header.
# ----------------------------------------------------------------------------
set(EngineAICore_INCLUDE_DIRS "${EngineAICore_INCLUDE_DIR}")

file(GLOB_RECURSE _all_headers "${EngineAICore_INCLUDE_DIR}/*.h" "${EngineAICore_INCLUDE_DIR}/*.hpp")
foreach(_header ${_all_headers})
    file(RELATIVE_PATH _rel_path "${EngineAICore_INCLUDE_DIR}" "${_header}")
    get_filename_component(_rel_dir "${_rel_path}" DIRECTORY)
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

# ----------------------------------------------------------------------------
# Import the compiled core-library targets (with dependencies, compile features, etc.)
# ----------------------------------------------------------------------------
include("${CMAKE_CURRENT_LIST_DIR}/EngineAICoreTargets.cmake")

# ----------------------------------------------------------------------------
# Iterate over every imported EngineAICore::src_* target:
# ----------------------------------------------------------------------------
get_property(_imported_targets DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY IMPORTED_TARGETS)

foreach(_full_target ${_imported_targets})
    if(NOT _full_target MATCHES "^EngineAICore::src_(.+)$")
        continue()
    endif()
    set(_module_name "${CMAKE_MATCH_1}")

    if(NOT TARGET ${_full_target})
        continue()
    endif()

    get_target_property(_target_type ${_full_target} TYPE)

    # 1. Relocate the library location (non-INTERFACE libraries only)
    if(NOT _target_type STREQUAL "INTERFACE_LIBRARY")
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

        get_target_property(_old_location ${_full_target} IMPORTED_LOCATION)
        if(_old_location)
            get_filename_component(_lib_name "${_old_location}" NAME)
            set_target_properties(${_full_target} PROPERTIES
                IMPORTED_LOCATION "${EngineAICore_SDK_ROOT}/lib/${_lib_name}"
            )
        endif()
    endif()

    # 2. Append include directories
    set_property(TARGET ${_full_target} APPEND PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES "${EngineAICore_INCLUDE_DIRS}"
    )

    # 3. Create the src:: alias, handling multi-level namespaces correctly.
    string(REGEX REPLACE "^([^_]+)_([^_]+)_(.+)$" "\\1::\\2::\\3" _alias_name "${_module_name}")
    if(_alias_name STREQUAL _module_name)
        string(REGEX REPLACE "^([^_]+)_(.+)$" "\\1::\\2" _alias_name "${_module_name}")
    endif()

    if(NOT TARGET src::${_alias_name})
        add_library(src::${_alias_name} ALIAS ${_full_target})
        message(STATUS "Created alias: src::${_alias_name} -> ${_full_target}")
    endif()
endforeach()

unset(_imported_targets)
unset(_full_target)
unset(_module_name)
unset(_target_type)
unset(_configs)
unset(_cfg)
unset(_cfg_upper)
unset(_old_location)
unset(_lib_name)
unset(_alias_name)

check_required_components(EngineAICore)

message(STATUS "EngineAI Core SDK loaded: ${EngineAICore_VERSION}")
