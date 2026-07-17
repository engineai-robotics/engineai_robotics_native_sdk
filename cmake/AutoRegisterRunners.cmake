# AutoRegisterRunners.cmake
# 这个脚本用于自动发现和注册runners
# 新机制：扫描 .h 文件中的 REGISTER_RUNNER 宏，生成显式注册代码

function(_auto_register_collect_used_runner_sources USED_RUNNERS_ENV OUT_SOURCES)
  set(_runner_sources "")
  string(REPLACE "," ";" _used_runners_list "${USED_RUNNERS_ENV}")

  foreach(_used_runner ${_used_runners_list})
    string(REGEX REPLACE "_runner$" "" _runner_base "${_used_runner}")
    file(GLOB_RECURSE _subdir_sources "${CMAKE_SOURCE_DIR}/src/runner/${_runner_base}/include/*.h")
    list(APPEND _runner_sources ${_subdir_sources})
    message(STATUS "AutoRegisterRunners: Including sources from ${_runner_base}")
  endforeach()

  set(${OUT_SOURCES} "${_runner_sources}" PARENT_SCOPE)
endfunction()

function(_auto_register_collect_sdk_core_sources OUT_SOURCES)
  set(_runner_sources "")
  file(GLOB _runner_subdirs RELATIVE "${CMAKE_SOURCE_DIR}/src/runner" "${CMAKE_SOURCE_DIR}/src/runner/*")

  foreach(_subdir ${_runner_subdirs})
    set(_subdir_cmake "${CMAKE_SOURCE_DIR}/src/runner/${_subdir}/CMakeLists.txt")
    if(EXISTS "${_subdir_cmake}")
      file(READ "${_subdir_cmake}" _cmake_content)
      if(_cmake_content MATCHES "set\\(SDK_CORE_MODULE[ \t]+ON\\)")
        file(GLOB_RECURSE _src_files "${CMAKE_SOURCE_DIR}/src/runner/${_subdir}/include/*.h")
        list(APPEND _runner_sources ${_src_files})
        message(STATUS "AutoRegisterRunners: Including SDK core module: ${_subdir}")
      endif()
    endif()
  endforeach()

  set(${OUT_SOURCES} "${_runner_sources}" PARENT_SCOPE)
endfunction()

function(_auto_register_extract_core_from_pregenerated PREGENERATED_FILE OUT_CORE_CLASSES OUT_DECLARATIONS OUT_REGISTRATIONS OUT_CORE_TARGETS)
  file(READ "${PREGENERATED_FILE}" _pregen_content)

  set(_core_runner_classes "")
  set(_factory_declarations "")
  set(_runner_registrations "")
  set(_core_targets "")

  string(REGEX MATCHALL "Register([A-Za-z0-9]+Runner)\\(registry\\)" _register_calls "${_pregen_content}")
  foreach(_call ${_register_calls})
    string(REGEX REPLACE "Register([A-Za-z0-9]+Runner)\\(registry\\)" "\\1" _class_name "${_call}")
    list(FIND _core_runner_classes "${_class_name}" _class_index)
    if(NOT _class_index EQUAL -1)
      continue()
    endif()

    list(APPEND _core_runner_classes "${_class_name}")
    string(APPEND _factory_declarations "extern \"C\" void Register${_class_name}(runner::RunnerRegistry& registry);\n")
    string(APPEND _runner_registrations "  Register${_class_name}(registry);\n")

    string(REGEX REPLACE "Runner$" "" _base_name "${_class_name}")
    string(REGEX REPLACE "([a-z])([A-Z])" "\\1_\\2" _snake_name "${_base_name}")
    string(TOLOWER "${_snake_name}" _snake_name)

    foreach(_suffix "" "_runner")
      set(_target "EngineAICore::src_runner_${_snake_name}${_suffix}")
      if(TARGET ${_target})
        list(APPEND _core_targets "${_target}")
        message(STATUS "AutoRegisterRunners: Found core library: ${_target}")
        break()
      endif()
    endforeach()
  endforeach()

  set(${OUT_CORE_CLASSES} "${_core_runner_classes}" PARENT_SCOPE)
  set(${OUT_DECLARATIONS} "${_factory_declarations}" PARENT_SCOPE)
  set(${OUT_REGISTRATIONS} "${_runner_registrations}" PARENT_SCOPE)
  set(${OUT_CORE_TARGETS} "${_core_targets}" PARENT_SCOPE)
endfunction()

function(_auto_register_extract_from_sources SOURCES SKIP_CLASSES OUT_DECLARATIONS OUT_REGISTRATIONS OUT_DEPENDENCIES)
  set(_factory_declarations "")
  set(_runner_registrations "")
  set(_runner_dependencies "")
  set(_registered_classes "")

  foreach(_source ${SOURCES})
    # Skip runner_registry.h itself - it contains the macro definition with 'ClassName' as parameter
    get_filename_component(_source_filename "${_source}" NAME)
    if(_source_filename STREQUAL "runner_registry.h")
      continue()
    endif()

    file(READ "${_source}" _source_content)
    string(REGEX MATCHALL "REGISTER_RUNNER\\(([A-Za-z0-9_]+)," _factory_matches "${_source_content}")

    foreach(_match ${_factory_matches})
      string(REGEX REPLACE "REGISTER_RUNNER\\(([A-Za-z0-9_]+)," "\\1" _class_name "${_match}")

      list(FIND SKIP_CLASSES "${_class_name}" _skip_index)
      if(NOT _skip_index EQUAL -1)
        continue()
      endif()

      list(FIND _registered_classes "${_class_name}" _registered_index)
      if(NOT _registered_index EQUAL -1)
        continue()
      endif()

      list(APPEND _registered_classes "${_class_name}")
      string(APPEND _factory_declarations "extern \"C\" void Register${_class_name}(runner::RunnerRegistry& registry);\n")
      string(APPEND _runner_registrations "  Register${_class_name}(registry);\n")
      message(STATUS "AutoRegisterRunners: Found factory for ${_class_name}")
    endforeach()

    string(REGEX MATCH "src/runner/([^/]+)/(src|include)" _path_match "${_source}")
    if(CMAKE_MATCH_1)
      list(APPEND _runner_dependencies "src::runner::${CMAKE_MATCH_1}")
    endif()
  endforeach()

  set(${OUT_DECLARATIONS} "${_factory_declarations}" PARENT_SCOPE)
  set(${OUT_REGISTRATIONS} "${_runner_registrations}" PARENT_SCOPE)
  set(${OUT_DEPENDENCIES} "${_runner_dependencies}" PARENT_SCOPE)
endfunction()

function(_auto_register_render_content HEADER_COMMENT FACTORY_DECLARATIONS RUNNER_REGISTRATIONS OUT_CONTENT)
  set(_content "// Auto-generated file - do not edit manually\n")
  string(APPEND _content "// ${HEADER_COMMENT}\n\n")
  string(APPEND _content "#include \"basic/runner_registry.h\"\n\n")
  string(APPEND _content "// Factory function declarations\n")
  string(APPEND _content "${FACTORY_DECLARATIONS}\n")
  string(APPEND _content "namespace runner {\n\n")
  string(APPEND _content "void RegisterAllRunners() {\n")
  string(APPEND _content "  auto& registry = RunnerRegistry::Instance();\n")
  string(APPEND _content "${RUNNER_REGISTRATIONS}")
  string(APPEND _content "}\n\n")
  string(APPEND _content "}  // namespace runner\n")

  set(${OUT_CONTENT} "${_content}" PARENT_SCOPE)
endfunction()

function(_auto_register_write_if_changed OUTPUT_FILE CONTENT)
  set(_should_write TRUE)
  if(EXISTS "${OUTPUT_FILE}")
    file(READ "${OUTPUT_FILE}" _existing_content)
    if("${_existing_content}" STREQUAL "${CONTENT}")
      set(_should_write FALSE)
    endif()
  endif()

  if(_should_write)
    file(WRITE ${OUTPUT_FILE} "${CONTENT}")
    message(STATUS "AutoRegisterRunners: Generated auto_register_runners.cc with explicit registration")
  else()
    message(STATUS "AutoRegisterRunners: auto_register_runners.cc unchanged, skipping write")
  endif()
endfunction()

function(_auto_register_filter_runner_dependencies RUNNER_DEPENDENCIES USED_RUNNERS_ENV OUT_FILTERED_DEPENDENCIES)
  if(NOT USED_RUNNERS_ENV)
    set(${OUT_FILTERED_DEPENDENCIES} "${RUNNER_DEPENDENCIES}" PARENT_SCOPE)
    return()
  endif()

  message(STATUS "Filtering runners based on environment variable: ${USED_RUNNERS_ENV}")
  string(REPLACE "," ";" _used_runners_list "${USED_RUNNERS_ENV}")
  set(_filtered_dependencies "")

  foreach(_used_runner ${_used_runners_list})
    string(REGEX REPLACE "_runner$" "" _runner_base "${_used_runner}")
    set(_runner_target "src::runner::${_runner_base}")
    list(FIND RUNNER_DEPENDENCIES "${_runner_target}" _target_index)
    if(NOT _target_index EQUAL -1)
      list(APPEND _filtered_dependencies "${_runner_target}")
      message(STATUS "Including runner: ${_used_runner} -> ${_runner_target}")
    else()
      message(STATUS "Runner target not found for: ${_used_runner} -> ${_runner_target}")
    endif()
  endforeach()

  set(${OUT_FILTERED_DEPENDENCIES} "${_filtered_dependencies}" PARENT_SCOPE)
endfunction()

function(auto_register_runners TARGET_NAME)
  # 检查是否设置了选择性编译环境变量
  set(used_runners_env "$ENV{ENGINEAI_ROBOTICS_USED_RUNNERS}")

  set(runner_sources "")
  set(core_runner_classes "")
  set(core_factory_declarations "")
  set(core_runner_registrations "")
  set(core_runner_targets "")
  set(comment_line "This file contains explicit runner registration calls")

  if(used_runners_env)
    message(STATUS "AutoRegisterRunners: Selective mode enabled")
    _auto_register_collect_used_runner_sources("${used_runners_env}" runner_sources)
  else()
    message(STATUS "AutoRegisterRunners: Full compilation mode")

    if(BUILD_SDK_MODE)
      message(STATUS "AutoRegisterRunners: BUILD_SDK_MODE - scanning only SDK core modules")
      _auto_register_collect_sdk_core_sources(runner_sources)
    elseif(SDK_FRAMEWORK_MODE)
      message(STATUS "AutoRegisterRunners: SDK_FRAMEWORK_MODE - merging core and local runners")
      set(pregenerated_file "${CMAKE_SOURCE_DIR}/${SDK_CORE_DIR_NAME}/registry/auto_register_runners.cc")
      if(NOT EXISTS "${pregenerated_file}")
        message(FATAL_ERROR "AutoRegisterRunners: Pre-generated file not found: ${pregenerated_file}")
      endif()
      _auto_register_extract_core_from_pregenerated(
        "${pregenerated_file}"
        core_runner_classes
        core_factory_declarations
        core_runner_registrations
        core_runner_targets
      )
      file(GLOB_RECURSE runner_sources
        "${CMAKE_SOURCE_DIR}/src/runner/*/src/*.cc"
        "${CMAKE_SOURCE_DIR}/src/runner/*/include/*.h")
      set(comment_line "Merged: pre-compiled core runners + locally compiled runners")
    else()
      file(GLOB_RECURSE runner_sources
        "${CMAKE_SOURCE_DIR}/src/runner/*/src/*.cc"
        "${CMAKE_SOURCE_DIR}/src/runner/*/include/*.h"
      )
    endif()
  endif()

  set(auto_register_file "${CMAKE_CURRENT_BINARY_DIR}/auto_register_runners.cc")
  set(local_factory_declarations "")
  set(local_runner_registrations "")
  set(runner_dependencies "")

  _auto_register_extract_from_sources(
    "${runner_sources}"
    "${core_runner_classes}"
    local_factory_declarations
    local_runner_registrations
    runner_dependencies
  )

  set(factory_declarations "${core_factory_declarations}${local_factory_declarations}")
  set(runner_registrations "${core_runner_registrations}${local_runner_registrations}")

  _auto_register_render_content(
    "${comment_line}"
    "${factory_declarations}"
    "${runner_registrations}"
    auto_register_content
  )
  _auto_register_write_if_changed("${auto_register_file}" "${auto_register_content}")

  if(BUILD_SDK_MODE AND DEFINED _SDK_INSTALL_DIR)
    install(FILES "${auto_register_file}"
        DESTINATION "${_SDK_INSTALL_DIR}/registry"
        RENAME "auto_register_runners.cc"
    )
    message(STATUS "AutoRegisterRunners: Will install to SDK: ${_SDK_INSTALL_DIR}/registry")
  endif()

  target_sources(${TARGET_NAME} PRIVATE ${auto_register_file})

  if(core_runner_targets)
    list(REMOVE_DUPLICATES core_runner_targets)
    target_link_libraries(${TARGET_NAME} PUBLIC ${core_runner_targets})
    message(STATUS "AutoRegisterRunners: Linked core libraries: ${core_runner_targets}")
  endif()

  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${runner_sources})

  if(runner_dependencies)
    list(REMOVE_DUPLICATES runner_dependencies)
    _auto_register_filter_runner_dependencies("${runner_dependencies}" "${used_runners_env}" filtered_dependencies)
    if(filtered_dependencies)
      target_link_libraries(${TARGET_NAME} PUBLIC ${filtered_dependencies})
      if(used_runners_env)
        message(STATUS "Auto-linked filtered runners: ${filtered_dependencies}")
      else()
        message(STATUS "Auto-linked all runners: ${filtered_dependencies}")
      endif()
    else()
      message(WARNING "No valid runner targets found for environment variable filter")
      target_link_libraries(${TARGET_NAME} PUBLIC ${runner_dependencies})
      message(STATUS "Auto-linked all runners: ${runner_dependencies}")
    endif()
  endif()

endfunction()
