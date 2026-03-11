# AutoRegisterTransitioners.cmake
# 这个脚本用于自动发现和注册transitioners

function(auto_register_transitioners TARGET_NAME)
  # 扫描所有transitioner头文件
  file(GLOB_RECURSE transitioner_headers 
    "${CMAKE_SOURCE_DIR}/src/task/transitioner/include/transitioner/*.h"
    "${CMAKE_SOURCE_DIR}/src/task/transitioner/include/transitioner/*.hpp"
  )
  
  set(auto_include_file "${CMAKE_CURRENT_BINARY_DIR}/auto_register_transitioners.cc")
  set(headers_with_register "")
  
  foreach(header ${transitioner_headers})
    execute_process(
      COMMAND grep -l "REGISTER_TRANSITIONER(" "${header}"
      OUTPUT_VARIABLE grep_result
      ERROR_QUIET
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
    if(grep_result)
      list(APPEND headers_with_register ${header})
    endif()
  endforeach()
  
  # 生成自动注册文件
  set(auto_include_content "// Auto-generated file - do not edit manually\n")
  string(APPEND auto_include_content "// This file includes all transitioner headers to trigger static registration\n\n")
  
  foreach(header ${headers_with_register})
    file(RELATIVE_PATH relative_header ${CMAKE_SOURCE_DIR}/src/manager/include ${header})
    string(APPEND auto_include_content "#include \"${relative_header}\"\n")
  endforeach()
  
  string(APPEND auto_include_content "\nnamespace manager {\nvoid auto_register_transitioners_dummy() {}\n}\n")
  
  # 只在内容真正改变时才写入文件（避免触发不必要的重新编译）
  set(should_write TRUE)
  if(EXISTS ${auto_include_file})
    file(READ ${auto_include_file} existing_content)
    if(existing_content STREQUAL auto_include_content)
      set(should_write FALSE)
    endif()
  endif()
  
  if(should_write)
    file(WRITE ${auto_include_file} "${auto_include_content}")
    message(STATUS "AutoRegisterTransitioners: Generated auto_register_transitioners.cc")
  else()
    message(STATUS "AutoRegisterTransitioners: auto_register_transitioners.cc unchanged, skipping write")
  endif()
  
  # 将生成的文件添加到目标
  target_sources(${TARGET_NAME} PRIVATE ${auto_include_file})
  
  # 添加头文件依赖，让CMake知道当这些头文件改变时需要重新配置
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${headers_with_register})
  
  message(STATUS "Auto-registered transitioners: ${headers_with_register}")
endfunction()

