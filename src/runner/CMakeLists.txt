# This CMakeLists.txt file is responsible for including all subdirectories within the current source directory.
# It uses the file() command to retrieve all subdirectories relative to the current source directory.
# Then, it iterates over each subdirectory and checks if it contains a CMakeLists.txt file.
# If a subdirectory contains a CMakeLists.txt file, it adds the subdirectory using the add_subdirectory() command.
# If a subdirectory does not contain a CMakeLists.txt file, it displays a warning message and ignores the subdirectory.

file(GLOB all_subdirectories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
set(added_runner_subdirs "")

# 检查是否设置了环境变量进行选择性编译
set(used_runners_env "$ENV{ENGINEAI_ROBOTICS_USED_RUNNERS}")

if(used_runners_env)
    message(STATUS "Selective runner compilation enabled: ${used_runners_env}")
    
    # 将环境变量中的runner列表转换为CMake列表
    string(REPLACE "," ";" used_runners_list "${used_runners_env}")
    
    # 从runner名称推导子目录名称
    # 始终包含basic目录（基础依赖）
    set(allowed_subdirs "basic")
    
    foreach(used_runner ${used_runners_list})
        # runner名称格式: xxx_runner -> 子目录: xxx
        string(REGEX REPLACE "_runner$" "" runner_base "${used_runner}")
        list(APPEND allowed_subdirs "${runner_base}")
    endforeach()
    
    list(REMOVE_DUPLICATES allowed_subdirs)
    
    message(STATUS "Allowed runner subdirectories: ${allowed_subdirs}")
    
    foreach(subdir ${all_subdirectories})
        set(subdir_cmake_file ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)

        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} AND EXISTS ${subdir_cmake_file})
            # 检查该子目录是否在允许列表中
            list(FIND allowed_subdirs "${subdir}" subdir_index)
            if(NOT subdir_index EQUAL -1)
                add_subdirectory(${subdir})
                list(APPEND added_runner_subdirs "${subdir}")
                message(STATUS "Including runner subdirectory: ${subdir}")
            else()
                message(STATUS "Skipping runner subdirectory (not in used list): ${subdir}")
            endif()
        elseif(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
            message(WARNING "[ ${subdir} ] is a directory but does not contain a CMakeLists.txt file and will be ignored.")
        endif()
    endforeach()
else()
    message(STATUS "Building all runners (no environment filter)")
    
    foreach(subdir ${all_subdirectories})
        set(subdir_cmake_file ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)

        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} AND EXISTS ${subdir_cmake_file})
            add_subdirectory(${subdir})
            list(APPEND added_runner_subdirs "${subdir}")
        elseif(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
            message(WARNING "[ ${subdir} ] is a directory but does not contain a CMakeLists.txt file and will be ignored.")
        endif()
    endforeach()
endif()

