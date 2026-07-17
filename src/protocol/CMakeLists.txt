# This CMakeLists.txt file is responsible for including all subdirectories within the current source directory.
# It uses the file() command to retrieve all subdirectories relative to the current source directory.
# Then, it iterates over each subdirectory and checks if it contains a CMakeLists.txt file.
# If a subdirectory contains a CMakeLists.txt file, it adds the subdirectory using the add_subdirectory() command.
# If a subdirectory does not contain a CMakeLists.txt file, it displays a warning message and ignores the subdirectory.

file(GLOB all_subdirectories RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)

foreach(subdir ${all_subdirectories})
    set(subdir_cmake_file ${CMAKE_CURRENT_SOURCE_DIR}/${subdir}/CMakeLists.txt)

    if(${subdir} STREQUAL "interface_protocol" AND NOT BUILD_ROS2)
        message(STATUS "Skipping ${subdir} because BUILD_ROS2 is OFF.")
        continue()
    endif()

    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} AND EXISTS ${subdir_cmake_file})
        add_subdirectory(${subdir})
    elseif(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
        message(WARNING "[ ${subdir} ] is a directory but does not contain a CMakeLists.txt file and will be ignored.")
    endif()
endforeach()
