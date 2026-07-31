cmake_minimum_required(VERSION 3.16)

project(engineai_robotics LANGUAGES C CXX)

# Set some variables
list(APPEND ENGINEAI_ROBOTICS_DIR ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)
list(APPEND ENGINEAI_ROBOTICS_HARDWARE /opt/engineai_robotics_hardware)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

# Find packages
find_package(GTest REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)
find_package(pinocchio REQUIRED)
find_package(qpOASES REQUIRED)

# ROS 2 dependencies (only when building ROS 2 components)
if(BUILD_ROS2)
  set(ROS2_VERSION "humble" CACHE STRING "ROS 2 distro to use (e.g. humble, jazzy)")
  set_property(CACHE ROS2_VERSION PROPERTY STRINGS "humble" "jazzy")
  message(STATUS "ROS 2 version: ${ROS2_VERSION}")

  # Ensure CMake can find ROS 2 installed under /opt/ros/${ROS2_VERSION}
  list(APPEND CMAKE_PREFIX_PATH "/opt/ros/${ROS2_VERSION}")
  # Fast-CDR is required by Fast RTPS based type support
  find_package(fastcdr REQUIRED)
endif()

include(AddGoogleTest)
include(GetNamespace)
include(GetAliasNamespace)

# Set some options
# In case of any exception, check the option variables in `build.sh` and `build\CMakeCache.txt` 
set(BUILD_TYPE "releasewithdebinfo" CACHE STRING "Build type: release, debug, releasewithdebinfo")
set_property(CACHE BUILD_TYPE PROPERTY STRINGS "release" "debug" "releasewithdebinfo")
option(BUILD_TESTS "Build tests" ON)
option(BUILD_ROS2 "Build ros2" ON)

# Some necessary settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/_install)

# Optimizes install process - skips checking dependencies during install
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY TRUE)
# Reduces install output verbosity
set(CMAKE_INSTALL_MESSAGE LAZY)

if(BUILD_TYPE STREQUAL "release")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Set build type as Release." FORCE)
    message(STATUS "Build [Release] version")
elseif(BUILD_TYPE STREQUAL "debug")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set build type as Debug." FORCE)
    message(STATUS "Build [Debug] version")
elseif(BUILD_TYPE STREQUAL "releasewithdebinfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Set build type as RelWithDebInfo." FORCE)
    message(STATUS "Build [RelWithDebInfo] version")
else()
    message(FATAL_ERROR "Invalid BUILD_TYPE: ${BUILD_TYPE}. Must be one of: release, debug, releasewithdebinfo")
endif()

if(BUILD_DCHECK)
    add_compile_options(-DDCHECK_ALWAYS_ON)
    message(STATUS "Build [DCHECK] version")
endif(BUILD_DCHECK)

set(BUILD_SHARED_LIBS ON)
if(UNIX)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    if (BUILD_TYPE STREQUAL "release")
        add_compile_options(-Werror=return-type -O3 -fno-omit-frame-pointer)
    else()
        add_compile_options(-Werror=return-type -fno-omit-frame-pointer)
    endif()
endif()

set(SDK_CORE_DIR_NAME "core" CACHE STRING "Native SDK core libraries directory name")
set(_SDK_CONFIG_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${SDK_CORE_DIR_NAME}/cmake/EngineAICoreConfig.cmake")
if(EXISTS "${_SDK_CONFIG_PATH}")
    message(STATUS "Detected SDK package, loading pre-built core libraries...")
    list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${SDK_CORE_DIR_NAME}/cmake")
    find_package(EngineAICore REQUIRED)
    include_directories(${ENGINEAI_ROBOTICS_HARDWARE}/include)
    message(STATUS "SDK Framework Mode: ${SDK_FRAMEWORK_MODE}")
    if(DEFINED EngineAICore_LIB_DIR AND EXISTS "${EngineAICore_LIB_DIR}")
        message(STATUS "SDK Framework Mode: staging core libs into install prefix lib/")
        get_filename_component(_engineai_core_lib_dir "${EngineAICore_LIB_DIR}" REALPATH)
        install(DIRECTORY "${_engineai_core_lib_dir}/"
            DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        )
        unset(_engineai_core_lib_dir)
    else()
        message(WARNING "EngineAICore_LIB_DIR not found, skip staging core libs to install prefix")
    endif()
    unset(_SDK_CONFIG_PATH)
endif()


# Add subdirectories
add_subdirectory(src)

