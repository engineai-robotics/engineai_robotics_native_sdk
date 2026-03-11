cmake_minimum_required(VERSION 3.16)

project(engineai_robotics_simulation_mujoco LANGUAGES C CXX)

# Set some variables
list(APPEND SIMULATION_MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${SIMULATION_MUJOCO_DIR}/simulate/cmake)
list(APPEND CMAKE_MODULE_PATH ${SIMULATION_MUJOCO_DIR}/../../cmake)
list(APPEND CMAKE_PREFIX_PATH ${ENGINEAI_ROBOTICS_THIRD_PARTY})

# Use local glfw/lodepng from _deps (e.g. from mujoco_deps tar) instead of FetchContent
option(MUJOCO_USE_LOCAL_DEPS "Use local glfw and lodepng from _deps, skip FetchContent download" OFF)
set(MUJOCO_LOCAL_DEPS_DIR "${SIMULATION_MUJOCO_DIR}/_deps" CACHE PATH "Directory containing glfw3-src and lodepng-src")

# Find packages
find_package(GTest REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)
find_package(mujoco REQUIRED)

include(AddGoogleTest)
include(GetNamespace)
include(GetAliasNamespace)

# Set some options
option(BUILD_RELEASE "Build release version" ON)

# Some necessary settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/_install)

if(BUILD_RELEASE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Set build type as Release." FORCE)
    message(STATUS "Build [Release] version")
else(BUILD_RELEASE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Set build type as Debug." FORCE)
    message(STATUS "Build [Debug] version")
endif(BUILD_RELEASE)

set(BUILD_SHARED_LIBS ON)
if(UNIX)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_compile_options(-Werror=return-type -O3)
endif()

set(SDK_CORE_DIR_NAME "core" CACHE STRING "Native SDK core libraries directory name")
set(EngineAICore_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../${SDK_CORE_DIR_NAME}/cmake" CACHE PATH
    "Path to EngineAICoreConfig.cmake")
if(EXISTS "${EngineAICore_DIR}/EngineAICoreConfig.cmake")
    message(STATUS "Detected SDK package, loading pre-built core libraries from: ${EngineAICore_DIR}")
    list(PREPEND CMAKE_PREFIX_PATH "${EngineAICore_DIR}")
    find_package(EngineAICore CONFIG REQUIRED)
    include_directories(${ENGINEAI_ROBOTICS_HARDWARE}/include)
    message(STATUS "SDK Framework Mode: ${SDK_FRAMEWORK_MODE}")
    if(DEFINED EngineAICore_LIB_DIR AND EXISTS "${EngineAICore_LIB_DIR}")
        message(STATUS "SDK Framework Mode: staging core libs into install prefix lib/")
        install(DIRECTORY "${EngineAICore_LIB_DIR}/"
            DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        )
    else()
        message(WARNING "EngineAICore_LIB_DIR not found, skip staging core libs to install prefix")
    endif()
else()
    message(STATUS "EngineAICoreConfig.cmake not found at: ${EngineAICore_DIR}, building without pre-built core package.")
endif()

# Add subdirectories
# Template sets for get_namespace() and get_alias_namespace()
set(ENGINEAI_ROBOTICS_DIR ${SIMULATION_MUJOCO_DIR})
add_subdirectory(src)
add_subdirectory(simulate)

# Build simulate executable
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/simulate/array_safety.h)
file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME} PRIVATE ${souce_files})
target_sources(${PROJECT_NAME} INTERFACE ${head_files})
target_compile_options(${PROJECT_NAME} PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
    libsimulate
    mujoco::mujoco
    glfw
    Threads::Threads
    lodepng
    Eigen3::Eigen
    lcm
    src::common::base
    src::common::monitor
    src::common::parameter
    src::common::tool
    src::data
    src::lcm_interface
)

# target_link_options(${PROJECT_NAME} PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
# message(STATUS "---------------Building simulate executable." ${MUJOCO_SIMULATE_LINK_OPTIONS})

# Set installation of target
# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
# )

# message(STATUS "---------------Building simulate executable." ${_INSTALL_SIMULATE})
# if(_INSTALL_SIMULATE)

# include(TargetAddRpath)

# # Add support to RPATH for the samples.
# target_add_rpath(
#     TARGETS
#     simulate
#     INSTALL_DIRECTORY
#     "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
#     LIB_DIRS
#     "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
#     DEPENDS
#     MUJOCO_ENABLE_RPATH
# )

# install(
#     TARGETS simulate
#     EXPORT ${PROJECT_NAME}
#     RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
#     LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
#     ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
#     BUNDLE DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
#     PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT simulate
# )

# if(NOT MUJOCO_SIMULATE_USE_SYSTEM_GLFW)
#     # We downloaded GLFW. Depending if it is a static or shared LIBRARY we might
#     # need to install it.
#     get_target_property(MJ_GLFW_LIBRARY_TYPE glfw TYPE)
#     if(MJ_GLFW_LIBRARY_TYPE STREQUAL SHARED_LIBRARY)
#     install(
#         TARGETS glfw
#         EXPORT ${PROJECT_NAME}
#         RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
#         LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
#         ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
#         PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT simulate
#     )
#     endif()
# endif()
# endif()