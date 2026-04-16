
# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set target name
get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
list(REMOVE_ITEM souce_files ${test_files})

# Add target
add_library(${CURRENT_TARGET_NAME} SHARED)
add_library(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})

# Set source file of target
target_sources(${CURRENT_TARGET_NAME} PRIVATE ${souce_files})

# Set head files of target
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${head_files})

# Set include path of target
target_include_directories(${CURRENT_TARGET_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Set link libraries of target
target_link_libraries(${CURRENT_TARGET_NAME}
  PUBLIC
  yaml-cpp::yaml-cpp
  glog::glog
  fmt::fmt
  Eigen3::Eigen
  src::data
  src::common::monitor
  src::common::tool
  src::common::containers
  src::core::math
  rclcpp::rclcpp
  interface_protocol__rosidl_generator_cpp
  interface_protocol__rosidl_typesupport_cpp
  interface_protocol__rosidl_typesupport_fastrtps_cpp
  interface_protocol__rosidl_typesupport_introspection_cpp
)

# Add ament dependencies for ROS2 packages
ament_target_dependencies(${CURRENT_TARGET_NAME}
  PUBLIC
  geometry_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
)

# Set installation of target
install(TARGETS ${CURRENT_TARGET_NAME}
  EXPORT ${CURRENT_TARGET_ALIAS_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Set tests
if(BUILD_TESTS AND test_files)
  add_googletest(test_${CURRENT_TARGET_NAME}
    SRCS
    ${test_files}
    DEPS
    GTest::GTest
    GTest::Main
    pthread
    src::core::math
    ${CURRENT_TARGET_NAME}
  )
endif()
