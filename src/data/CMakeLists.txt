
# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CURRENT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set target name
get_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_NAME)
get_alias_namespace(${CMAKE_CURRENT_SOURCE_DIR} CURRENT_TARGET_ALIAS_NAME)

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp)
file(GLOB_RECURSE souce_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cc ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE test_files  ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cc ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)
list(REMOVE_ITEM souce_files ${test_files})


if(SDK_FRAMEWORK_MODE)
  # src::data is prebuilt in core, supplement with source-distributed params
  # Add _param include path to the imported target so it propagates to all linked targets
  set_property(TARGET EngineAICore::src_data APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/_param"
  )

  # Compile supplementary source files if any exist
  if(souce_files)
    add_library(${CURRENT_TARGET_NAME}_supplement SHARED ${souce_files})
    target_include_directories(${CURRENT_TARGET_NAME}_supplement
      PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/_param
    )
    target_link_libraries(${CURRENT_TARGET_NAME}_supplement
      PUBLIC
        src::data
    )
    install(TARGETS ${CURRENT_TARGET_NAME}_supplement
      RUNTIME DESTINATION bin
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
    )
  endif()

  return()
endif()

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/_param>
)

# Set link libraries of target
target_link_libraries(${CURRENT_TARGET_NAME}
  PUBLIC 
    yaml-cpp::yaml-cpp
    glog::glog
    fmt::fmt
    Eigen3::Eigen
    src::common::parameter
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
