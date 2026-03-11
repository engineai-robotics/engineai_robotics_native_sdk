
# Set third-party libraries
include(${CMAKE_SOURCE_DIR}/cmake/Crashpad.cmake)

# Include auto register function
include(${CMAKE_SOURCE_DIR}/cmake/AutoRegisterRunners.cmake)

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
  list(FILTER souce_files EXCLUDE REGEX "auto_register_.*\\.cc$")
endif()

# Add target
add_executable(${CURRENT_TARGET_NAME})
add_executable(${CURRENT_TARGET_ALIAS_NAME} ALIAS ${CURRENT_TARGET_NAME})

# Set source file of target
target_sources(${CURRENT_TARGET_NAME} PRIVATE ${souce_files})

# Set head files of target
target_sources(${CURRENT_TARGET_NAME} INTERFACE ${head_files})

# Set include path of target
target_include_directories(${CURRENT_TARGET_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/>
)

# Set link libraries of target
target_link_libraries(${CURRENT_TARGET_NAME}
  PRIVATE
    src::app
    crashpad_client
)

# Auto-register runners (normal / SDK package / SDK framework modes)
auto_register_runners(${CURRENT_TARGET_NAME})

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
    ${CURRENT_TARGET_NAME}
)
endif()
