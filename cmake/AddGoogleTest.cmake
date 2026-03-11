# Convenience function for adding tests based on `googletest`.
# add_googletest(<target>
#   SRCS <source1> [<source2>...]
#   DEPS <lib1> [<lib2>...]
# )
#
# Usage:
# add_googletest(foo_test SRCS foo_test.cc foo.cc DEPS gtest_main)
function(add_googletest target)
  if(NOT BUILD_TESTS)
    return()
  endif()

  cmake_parse_arguments(TEST "" "" "SRCS;DEPS" ${ARGN})
  if(NOT target)
    message(FATAL_ERROR "Must provide a target")
  endif()
  if(NOT TEST_SRCS)
    message(FATAL_ERROR "Must provide sources")
  endif()
  if(NOT TEST_DEPS)
    message(FATAL_ERROR "Must provide dependencies, at least GTest::Main")
  endif()

  add_executable(${target} "")
  target_sources(${target} PRIVATE ${TEST_SRCS})
  target_include_directories(${target} PRIVATE ${ENGINEAI_ROBOTICS_DIR})
  target_link_libraries(${target} PRIVATE ${TEST_DEPS})
  add_test(${target} ${target})
  ## If you want to automate googletest, replace `add_test()` command with the following command
  # include(GoogleTest)
  # gtest_discover_tests(${target})
endfunction(add_googletest)
