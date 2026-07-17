list(APPEND ENGINEAI_ROBOTICS_THIRD_PARTY /opt/engineai_robotics_third_party)
message(STATUS "Calculated Crashpad third-party path: ${ENGINEAI_ROBOTICS_THIRD_PARTY}")

add_library(crashpad_client INTERFACE)

target_include_directories(crashpad_client INTERFACE
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}/include/crashpad/"
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}/include/crashpad/client/"
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}/include/crashpad/util/"
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}/include/crashpad/snapshot/"
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}/include/crashpad/third_party/mini_chromium/"
)

target_link_libraries(crashpad_client INTERFACE
    "${ENGINEAI_ROBOTICS_THIRD_PARTY}/lib/libclient.a"
)
