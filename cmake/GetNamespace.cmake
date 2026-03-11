function(get_namespace CURRENT_DIR RESULT)
    get_property(ROOT_DIR VARIABLE PROPERTY ENGINEAI_ROBOTICS_DIR)

    if(NOT ROOT_DIR MATCHES "/$")
        set(ROOT_DIR "${ROOT_DIR}/")
    endif()

    if (NOT "${CURRENT_DIR}" MATCHES "^${ROOT_DIR}")
        message(FATAL_ERROR "Current directory [${CURRENT_DIR}] is not under root directory [${ROOT_DIR}].")
    endif()

    string(REGEX REPLACE "^${ROOT_DIR}" "" DIFF_PART "${CURRENT_DIR}")

    # Replace `/` with `_` and remove the first `_` if it exists
    string(REPLACE "/" "_" RESULT_VAR "${DIFF_PART}")
    string(REGEX REPLACE "^_" "" RESULT_VAR "${RESULT_VAR}")
    set(${RESULT} "${RESULT_VAR}" PARENT_SCOPE)
endfunction()
