if(CONFIG_ROBOT_CONTROL)
    set(ROBOT_CONTROL_INC include)
    set(ROBOT_CONTROL_SRC_DIR src)

    file(GLOB ROBOT_CONTROL_SRC ${ROBOT_CONTROL_SRC_DIR}/robot_controller.cpp
                                ${ROBOT_CONTROL_SRC_DIR}/control_loop.cpp)

    if(CONFIG_PID_ENABLED)
        list(APPEND ROBOT_CONTROL_SRC ${ROBOT_CONTROL_SRC_DIR}/pid.cpp) # Appending necessary so as not to overwrite previous ROBOT_CONTROL_SRC value.
        list(APPEND ROBOT_CONTROL_SRC ${ROBOT_CONTROL_SRC_DIR}/low_pass_filter.cpp)
    else()
        list(APPEND ROBOT_CONTROL_SRC ${ROBOT_CONTROL_SRC_DIR}/lqr.cpp)
    endif() # CONFIG_PID_ENABLED

    zephyr_include_directories(${ROBOT_CONTROL_INC})
    zephyr_library_sources(${ROBOT_CONTROL_SRC})

endif() #CONFIG_ROBOT_CONTROL
