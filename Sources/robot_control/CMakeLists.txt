# Copyright 2026 Filip Dymczyk and Konrad Grucel

if(CONFIG_ROBOT_CONTROL)
    
    set(ROBOT_CONTROL_INC include)
    set(CONTROLLERS_INC ${ROBOT_CONTROL_INC}/controllers)
    set(TRAJECTORY_INC ${ROBOT_CONTROL_INC}/trajectory)
    set(UTILS_INC ${ROBOT_CONTROL_INC}/utils)

    set(ROBOT_CONTROL_SRC_DIR src)
    set(CONTROLLERS_SRC_DIR ${ROBOT_CONTROL_SRC_DIR}/controllers)
    set(TRAJECTORY_SRC_DIR ${ROBOT_CONTROL_SRC_DIR}/trajectory)
    set(UTILS_SRC_DIR ${ROBOT_CONTROL_SRC_DIR}/utils)

    set(ROBOT_CONTROL_SRC ${ROBOT_CONTROL_SRC_DIR}/control_loop.cpp
                            ${ROBOT_CONTROL_SRC_DIR}/data_manager.cpp
                            ${ROBOT_CONTROL_SRC_DIR}/drivers_initializer.cpp
                            ${ROBOT_CONTROL_SRC_DIR}/main_state_machine.cpp
                            ${ROBOT_CONTROL_SRC_DIR}/robot_controller.cpp)
    if(CONFIG_ROBOT_CONTROL_LOG AND CONFIG_BLUETOOTH_DRV)
        list(APPEND ROBOT_CONTROL_SRC ${ROBOT_CONTROL_SRC_DIR}/telemetry.cpp)
    endif()
    
    set(CONTROLLERS_SRC ${CONTROLLERS_SRC_DIR}/pid.cpp)
    if(NOT CONFIG_PID_ENABLED)
        list(APPEND CONTROLLERS_SRC ${CONTROLLERS_SRC_DIR}/lqr.cpp)
    endif()

    set(TRAJECTORY_SRC ${TRAJECTORY_SRC_DIR}/trajectory_manager.cpp
                        ${TRAJECTORY_SRC_DIR}/trajectory_state_machine.cpp)

    set(UTILS_SRC ${UTILS_SRC_DIR}/low_pass_filter.cpp
                    ${UTILS_SRC_DIR}/ramp.cpp
                    ${UTILS_SRC_DIR}/saturation.cpp)

    zephyr_include_directories(${ROBOT_CONTROL_INC} 
                                ${CONTROLLERS_INC} 
                                ${TRAJECTORY_INC} 
                                ${UTILS_INC})

    zephyr_library_sources(${ROBOT_CONTROL_SRC}
                            ${CONTROLLERS_SRC}
                            ${TRAJECTORY_SRC}
                            ${UTILS_SRC})

endif() #CONFIG_ROBOT_CONTROL
