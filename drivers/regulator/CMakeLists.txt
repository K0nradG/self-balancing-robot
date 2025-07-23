if(CONFIG_REGULATOR_DRV)
    set(REGULATOR_DRV_INC include)
    set(REGULATOR_DRV_SRC_DIR src)

    file(GLOB REGULATOR_DRV_SRC ${REGULATOR_DRV_SRC_DIR}/regulator_utils.c
                                ${REGULATOR_DRV_SRC_DIR}/regulator.c
                                ${REGULATOR_DRV_SRC_DIR}/pid_controller.cpp
                                ${REGULATOR_DRV_SRC_DIR}/robot_controller.cpp
                                ${REGULATOR_DRV_SRC_DIR}/control_loop.cpp)

    if(CONFIG_PID_ENABLED)
        list(APPEND REGULATOR_DRV_SRC ${REGULATOR_DRV_SRC_DIR}/pid.c) # Appending necessary so as not to overwrite previous REGULATOR_DRV_SRC value.
    else()
        list(APPEND REGULATOR_DRV_SRC ${REGULATOR_DRV_SRC_DIR}/lqr.c)
    endif() # CONFIG_PID_ENABLED

    zephyr_include_directories(${REGULATOR_DRV_INC})
    zephyr_library_sources(${REGULATOR_DRV_SRC})

endif() #CONFIG_REGULATOR_DRV
