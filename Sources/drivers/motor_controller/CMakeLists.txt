# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(MOTOR_CONTROLLER_DRV_INC include)
set(MOTOR_CONTROLLER_DRV_SRC_DIR src)

set(MOTOR_CONTROLLER_DRV_SRC ${MOTOR_CONTROLLER_DRV_SRC_DIR}/motor_controller.cpp)

zephyr_include_directories(${MOTOR_CONTROLLER_DRV_INC})
zephyr_library_sources(${MOTOR_CONTROLLER_DRV_SRC})