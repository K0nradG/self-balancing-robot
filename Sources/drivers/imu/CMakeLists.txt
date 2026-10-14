# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(IMU_DRV_INC include)
set(IMU_DRV_SRC_DIR src)

set(IMU_DRV_SRC ${IMU_DRV_SRC_DIR}/imu_config.cpp
                ${IMU_DRV_SRC_DIR}/imu.cpp)

zephyr_include_directories(${IMU_DRV_INC})
zephyr_library_sources(${IMU_DRV_SRC})