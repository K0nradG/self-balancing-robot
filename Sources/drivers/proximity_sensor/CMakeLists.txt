# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(PROXIMITY_SENSOR_DRV_INC include)
set(PROXIMITY_SENSOR_DRV_SRC_DIR src)

file(GLOB PROXIMITY_SENSOR_DRV_SRC ${PROXIMITY_SENSOR_DRV_SRC_DIR}/*.cpp)

zephyr_include_directories(${PROXIMITY_SENSOR_DRV_INC})
zephyr_library_sources(${PROXIMITY_SENSOR_DRV_SRC})