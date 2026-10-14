# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(BATTERY_LVL_DRV_INC include)
set(BATTERY_LVL_DRV_SRC_DIR src)

set(BATTERY_LVL_DRV_SRC ${BATTERY_LVL_DRV_SRC_DIR}/battery_level.cpp)

zephyr_include_directories(${BATTERY_LVL_DRV_INC})
zephyr_library_sources(${BATTERY_LVL_DRV_SRC})