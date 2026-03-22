# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(BLUETOOTH_DRV_SRC_INC include)
set(BLUETOOTH_DRV_SRC_SRC_DIR src)

file(GLOB BLUETOOTH_DRV_SRC_SRC ${BLUETOOTH_DRV_SRC_SRC_DIR}/*.cpp)

zephyr_include_directories(${BLUETOOTH_DRV_SRC_INC})
zephyr_library_sources(${BLUETOOTH_DRV_SRC_SRC})