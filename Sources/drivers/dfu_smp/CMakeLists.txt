# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(DFU_BLE_DRV_SRC_INC include)
set(DFU_BLE_DRV_SRC_SRC_DIR src)

file(GLOB DFU_BLE_DRV_SRC ${DFU_BLE_DRV_SRC_SRC_DIR}/*.cpp)

zephyr_include_directories(${DFU_BLE_DRV_SRC_INC})
zephyr_library_sources(${DFU_BLE_DRV_SRC})