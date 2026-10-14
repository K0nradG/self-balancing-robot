# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(DFU_BLE_DRV_SRC_INC include)
set(DFU_BLE_DRV_SRC_SRC_DIR src)

set(DFU_BLE_DRV_SRC ${DFU_BLE_DRV_SRC_SRC_DIR}/app_version.cpp
                    ${DFU_BLE_DRV_SRC_SRC_DIR}/dfu_ble.cpp)

zephyr_include_directories(${DFU_BLE_DRV_SRC_INC})
zephyr_library_sources(${DFU_BLE_DRV_SRC})