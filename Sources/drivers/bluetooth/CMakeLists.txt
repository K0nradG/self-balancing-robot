# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(BLUETOOTH_DRV_SRC_INC include)
set(BLUETOOTH_DRV_SRC_SRC_DIR src)

set(BLUETOOTH_DRV_SRC_SRC ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_connection.cpp
                            ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_payload_reader.cpp
                            ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_payload_writer.cpp
                            ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_protocol_helpers.cpp
                            ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_service.cpp
                            ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_setup.cpp
                            ${BLUETOOTH_DRV_SRC_SRC_DIR}/ble_transfer_handler.cpp)

zephyr_include_directories(${BLUETOOTH_DRV_SRC_INC})
zephyr_library_sources(${BLUETOOTH_DRV_SRC_SRC})