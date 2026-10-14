# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(INTERFACE_DRV_INC include)
set(INTERFACE_DRV_SRC_DIR src)

set(INTERFACE_DRV_SRC ${INTERFACE_DRV_SRC_DIR}/interface.cpp)

zephyr_include_directories(${INTERFACE_DRV_INC})
zephyr_library_sources(${INTERFACE_DRV_SRC})