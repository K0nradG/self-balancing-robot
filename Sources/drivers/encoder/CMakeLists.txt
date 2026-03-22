# Copyright 2026 Filip Dymczyk and Konrad Grucel

set(ENCODER_DRV_INC include)
set(ENCODER_DRV_SRC_DIR src)

file(GLOB ENCODER_DRV_SRC ${ENCODER_DRV_SRC_DIR}/*.cpp)

zephyr_include_directories(${ENCODER_DRV_INC})
zephyr_library_sources(${ENCODER_DRV_SRC})