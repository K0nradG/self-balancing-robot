set(LOGGER_DRV_INC include)
set(LOGGER_DRV_SRC_DIR src)

file(GLOB LOGGER_DRV_SRC ${LOGGER_DRV_SRC_DIR}/*.c)

zephyr_include_directories(${LOGGER_DRV_INC})
zephyr_library_sources(${LOGGER_DRV_SRC})