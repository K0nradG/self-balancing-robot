set(SUPERVISOR_DRV_INC include)
set(SUPERVISOR_DRV_SRC_DIR src)

file(GLOB SUPERVISOR_DRV_SRC ${SUPERVISOR_DRV_SRC_DIR}/*.c)

zephyr_include_directories(${SUPERVISOR_DRV_INC})
zephyr_library_sources(${SUPERVISOR_DRV_SRC})
