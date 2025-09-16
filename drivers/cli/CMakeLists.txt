set(SHELL_DRV_INC include)
set(SHELL_DRV_SRC_DIR src)

file(GLOB SHELL_DRV_SRC ${SHELL_DRV_SRC_DIR}/*.c)

zephyr_include_directories(${SHELL_DRV_INC})
zephyr_library_sources(${SHELL_DRV_SRC})