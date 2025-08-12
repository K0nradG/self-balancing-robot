set(BATTERY_LVL_DRV_INC include)
set(BATTERY_LVL_DRV_SRC_DIR src)

file(GLOB BATTERY_LVL_DRV_SRC ${BATTERY_LVL_DRV_SRC_DIR}/*.c)

zephyr_include_directories(${BATTERY_LVL_DRV_INC})
zephyr_library_sources(${BATTERY_LVL_DRV_SRC})

#zephyr_linker_sources(SECTIONS iterables.ld)