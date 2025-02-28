if(CONFIG_REGULATOR_DRV)
set(REGULATOR_DRV_INC include)
set(REGULATOR_DRV_SRC_DIR src)

file(GLOB REGULATOR_DRV_SRC ${REGULATOR_DRV_SRC_DIR}/*.c)

zephyr_include_directories(${REGULATOR_DRV_INC})
zephyr_library_sources(${REGULATOR_DRV_SRC})

endif() #CONFIG_REGULATOR_DRV
