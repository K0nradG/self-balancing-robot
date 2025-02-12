if(CONFIG_IMU_DRV)

    set(IMU_DRV_INC include)
    set(IMU_DRV_SRC_DIR src)

    file(GLOB IMU_DRV_SRC ${IMU_DRV_SRC_DIR}/*.c)

    zephyr_include_directories(${IMU_DRV_INC})
    zephyr_library_sources(${IMU_DRV_SRC})

endif() # CONFIG_IMU_DRV

