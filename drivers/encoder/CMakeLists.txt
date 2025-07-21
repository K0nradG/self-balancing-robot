if(CONFIG_ENCODER_DRV)

    set(ENCODER_DRV_INC include)
    set(ENCODER_DRV_SRC_DIR src)

    file(GLOB ENCODER_DRV_SRC ${ENCODER_DRV_SRC_DIR}/*.c)

    zephyr_include_directories(${ENCODER_DRV_INC})
    zephyr_library_sources(${ENCODER_DRV_SRC})

endif() # CONFIG_ENCODER_DRV
