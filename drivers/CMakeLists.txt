if(DEFINED CONFIG_BATTERY_LEVEL_DRV OR DEFINED CONFIG_IMU_DRV)
    add_subdirectory(utils)
endif()

add_subdirectory_ifdef(CONFIG_BATTERY_LEVEL_DRV battery_level)