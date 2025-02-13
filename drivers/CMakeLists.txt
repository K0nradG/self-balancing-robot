if(DEFINED CONFIG_BATTERY_LEVEL_DRV OR DEFINED CONFIG_MOTOR_CONTROLLER_DRV)
    add_subdirectory(utils)
endif()

add_subdirectory_ifdef(CONFIG_BATTERY_LEVEL_DRV battery_level)
add_subdirectory_ifdef(CONFIG_MOTOR_CONTROLLER_DRV motor_controller)