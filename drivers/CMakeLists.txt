add_subdirectory_ifdef(CONFIG_BATTERY_LEVEL_DRV utils) # Add other drivers flag with OR to conditionally add for build.
add_subdirectory_ifdef(CONFIG_BATTERY_LEVEL_DRV battery_level)