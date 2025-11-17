# Templated class needs to be inlined, thus putting the declarations and definitions inside the .h file
set(LOGGER_DRV_INC include)

zephyr_include_directories(${LOGGER_DRV_INC})