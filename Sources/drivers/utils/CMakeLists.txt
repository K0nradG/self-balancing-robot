set(UTIL_DRV_INC include)
set(UTIL_DRV_SRC_DIR src)

file(GLOB UTIL_DRV_SRC ${UTIL_DRV_SRC_DIR}/*.cpp)

zephyr_include_directories(${UTIL_DRV_INC})
zephyr_library_sources(${UTIL_DRV_SRC})