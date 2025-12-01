set(INTERFACE_DRV_INC include)
set(INTERFACE_DRV_SRC_DIR src)

file(GLOB INTERFACE_DRV_SRC ${INTERFACE_DRV_SRC_DIR}/*.cpp)

zephyr_include_directories(${INTERFACE_DRV_INC})
zephyr_library_sources(${INTERFACE_DRV_SRC})