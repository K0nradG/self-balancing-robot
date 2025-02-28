#include "imu_config.h"
#include <zephyr/drivers/i2c.h>

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif

#define MPU6050_I2C_ADDR 0x68
#define DLPF_REG_ADDR 0x1A
#define DLPF_44_HZ_REG_VAL 0x03

void
set_dlpf(const struct device* i2c_dev)
{
    i2c_reg_write_byte(i2c_dev, MPU6050_I2C_ADDR, DLPF_REG_ADDR, DLPF_44_HZ_REG_VAL);
#ifdef CONFIG_LOG_IMU
    platform_log("APP", LOG_LEVEL_INF, "dlpf set to 44hz");
#endif
}
