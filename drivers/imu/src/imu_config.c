#include "imu_config.h"
#include <zephyr/drivers/i2c.h>

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif

#define MPU6050_I2C_ADDR 0x68

#define DLPF_REG_ADDR 0x1A
#define DLPF_44_HZ_REG_VAL 0x03
#define DLPF_REG_MASK 0x07

#define IMU_MEASUREMENT_INTERVAL_REG_ADDR 0x19
#define IMU_MEASUREMENT_INTERVAL_REG_VAL 0x09

bool
check_dlpf(const struct device* i2c_dev)
{
    uint8_t dlpf_value;
    i2c_reg_read_byte(i2c_dev, MPU6050_I2C_ADDR, DLPF_REG_ADDR, &dlpf_value);

#ifdef CONFIG_LOG_IMU
    platform_log("APP", LOG_LEVEL_INF, "DLPF ustawione na: %d\n", dlpf_value & DLPF_REG_MASK);
#endif

    dlpf_value &= DLPF_REG_MASK;

    if(dlpf_value > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void
set_dlpf(const struct device* i2c_dev)
{
    i2c_reg_write_byte(i2c_dev, MPU6050_I2C_ADDR, DLPF_REG_ADDR, DLPF_44_HZ_REG_VAL);
#ifdef CONFIG_LOG_IMU
    platform_log("APP", LOG_LEVEL_INF, "dlpf set to 44hz");
#endif
}

void
set_mesurement_interval(const struct device* i2c_dev)
{
    i2c_reg_write_byte(i2c_dev, MPU6050_I2C_ADDR, IMU_MEASUREMENT_INTERVAL_REG_ADDR, IMU_MEASUREMENT_INTERVAL_REG_VAL);
#ifdef CONFIG_LOG_IMU
    platform_log("APP", LOG_LEVEL_INF, "imu mesurement interval seto to 100hz");
#endif
}
