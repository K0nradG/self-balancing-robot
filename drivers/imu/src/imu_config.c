#include "imu_config.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif

#define MPU6050_I2C_ADDR 0x68

#define DLPF_REG_ADDR 0x1A
#define DLPF_44_HZ_REG_VAL 0x03

#define IMU_MEASUREMENT_INTERVAL_REG_ADDR 0x19
#define IMU_MEASUREMENT_INTERVAL_REG_VAL 0x01

#define HDC_2080_NODE DT_INST(0, invensense_mpu6050)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(HDC_2080_NODE);

static void
set_sensor_settings(uint8_t reg, uint8_t _configuration)
{
    uint8_t configuration[2] = {reg, _configuration};

    int ret = i2c_write_dt(&dev_i2c, configuration, sizeof(configuration));
#ifdef CONFIG_IMU_LOG
    if(ret != 0)
    {
        platform_log(
            "IMU", LOG_LEVEL_ERR, "Failed to write I2C device %x at Reg. %x", dev_i2c.addr, (double)configuration[0]);
    }
    else
    {
        platform_log("IMU", LOG_LEVEL_INF, "I2C reg write successful.");
    }
#endif  // CONFIG_IMU_LOG
}

void
set_dlpf(void)
{
    set_sensor_settings(DLPF_REG_ADDR, DLPF_44_HZ_REG_VAL);
}

void
set_measurement_interval(void)
{
    set_sensor_settings(IMU_MEASUREMENT_INTERVAL_REG_ADDR, IMU_MEASUREMENT_INTERVAL_REG_VAL);
}
