#include "imu_config.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#else
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif  // CONFIG_IMU_LOG

#define MPU6050_I2C_ADDR 0x68

#define DLPF_REG_ADDR 0x1A
#define DLPF_44_HZ_REG_VAL 0x02

#define IMU_MEASUREMENT_INTERVAL_REG_ADDR 0x19
#define IMU_MEASUREMENT_INTERVAL_REG_VAL 0x01

#define PWR_MGMT_1 0x68

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

static int
get_sensor_settings(uint8_t reg, uint8_t* value)
{
    int ret          = 0;
    uint8_t reg_addr = reg;

    ret = i2c_write_dt(&dev_i2c, &reg_addr, sizeof(reg_addr));
    if(ret != 0)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "Failed to write register address %x", reg);
#endif
        return ret;
    }

    // Read register value:
    ret = i2c_read_dt(&dev_i2c, value, sizeof(*value));
    if(ret != 0)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "Failed to read value from register %x", reg);
#endif
        return ret;
    }

#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "I2C reg read successful: Reg %x, Value %x", reg, *value);
#endif

    return 0;
}

void
mpu_reset(uint8_t conf)
{
    uint8_t tmp = 0u;
    get_sensor_settings(PWR_MGMT_1, &tmp);
    tmp &= ~(1 << 7);
    tmp |= ((conf & 0x1) << 7);
    set_sensor_settings(PWR_MGMT_1, tmp);
}

void
set_dlpf(void)
{
    uint8_t tmp = 0u;
    get_sensor_settings(DLPF_REG_ADDR, &tmp);
    tmp |= DLPF_44_HZ_REG_VAL & 0x7;
    set_sensor_settings(DLPF_REG_ADDR, tmp);
}

void
set_measurement_interval(void)
{
    set_sensor_settings(IMU_MEASUREMENT_INTERVAL_REG_ADDR, IMU_MEASUREMENT_INTERVAL_REG_VAL);
}
