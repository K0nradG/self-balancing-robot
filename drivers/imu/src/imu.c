#pragma GCC diagnostic ignored "-Wunused-variable"

#include "imu.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>

#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif
#endif

static struct sensor_value temperature           = {0};
static struct sensor_value accelerometer_data[3] = {0};
static struct sensor_value gyro_data[3]          = {0};

struct device const* imu_dev = DEVICE_DT_GET_ONE(invensense_mpu6050);

imu_updated_cb_t new_imu_cb;

struct sensor_value const* const
get_gyro_data(void)
{
    return gyro_data;
}

struct sensor_value const* const
get_accelerometer_data(void)
{
    return accelerometer_data;
}

struct sensor_value const
get_temperature(void)
{
    return temperature;
}

static int
process_imu(struct device const* dev);

#ifdef CONFIG_MPU6050_TRIGGER

static struct sensor_trigger trigger;

// Interrupt handler:
static void
handle_imu_drdy(struct device const* dev, struct sensor_trigger const* trig)
{
    int ret = process_imu(dev);  // Read and process IMU data

    if(ret != 0)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu not reading/processing data");
#endif
#endif
        (void)sensor_trigger_set(dev, trig, NULL);  // Disable trigger if error
    }
}

#endif  // CONFIG_MPU6050_TRIGGER

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb);

static int
init(void)
{
    bool const is_imu_device_ready = device_is_ready(imu_dev);

    if(!is_imu_device_ready)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu not ready");
#endif
#endif

        return -ENODEV;
    }

#ifdef CONFIG_MPU6050_TRIGGER
    trigger = (struct sensor_trigger) {
        .type = SENSOR_TRIG_DATA_READY,  // Trigger when data is ready
        .chan = SENSOR_CHAN_ALL,         // Apply to all channels
    };

    if(sensor_trigger_set(imu_dev, &trigger, handle_imu_drdy) < 0)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu cannot configure trigger");
#endif
#endif
        return -ENODEV;
    }
#endif  // CONFIG_MPU6050_TRIGGER

#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "imu init finished");
#endif
#endif
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static int
process_imu(struct device const* dev)
{
    int ret = sensor_sample_fetch(dev);  // Fetch new data

    ret |= sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accelerometer_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);

    if(ret)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu processing failed");
#endif
#endif
        return -ENODEV;
    }

    new_imu_cb();

    return ret;
}

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb)
{
    if(_new_imu_cb)
    {
        new_imu_cb = _new_imu_cb;
    }
}
