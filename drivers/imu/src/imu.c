#pragma GCC diagnostic ignored "-Wunused-variable"

#include "imu.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include "utils.h"

#define APPLICATION_INIT_PRIORITY 99

LOG_MODULE_REGISTER(imu, CONFIG_BAT_LVL_LOG_LEVEL);

static struct sensor_value temperature           = {0};
static struct sensor_value accelerometer_data[3] = {0};
static struct sensor_value gyro_data[3]          = {0};

struct device const*         imu_dev = DEVICE_DT_GET_ONE(invensense_mpu6050);
static struct sensor_trigger trigger;
imu_updated_cb_t             new_imu_cb;

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

// Interrupt handler:
static void
handle_imu_drdy(struct device const* dev, struct sensor_trigger const* trig);

static int
process_imu(struct device const* dev);

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb);

static int
init(void)
{
    int ret = 0;

    bool const is_imu_device_ready = device_is_ready(imu_dev);

    __ASSERT(is_imu_device_ready, "IMU device not ready!");

#ifdef CONFIG_MPU6050_TRIGGER
    trigger = (struct sensor_trigger) {
        .type = SENSOR_TRIG_DATA_READY,  // Trigger when data is ready
        .chan = SENSOR_CHAN_ALL,         // Apply to all channels
    };

    if(sensor_trigger_set(imu_dev, &trigger, handle_imu_drdy) < 0)
    {
        LOG_DBG("Cannot configure trigger...");
    }
    else
    {
        LOG_DBG("IMU configured for triggered sampling");
    }
#endif
    return ret;
}

SYS_INIT(init, APPLICATION, APPLICATION_INIT_PRIORITY);

// Interrupt handler:
static void
handle_imu_drdy(struct device const* dev, struct sensor_trigger const* trig)
{
    int ret = process_imu(dev);  // Read and process IMU data

    if(ret != 0)
    {
        LOG_ERR("IMU triggering cancelled due to error!");
        (void)sensor_trigger_set(dev, trig, NULL);  // Disable trigger if error
    }
}

static int
process_imu(struct device const* dev)
{
    int ret = sensor_sample_fetch(dev);  // Fetch new data

    if(ret != 0)
    {
        LOG_ERR("Failed to fetch sensor sample! Error code: %d", ret);
        return ret;
    }

    ret = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accelerometer_data);
    if(ret != 0)
    {
        LOG_ERR("Failed to get accelerometer data! Error code: %d", ret);
        return ret;
    }

    ret = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
    if(ret != 0)
    {
        LOG_ERR("Failed to get gyroscope data! Error code: %d", ret);
        return ret;
    }

    ret = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
    if(ret != 0)
    {
        LOG_ERR("Failed to get temperature data! Error code: %d", ret);
        return ret;
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
