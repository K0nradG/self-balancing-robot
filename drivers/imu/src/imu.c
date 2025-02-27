#pragma GCC diagnostic ignored "-Wunused-variable"

#include "imu.h"
#include <math.h>
#include <zephyr/init.h>

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif  // CONFIG_IMU_LOG

#define ALPHA 0.98
#define M_PI 3.14159265358979323846

struct device const* imu_dev = DEVICE_DT_GET_ONE(invensense_mpu6050);

imu_updated_cb_t new_imu_cb;

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
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu not reading/processing data");
#endif  // CONFIG_IMU_LOG
        (void)sensor_trigger_set(dev, trig, NULL);
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
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu not ready");
#endif  // CONFIG_IMU_LOG

        return -ENODEV;
    }

#ifdef CONFIG_MPU6050_TRIGGER
    trigger = (struct sensor_trigger) {
        .type = SENSOR_TRIG_DATA_READY,  // Trigger when data is ready
        .chan = SENSOR_CHAN_ALL,         // Apply to all channels
    };

    if(sensor_trigger_set(imu_dev, &trigger, handle_imu_drdy) < 0)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu cannot configure trigger");
#endif  // CONFIG_IMU_LOG
        return -ENODEV;
    }
#endif  // CONFIG_MPU6050_TRIGGER

#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "imu init finished");
#endif  // CONFIG_IMU_LOG
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void
calculate_angle(
    int* angle_int_part, int* angle_fract_part, struct sensor_value* accelerometer_data, struct sensor_value* gyro_data)
{
    static int64_t last_time = 0;
    int64_t current_time     = k_uptime_get();

    // Compute dynamic DT in seconds
    float dt  = (last_time > 0) ? (current_time - last_time) / 1000.0 : 0.01;  // Default DT if first run
    last_time = current_time;

    float ax = accelerometer_data[0].val1 + accelerometer_data[0].val2 / 1000000.0;
    float ay = accelerometer_data[1].val1 + accelerometer_data[1].val2 / 1000000.0;
    float az = accelerometer_data[2].val1 + accelerometer_data[2].val2 / 1000000.0;

    // float accel_angle = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / M_PI); (-90,90)
    float accel_angle = atan2(ay, az) * (180.0 / M_PI);  //(-180,180)

    float gyro_rate = gyro_data[1].val1 + gyro_data[1].val2 / 1000000.0;

    // integration of angular acceleration from the gyroscope
    static float gyro_angle = 0;
    gyro_angle += gyro_rate * dt;  // imu has its own integration time independent from regualtor

    // complementary filter
    float angle = ALPHA * (gyro_angle) + (1.0f - ALPHA) * accel_angle;

    // Separation into integer and fractional parts (6 decimal places)
    *angle_int_part   = (int)angle;
    *angle_fract_part = (int)((angle - *angle_int_part) * 1000000);
}

static int
process_imu(struct device const* dev)
{
    static struct sensor_value temperature           = {0};
    static struct sensor_value accelerometer_data[3] = {0};
    static struct sensor_value gyro_data[3]          = {0};

    int angle_int_part;
    int angle_fract_part;

    struct imu_data imu_data;

    int ret = sensor_sample_fetch(dev);  // Fetch new data

    ret |= sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accelerometer_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);

    calculate_angle(&angle_int_part, &angle_fract_part, accelerometer_data, gyro_data);

    imu_data.angle_data.angle_int   = angle_int_part;
    imu_data.angle_data.angle_fract = angle_fract_part;

    if(ret)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu processing failed");
#endif  // CONFIG_IMU_LOG
        return -ENODEV;
    }

    if(new_imu_cb)
    {
        new_imu_cb(imu_data);
    }

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
