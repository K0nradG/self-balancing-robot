#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include "imu.h"
#include <math.h>
#include <zephyr/init.h>
#include "control_loop.h"
#include "imu_config.h"

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif  // CONFIG_IMU_LOG

#define GYRO_CALIBRATION_SAMPLES 10000

#define ALPHA               0.997f
#define M_PI                3.14159265358979323846f
#define MICRO_PARTS_CONVERT 1e-06f

#define ANGLE_OFFSET 90.0f
#define DEG_TO_RAD   (M_PI / 180.0f)
#define OFFSET       (ANGLE_OFFSET * DEG_TO_RAD)

#define GYRO_X_OFFSET 0.032657f  // -0.029123f
#define GYRO_Y_OFFSET 0.023897f  // 0.066859
#define GYRO_Z_OFFSET -0.019945  // -0.001628

typedef struct gyro_calibration_data
{
    int sample_count;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
} gyro_calibration_data;

static gyro_calibration_data g_calibration_data = {0};
struct device const* imu_dev                    = DEVICE_DT_GET_ONE(invensense_mpu6050);

imu_data s_imu_data = {0};

static int
process_imu(struct device const* dev);

#ifdef CONFIG_MPU6050_TRIGGER

static struct sensor_trigger trigger = {0};

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
calibrate_gyro(struct sensor_value* gyro_data)
{
    if(g_calibration_data.sample_count < GYRO_CALIBRATION_SAMPLES)
    {
        g_calibration_data.gyro_offset_x += (float)sensor_value_to_double(&gyro_data[0]);
        g_calibration_data.gyro_offset_y += (float)sensor_value_to_double(&gyro_data[1]);
        g_calibration_data.gyro_offset_z += (float)sensor_value_to_double(&gyro_data[2]);
        g_calibration_data.sample_count++;
    }
    else
    {
        g_calibration_data.gyro_offset_x /= GYRO_CALIBRATION_SAMPLES;
        g_calibration_data.gyro_offset_y /= GYRO_CALIBRATION_SAMPLES;
        g_calibration_data.gyro_offset_z /= GYRO_CALIBRATION_SAMPLES;

#ifdef CONFIG_IMU_LOG
        static bool log_offsets = true;
        if(log_offsets)
        {
            log_offsets = false;
            platform_log(
                "IMU", LOG_LEVEL_INF, "Gyro calibration complete: X = %f, Y = %f, Z = %f",
                g_calibration_data.gyro_offset_x, g_calibration_data.gyro_offset_y, g_calibration_data.gyro_offset_z);
        }
#endif  // CONFIG_IMU_LOG
    }
}

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

    mpu_reset(1);

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

    set_dlpf();
#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "imu dlpf set");
#endif  // CONFIG_IMU_LOG

    // set_measurement_interval();
#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "meas interval set");
#endif  // CONFIG_IMU_LOG

#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "imu init finished");
#endif  // CONFIG_IMU_LOG

    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static imu_data
get_data(struct sensor_value* accelerometer_data, struct sensor_value* gyro_data, struct sensor_value* temperature)
{
    ARG_UNUSED(temperature);

    static float angle_balance  = 0.0f;
    static float angle_rotation = 0.0f;
    static int64_t last_time    = 0;
    int64_t const current_time  = k_uptime_get();

    // Compute dynamic DT in seconds:
    float const dt = (last_time > 0) ? (current_time - last_time) / 1000.0 : 0.01;  // Default DT if first run

    float const ay = (float)sensor_value_to_double(&accelerometer_data[1]);
    float const az = (float)sensor_value_to_double(&accelerometer_data[2]);

    float const accel_angle = (float)atan2f(ay, az) + OFFSET;  // [radians]
    float gyro_rate_x       = (float)sensor_value_to_double(&gyro_data[0]) + GYRO_X_OFFSET;
    float gyro_rate_y       = (float)sensor_value_to_double(&gyro_data[1]) + GYRO_Y_OFFSET;

    if(dt > 0.01)
    {
        angle_balance = accel_angle;
    }
    else
    {
        angle_balance = ALPHA * (angle_balance + gyro_rate_x * dt) + (1 - ALPHA) * accel_angle;
    }

    angle_rotation += gyro_rate_y * dt;

    // TODO: Maybe wrap around not needed, but setting direction directly by the user.
    // angle_rotation = fmod(angle_rotation, 2.0f * M_PI);  // Wrap around 360.

    last_time           = current_time;
    imu_data const data = {
        .angle_balance = angle_balance, .angle_balance_dt = gyro_rate_x, .angle_rotation = angle_rotation};

    return data;
}

static int
process_imu(struct device const* dev)
{
    static struct sensor_value accelerometer_data[3] = {0};
    static struct sensor_value gyro_data[3]          = {0};
    static struct sensor_value temperature           = {0};

    int ret = sensor_sample_fetch(dev);  // Fetch new data

    ret |= sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accelerometer_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);

    // calibrate_gyro(gyro_data);  // TODO: make Calibration on Kconfig

    if(ret)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu processing failed");
#endif  // CONFIG_IMU_LOG
        return -ENODEV;
    }

    s_imu_data = get_data(accelerometer_data, gyro_data, &temperature);
    trigger_control_loop();

    return ret;
}

imu_data
_get_imu_data(void)
{
    return s_imu_data;
}