#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include "imu.h"
#include <zephyr/init.h>
#include "control_loop.h"
#include "imu_config.h"

#ifdef CONFIG_IMU_LOG
#include "logger.h"
#endif  // CONFIG_IMU_LOG

#ifdef CONFIG_IMU_CALIBRATE_GYRO
#define GYRO_CALIBRATION_SAMPLES 100000
#endif  // CONFIG_IMU_CALIBRATE_GYRO

#define ALPHA               0.992f
#define M_PI                3.14159265358979323846f
#define MILLI_PARTS_CONVERT 1e-03f

#define ANGLE_OFFSET         -90.0f
#define DEG_TO_RAD           (M_PI / 180.0f)
#define ACCELEROMETER_OFFSET (ANGLE_OFFSET * DEG_TO_RAD)

// "Balancing" position measurement:
// #define GYRO_X_DRIFT -0.159268f
// #define GYRO_Y_DRIFT -0.318539f
// #define GYRO_Z_DRIFT -0.393751f

// #define GYRO_X_DRIFT -0.079107f
// #define GYRO_Y_DRIFT -0.097314f
// #define GYRO_Z_DRIFT -0.024064f

// #define GYRO_X_DRIFT -2.689413f
// #define GYRO_Y_DRIFT -1.387693f
// #define GYRO_Z_DRIFT -0.762965f

#define GYRO_X_DRIFT -2.218894f
#define GYRO_Y_DRIFT -1.262870f
#define GYRO_Z_DRIFT -0.658018f

// // Laying down measurement:
// #define GYRO_X_DRIFT 0.001439f
// #define GYRO_Y_DRIFT -0.5362055f
// #define GYRO_Z_DRIFT -0.2785235f

// // Standing still measurement:
// #define GYRO_X_DRIFT -0.55392f
// #define GYRO_Y_DRIFT -0.5578995f
// #define GYRO_Z_DRIFT -0.4397565f

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
    int const ret = process_imu(dev);  // Read and process IMU data

    if(ret != 0)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu not reading/processing data");
#endif  // CONFIG_IMU_LOG
        (void)sensor_trigger_set(dev, trig, NULL);
    }
}

#endif  // CONFIG_MPU6050_TRIGGER

#ifdef CONFIG_IMU_CALIBRATE_GYRO
void
calibrate_gyro(struct sensor_value* gyro_data)
{
    static bool calibration_finished = false;
    if(calibration_finished)
    {
        return;
    }

    if(g_calibration_data.sample_count < GYRO_CALIBRATION_SAMPLES)
    {
        g_calibration_data.gyro_offset_x += sensor_value_to_float(&gyro_data[0]);
        g_calibration_data.gyro_offset_y += sensor_value_to_float(&gyro_data[1]);
        g_calibration_data.gyro_offset_z += sensor_value_to_float(&gyro_data[2]);
        g_calibration_data.sample_count++;
    }
    else
    {
        g_calibration_data.gyro_offset_x /= (float)GYRO_CALIBRATION_SAMPLES;
        g_calibration_data.gyro_offset_y /= (float)GYRO_CALIBRATION_SAMPLES;
        g_calibration_data.gyro_offset_z /= (float)GYRO_CALIBRATION_SAMPLES;

#ifdef CONFIG_IMU_LOG
        platform_log(
            "IMU", LOG_LEVEL_INF, "Gyro calibration complete: X = %f, Y = %f, Z = %f", g_calibration_data.gyro_offset_x,
            g_calibration_data.gyro_offset_y, g_calibration_data.gyro_offset_z);
#endif  // CONFIG_IMU_LOG

        calibration_finished = true;
    }
}
#endif  // CONFIG_IMU_CALIBRATE_GYRO

int
imu_init(void)
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

    set_dlpf();
#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "imu dlpf set");
#endif  // CONFIG_IMU_LOG

#ifdef CONFIG_IMU_LOG
    platform_log("IMU", LOG_LEVEL_INF, "imu init finished");
#endif  // CONFIG_IMU_LOG

    return 0;
}

#ifndef CONFIG_IMU_CALIBRATE_GYRO
#include <math.h>

static imu_data
get_data(struct sensor_value* accelerometer_data, struct sensor_value* gyro_data, struct sensor_value* temperature)
{
    ARG_UNUSED(temperature);

    static float angle_balance    = 0.0f;
    static float angle_rotation   = 0.0f;
    static int64_t last_time_ms   = 0;
    int64_t const current_time_ms = k_uptime_get();

    // Compute dynamic DT in seconds:
    float const dt = (float)(current_time_ms - last_time_ms) * MILLI_PARTS_CONVERT;

    float const ay = sensor_value_to_float(&accelerometer_data[1]);
    float const az = sensor_value_to_float(&accelerometer_data[2]);

    // [radians]
    float const accel_angle = atan2f(ay, az) - ACCELEROMETER_OFFSET;  // Wrap detection needed.
    float const gyro_rate_x = sensor_value_to_float(&gyro_data[0]) - GYRO_X_DRIFT;
    float const gyro_rate_y = sensor_value_to_float(&gyro_data[1]) - GYRO_Y_DRIFT;

#if defined(CONFIG_IMU_LOG) && !defined(IMU_CALIBRATE_GYRO)
    platform_log("IMU", LOG_LEVEL_INF, "gx: %f, gy: %f", (double)gyro_rate_x, (double)gyro_rate_y);
#endif  // CONFIG_IMU_LOG

    angle_balance = ALPHA * (angle_balance + gyro_rate_x * dt) + (1.0f - ALPHA) * accel_angle;
    angle_rotation += gyro_rate_y * dt;

    last_time_ms        = current_time_ms;
    imu_data const data = {
        .angle_balance = angle_balance, .angle_balance_dt = gyro_rate_x, .angle_rotation = angle_rotation};

    return data;
}
#endif  // not CONFIG_IMU_CALIBRATE_GYRO

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

    if(ret != 0)
    {
#ifdef CONFIG_IMU_LOG
        platform_log("IMU", LOG_LEVEL_ERR, "imu processing failed");
#endif  // CONFIG_IMU_LOG
        stop_control_loop();
        return -ENODEV;
    }

#ifdef CONFIG_IMU_CALIBRATE_GYRO
    calibrate_gyro(gyro_data);
#else
    s_imu_data = get_data(accelerometer_data, gyro_data, &temperature);
    trigger_control_loop();
#endif  // CONFIG_IMU_CALIBRATE_GYRO

    return ret;
}

imu_data
_get_imu_data(void)
{
    return s_imu_data;
}