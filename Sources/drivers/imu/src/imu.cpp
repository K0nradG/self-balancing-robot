#include "imu.h"
#include <zephyr/init.h>
#include "control_loop.h"
#include "imu_config.h"
#include "logger.h"

#ifdef CONFIG_IMU_CALIBRATE_GYRO
#define GYRO_CALIBRATION_SAMPLES 100000
#endif  // CONFIG_IMU_CALIBRATE_GYRO

#define ALPHA               0.997f
#define PI                  3.14159265358979323846f
#define MILLI_PARTS_CONVERT 1e-03f

#define ANGLE_OFFSET         -90.0f
#define DEG_TO_RAD           (PI / 180.0f)
#define ACCELEROMETER_OFFSET (ANGLE_OFFSET * DEG_TO_RAD)

// "Balancing" position measurement:

#define GYRO_X_DRIFT -0.087331f
#define GYRO_Y_DRIFT 0.012047f
#define GYRO_Z_DRIFT 0.005287f

#ifdef CONFIG_IMU_CALIBRATE_GYRO
struct gyro_calibration_data
{
    int sample_count;
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
};

static gyro_calibration_data g_calibration_data {};
#endif  // CONFIG_IMU_CALIBRATE_GYRO

static Logger<IS_ENABLED(CONFIG_IMU_LOG)> imu_logger("IMU");

device const* imu_dev = DEVICE_DT_GET_ONE(invensense_mpu6050);
imu_data s_imu_data {};
static bool s_reset_balance_angle = false;

static int
process_imu(device const* dev);

#ifdef CONFIG_MPU6050_TRIGGER

static sensor_trigger trigger {};

// Interrupt handler:
static void
handle_imu_drdy(device const* dev, sensor_trigger const* trig)
{
    int const ret = process_imu(dev);  // Read and process IMU data

    if(ret != 0)
    {
        imu_logger.platform_log(LOG_LEVEL::ERR, "imu not reading/processing data");
        (void)sensor_trigger_set(dev, trig, nullptr);
    }
}

#endif  // CONFIG_MPU6050_TRIGGER

#ifdef CONFIG_IMU_CALIBRATE_GYRO
void
calibrate_gyro(sensor_value* gyro_data)
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

        imu_logger.platform_log(
            LOG_LEVEL::INF, "Gyro calibration complete: X = %f, Y = %f, Z = %f",
            (double)g_calibration_data.gyro_offset_x, (double)g_calibration_data.gyro_offset_y,
            (double)g_calibration_data.gyro_offset_z);

        calibration_finished = true;
    }
}
#endif  // CONFIG_IMU_CALIBRATE_GYRO

int
imu_init()
{
    bool const is_imu_device_ready = device_is_ready(imu_dev);

    if(!is_imu_device_ready)
    {
        imu_logger.platform_log(LOG_LEVEL::ERR, "imu not ready");
        return -ENODEV;
    }

#ifdef CONFIG_MPU6050_TRIGGER
    trigger = (sensor_trigger) {
        .type = SENSOR_TRIG_DATA_READY,  // Trigger when data is ready
        .chan = SENSOR_CHAN_ALL,         // Apply to all channels
    };

    if(sensor_trigger_set(imu_dev, &trigger, handle_imu_drdy) < 0)
    {
        imu_logger.platform_log(LOG_LEVEL::ERR, "imu cannot configure trigger");
        return -ENODEV;
    }
#endif  // CONFIG_MPU6050_TRIGGER

    set_dlpf();
    imu_logger.platform_log(LOG_LEVEL::INF, "imu dlpf set");

    imu_logger.platform_log(LOG_LEVEL::INF, "imu init finished");
    return 0;
}

#ifndef CONFIG_IMU_CALIBRATE_GYRO
#include <math.h>

static float
unwrap_accelerometer_angle(float accel_angle)
{
    static float prev_angle = 0.0f;
    float const delta       = accel_angle - prev_angle;

    if(delta > PI)
    {
        accel_angle -= 2.0f * PI;
    }
    else if(delta < -PI)
    {
        accel_angle += 2.0f * PI;
    }

    prev_angle = accel_angle;
    return accel_angle;
}

static imu_data
get_data(sensor_value* accelerometer_data, sensor_value* gyro_data, sensor_value* temperature)
{
    ARG_UNUSED(temperature);

    static float angle_balance = 0.0f;
    if(s_reset_balance_angle)
    {
        angle_balance         = 0.0f;
        s_reset_balance_angle = false;
    }

    static int64_t last_time_ms   = 0;
    int64_t const current_time_ms = k_uptime_get();

    // Compute dynamic DT in seconds:
    float const dt = (float)(current_time_ms - last_time_ms) * MILLI_PARTS_CONVERT;

    float const ay = sensor_value_to_float(&accelerometer_data[1]);
    float const az = sensor_value_to_float(&accelerometer_data[2]);

    // [radians]
    float accel_angle = atan2f(ay, az);
    accel_angle       = unwrap_accelerometer_angle(accel_angle);
    accel_angle -= ACCELEROMETER_OFFSET;
    float const gyro_rate_x = sensor_value_to_float(&gyro_data[0]) - GYRO_X_DRIFT;
    float const gyro_rate_y = sensor_value_to_float(&gyro_data[1]) - GYRO_Y_DRIFT;

#ifndef CONFIG_IMU_CALIBRATE_GYRO
    imu_logger.platform_log(LOG_LEVEL::INF, "angle: %f", (double)(accel_angle * (180.0f / PI)));
    imu_logger.platform_log(LOG_LEVEL::INF, "gx: %f, gy: %f", (double)gyro_rate_x, (double)gyro_rate_y);
#endif  // not CONFIG_IMU_CALIBRATE_GYRO

    angle_balance = ALPHA * (angle_balance + gyro_rate_x * dt) + (1.0f - ALPHA) * accel_angle;

    last_time_ms        = current_time_ms;
    imu_data const data = {
        .angle_balance     = angle_balance,
        .angle_balance_dt  = gyro_rate_x,
        .angle_rotation_dt = gyro_rate_y,
        .time_dt           = dt};

    return data;
}
#endif  // not CONFIG_IMU_CALIBRATE_GYRO

static int
process_imu(device const* dev)
{
    static sensor_value accelerometer_data[3] {};
    static sensor_value gyro_data[3] {};
    static sensor_value temperature {};

    int ret = sensor_sample_fetch(dev);  // Fetch new data

    ret |= sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accelerometer_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);
    ret |= sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);

    if(ret != 0)
    {
        imu_logger.platform_log(LOG_LEVEL::ERR, "imu processing failed");
        Robot_Control::stop_control_loop();
        return -ENODEV;
    }

#ifdef CONFIG_IMU_CALIBRATE_GYRO
    calibrate_gyro(gyro_data);
#else
    s_imu_data = get_data(accelerometer_data, gyro_data, &temperature);
    Robot_Control::trigger_control_loop();
#endif  // CONFIG_IMU_CALIBRATE_GYRO

    return ret;
}

imu_data
_get_imu_data()
{
    return s_imu_data;
}

void
reset_imu_balance_angle()
{
    s_reset_balance_angle = true;
}