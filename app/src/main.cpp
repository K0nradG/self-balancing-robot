#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_APP_LOG_LEVEL);

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"

#define MEASUREMENT_INTERVAL 10000

static void
new_battery_level_callback(battery_level_data data)
{
    LOG_INF("Battery level at %u", data.battery_level_percent);
    LOG_INF("Battery level voltage %u", data.battery_level_mv);
}
#endif

#ifdef CONFIG_IMU_DRV
#include "imu.h"

static void
new_imu_callback(void)
{
    struct sensor_value const* const gyro_data          = get_gyro_data();
    struct sensor_value const* const accelerometer_data = get_accelerometer_data();
    struct sensor_value const        temperature        = get_temperature();

    if(gyro_data)
    {
        LOG_INF("Gyro X: %d.%06d", gyro_data[0].val1, gyro_data[0].val2);
    }
    else
    {
        LOG_ERR("Gyro data not valid!");
        return;
    }

    if(accelerometer_data)
    {
        LOG_INF("Accelerometer X: %d.%06d", accelerometer_data[0].val1, accelerometer_data[0].val2);
    }
    else
    {
        LOG_ERR("Accelerometer data not valid!");
        return;
    }

    LOG_INF("Temperature: %d.%06d", temperature.val1, temperature.val2);
}
#endif

int
main()
{
#ifdef CONFIG_BATTERY_LEVEL_DRV
    LOG_INF("Battery level driver is enabled.");
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#else
    LOG_INF("Battery level driver is not enabled.");
#endif

#ifdef CONFIG_IMU_DRV
    LOG_INF("IMU driver is enabled.");
    new_imu_cb_register(new_imu_callback);
#else
    LOG_INF("IMU driver is not enabled.");
#endif
}
