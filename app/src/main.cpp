#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_LOGGER_DRV
#include "logger.h"
#endif

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#include "motor_controller.h"
#endif

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif

#ifdef CONFIG_INTERFACE_DRV

#include "interface.h"
#define BLINKING_INTERVAL 500

#endif

#ifdef CONFIG_BATTERY_LEVEL_DRV
#define MEASUREMENT_INTERVAL 500

static void
new_battery_level_callback(battery_level_data data)
{
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "bat lvl %u", data.battery_level_percent);
    platform_log("APP", LOG_LEVEL_INF, "bat lvl mv %u", data.battery_level_mv);
#endif
#endif
}
#endif

#ifdef CONFIG_IMU_DRV
#include "imu.h"

static void
new_imu_callback(void)
{
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    struct sensor_value const* const gyro_data          = get_gyro_data();
    struct sensor_value const* const accelerometer_data = get_accelerometer_data();
    struct sensor_value const temperature               = get_temperature();

    if(gyro_data)
    {
        platform_log("APP", LOG_LEVEL_INF, "Gyro X: %d.%06d", gyro_data[0].val1, gyro_data[0].val2);
    }
    else
    {
        platform_log("APP", LOG_LEVEL_ERR, "Gyro data not valid!");
        return;
    }

    if(accelerometer_data)
    {
        platform_log(
            "APP", LOG_LEVEL_INF, "Accelerometer X: %d.%06d", accelerometer_data[0].val1, accelerometer_data[0].val2);
    }
    else
    {
        platform_log("APP", LOG_LEVEL_ERR, "Accelerometer data not valid!");
        return;
    }

    platform_log("APP", LOG_LEVEL_INF, "Temperature: %d.%06d", temperature.val1, temperature.val2);
#endif
#endif
}
#endif

int
main(void)
{
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Application started.");
#endif
#endif

#ifdef CONFIG_INTERFACE_DRV
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "interface driver is enabled.");
#endif
#endif
    led_start_periodic_blinking(BLINKING_INTERVAL);
#endif

#ifdef CONFIG_BATTERY_LEVEL_DRV
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Battery level driver is enabled.");
#endif
#endif
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#else
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Battery level driver is not enabled.");
#endif
#endif
#endif

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Motor controller  driver is enabled.");
#endif
#endif

    set_enable_controller(true);
    set_start_motors(true);
    set_direction(POSITIVE);
    motor_controller_start();

    for(int pwm = 0; pwm <= 100; pwm++)
    {
        set_duty_cycle_value(pwm);
        k_usleep(50000);
    }

    for(int pwm = 100; pwm >= 0; pwm--)
    {
        set_duty_cycle_value(pwm);
        k_usleep(50000);
    }
    set_direction(NEGATIVE);

    for(int pwm = 0; pwm <= 100; pwm++)
    {
        set_duty_cycle_value(pwm);
        k_usleep(50000);
    }

    for(int pwm = 100; pwm >= 0; pwm--)
    {
        set_duty_cycle_value(pwm);
        k_usleep(50000);
    }
#else
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Motor controller driver is not enabled.");
#endif
#endif
#endif

#ifdef CONFIG_IMU_DRV
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "IMU driver is enabled.");
    new_imu_cb_register(new_imu_callback);
#endif
#endif
#endif
}
