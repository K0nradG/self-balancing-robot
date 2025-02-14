#include <zephyr/kernel.h>

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#include "motor_controller.h"
#endif

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app, CONFIG_APP_LOG_LEVEL);

#ifdef CONFIG_BATTERY_LEVEL_DRV
#define MEASUREMENT_INTERVAL 500

static void
new_battery_level_callback(battery_level_data data)
{
    LOG_INF("Battery level at %u", data.battery_level_percent);
    LOG_INF("Battery level voltage %u", data.battery_level_mv);
}
#endif

int
main(void)
{
#ifdef CONFIG_BATTERY_LEVEL_DRV
    LOG_INF("Battery level driver is enabled.");
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#else
    LOG_INF("Battery level driver is not enabled.");
#endif

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
    LOG_INF("Motor controller driver is enabled.");
    set_enable_controller(true);
    set_start_motors(true);
    set_direction(POSITIVE);
    set_duty_cycle_value(50);
    motor_controller_start();
#else
    LOG_INF("Motor controller driver is not enabled.");
#endif
    LOG_INF("Application started.");
    return 0;
}
