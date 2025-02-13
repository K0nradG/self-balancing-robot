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

#ifdef CONFIG_MOTOR_CONTROLLER_DRV
#include "motor_controller.h"
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
    motor_controller_start();
    set_enable_controller(true);
    set_start_motors(true);
    set_direction(POSITIVE);
    set_duty_cycle_value(0.5f);
#else
    LOG_INF("Motor controller driver is not enabled.");
#endif
    LOG_INF("Application started.");
}
