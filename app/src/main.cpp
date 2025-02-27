#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_LOGGER_DRV
#include "logger.h"
#endif  // CONFIG_LOGGER_DRV

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#define BLINKING_INTERVAL 500
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_REGULATOR_DRV
#include "regulator.h"
#endif //CONFIG_REGULATOR_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
#include "battery_level.h"
#define MEASUREMENT_INTERVAL 9000

static void
new_battery_level_callback(battery_level_data data)
{
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "bat lvl %u", data.battery_level_percent);
    platform_log("APP", LOG_LEVEL_INF, "bat lvl mv %u", data.battery_level_mv);
#endif  // CONFIG_APP_LOG
}
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_REGULATOR_DRV
void new_pid_regulator_parameters(pid_regulator_parameters data) {
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Kp: %f, Ki: %f, Kd: %f, Setpoint: %f", data.k, data.i, data.d, data.setpoint);
#endif  // CONFIG_APP_LOG
}

#endif //CONFIG_REGULATOR_DRV

int
main(void)
{
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Application started.");
#endif  // CONFIG_APP_LOG

#ifdef CONFIG_INTERFACE_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "interface driver is enabled.");
#endif  // CONFIG_APP_LOG
    led_start_periodic_blinking(BLINKING_INTERVAL);
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_BATTERY_LEVEL_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Battery level driver is enabled.");
#endif  // CONFIG_APP_LOG
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#endif  // CONFIG_BATTERY_LEVEL_DRV

#ifdef CONFIG_REGULATOR_DRV
#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Regulator driver is enabled.");
#endif //CONFIG_APP_LOG
    regulator_start_automatic_control();
    new_pid_regulator_parameters_cb_register(new_pid_regulator_parameters);
#endif //CONFIG_REGULATOR_DRV


}
