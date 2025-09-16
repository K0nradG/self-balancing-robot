#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include "shell.h"

#ifdef CONFIG_APP_LOG
#include "logger.h"
#endif  // CONFIG_APP_LOG

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#define BLINKING_INTERVAL 500
#endif  // CONFIG_INTERFACE_DRV

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

#ifdef CONFIG_ROBOT_CONTROL
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "control_loop.h"
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
#endif  // CONFIG_ROBOT_CONTROL

int
main(void)
{

    //register_shell_commands();

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

#ifdef CONFIG_ROBOT_CONTROL
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    new_send_identification_data_cb_register(new_regulator_data_for_identification);

#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Model identification driver is enabled.");
#endif  // CONFIG_APP_LOG

#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
#endif  // CONFIG_ROBOT_CONTROL

#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Application started.");
#endif  // CONFIG_APP_LOG

}
