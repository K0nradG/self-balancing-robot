#pragma GCC diagnostic ignored "-Wdouble-promotion"

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

#include <zephyr/kernel.h>

/*TODO: ble dfu is mandatory needes to be default y*/
#include "dfu_ble.h"

K_SEM_DEFINE(start_app_sem, 0, 1);

static void
dfu_action()
{
    k_sem_give(&start_app_sem);
}

int
main(void)
{
    dfu_action_cb_register(dfu_action);

    // wait for dfu action from dfu module before starting main app
    k_sem_take(&start_app_sem, K_FOREVER);

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

#ifdef CONFIG_ROBOT_CONTROL
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    new_send_identification_data_cb_register(new_regulator_data_for_identification);

#ifdef CONFIG_APP_LOG
    platform_log("APP", LOG_LEVEL_INF, "Model identification driver is enabled.");
#endif  // CONFIG_APP_LOG

#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
#endif  // CONFIG_ROBOT_CONTROL
}
