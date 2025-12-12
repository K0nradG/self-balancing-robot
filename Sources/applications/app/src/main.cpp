#include "logger.h"

static Logger<IS_ENABLED(CONFIG_APP_LOG)> app_logger("APP");

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#define BLINKING_INTERVAL 500
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_ROBOT_CONTROL
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "control_loop.h"
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
#endif  // CONFIG_ROBOT_CONTROL

#include <zephyr/kernel.h>

/*TODO: ble dfu is mandatory - needs to be default y*/
#include "dfu_ble.h"

K_SEM_DEFINE(start_app_sem, 0, 1);

static void
dfu_action()
{
    k_sem_give(&start_app_sem);
}

int
main()
{
    dfu_action_cb_register(dfu_action);

    // wait for dfu action from dfu module before starting main app
    k_sem_take(&start_app_sem, K_FOREVER);

    app_logger.platform_log(LOG_LEVEL::INF, "Application started.");
#ifdef CONFIG_INTERFACE_DRV
    led_start_periodic_blinking(BLINKING_INTERVAL);
    app_logger.platform_log(LOG_LEVEL::INF, "Interface driver is enabled.");
#endif  // CONFIG_INTERFACE_DRV

#ifdef CONFIG_ROBOT_CONTROL
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    Robot_Control::new_send_identification_data_cb_register(Robot_Control::new_regulator_data_for_identification);
    app_logger.platform_log(LOG_LEVEL::INF, "Model identification driver is enabled.");
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
#endif  // CONFIG_ROBOT_CONTROL
}
