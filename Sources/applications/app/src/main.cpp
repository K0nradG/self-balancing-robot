#include "logger.h"

static Logger<IS_ENABLED(CONFIG_APP_LOG)> app_logger("APP");

#ifdef CONFIG_INTERFACE_DRV
#include "interface.h"
#define BLINKING_INTERVAL 500
#endif  // CONFIG_INTERFACE_DRV

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

    // Wait for dfu action from dfu module before starting main app
    k_sem_take(&start_app_sem, K_FOREVER);
    app_logger.platform_log(LOG_LEVEL::INF, "Application started.");

#ifdef CONFIG_INTERFACE_DRV
    led_start_periodic_blinking(BLINKING_INTERVAL);
    app_logger.platform_log(LOG_LEVEL::INF, "Interface driver is enabled.");
#endif  // CONFIG_INTERFACE_DRV
}
