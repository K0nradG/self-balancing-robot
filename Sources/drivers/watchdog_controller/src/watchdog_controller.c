#include "watchdog_controller.h"
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV_LOG
#include "logger.h"
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV_LOG

struct device const* watchdog = DEVICE_DT_GET(DT_ALIAS(watchdog0));
int channel_id                = 0;

int
watchdog_controller_init(void)
{
    if(!device_is_ready(watchdog))
    {
#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV_LOG
        platform_log("WDG", LOG_LEVEL_ERR, "Watchdog device not ready");
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV_LOG

        return -ENODEV;
    }

    static struct wdt_timeout_cfg const watchdog_cfg = {
        .window.min = 0U,                             /* feed allowed immediately */
        .window.max = CONFIG_WATCHDOG_MAX_TIMEOUT_MS, /* timeout in ms */
        .callback   = NULL,                           /* NULL = system reset */
        .flags      = WDT_FLAG_RESET_SOC,             /* reset the system on timeout */
    };

    channel_id = wdt_install_timeout(watchdog, &watchdog_cfg);
    if(channel_id < 0)
    {
#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV_LOG
        platform_log("WDG", LOG_LEVEL_ERR, "Failed to install watchdog timeout");
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV_LOG
        return -1;
    }

    int const err = wdt_setup(watchdog, 0);
    return err;
}

void
feed_watchdog(void)
{
    if(wdt_feed(watchdog, channel_id) < 0)
    {
#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV_LOG
        platform_log("WDG", LOG_LEVEL_ERR, "Failed to feed the watchdog");
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV_LOG
    }
}