#include "watchdog_controller.h"
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include "logger.h"

static Logging::Logger<IS_ENABLED(CONFIG_WATCHDOG_CONTROLLER_DRV_LOG)> watchdog_controller_logger("WDG");

device const* watchdog = DEVICE_DT_GET(DT_ALIAS(watchdog0));
int channel_id         = 0;

int
watchdog_controller_init()
{
    if(!device_is_ready(watchdog))
    {
        watchdog_controller_logger.platform_log(Logging::LOG_LEVEL::ERR, "Watchdog device not ready");
        return -ENODEV;
    }

    static wdt_timeout_cfg const watchdog_cfg = {
        .window = {.min = 0U /* feed allowed immediately */, .max = CONFIG_WATCHDOG_MAX_TIMEOUT_MS /* timeout in ms */},
        .callback = nullptr,            /* nullptr = system reset */
        .flags    = WDT_FLAG_RESET_SOC, /* reset the system on timeout */
    };

    channel_id = wdt_install_timeout(watchdog, &watchdog_cfg);
    if(channel_id < 0)
    {
        watchdog_controller_logger.platform_log(Logging::LOG_LEVEL::ERR, "Failed to install watchdog timeout");
        return -1;
    }

    int const err = wdt_setup(watchdog, 0);
    return err;
}

void
feed_watchdog()
{
    if(wdt_feed(watchdog, channel_id) < 0)
    {
        watchdog_controller_logger.platform_log(Logging::LOG_LEVEL::ERR, "Failed to feed the watchdog");
    }
}