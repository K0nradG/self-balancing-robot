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

int
main()
{
#ifdef CONFIG_BATTERY_LEVEL_DRV
    new_battery_level_cb_register(new_battery_level_callback);
    battery_start_periodic_measurement(MEASUREMENT_INTERVAL);
#endif
    LOG_INF("dummy app");
}
