#include "interface.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "utils.h"

#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
#include "logger.h"
#endif
#endif

static const struct gpio_dt_spec led_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static struct k_work_delayable led_toggle_work;

static uint16_t blinking_interval;
static bool periodic_blinking_started;

static void
led_toggle_work_handler(struct k_work* work)
{
    gpio_pin_toggle_dt(&led_dev);
    k_work_schedule(&led_toggle_work, K_MSEC(500));
}

static K_WORK_DELAYABLE_DEFINE(led_toggle_work, led_toggle_work_handler);

static int
init(void)
{
    if(!device_is_ready(led_dev.port))
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led not ready");
#endif
#endif
        return -ENODEV;
    }

    int ret = 0;

    ret = gpio_pin_configure_dt(&led_dev, GPIO_OUTPUT_ACTIVE);

#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
    if(ret)
    {
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led pins not ready");
        return ret;
    }
    platform_log("INTERFACE", LOG_LEVEL_INF, "led init finished");
#endif
#endif
    return ret;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void
led_start_periodic_blinking(uint16_t interval_ms)
{
    if(periodic_blinking_started)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led worker already started");
#endif
#endif
    }
    periodic_blinking_started = true;

    blinking_interval = interval_ms;
    reschedule_work(&led_toggle_work, K_NO_WAIT, "battery_level_measurement");
}

void
led_stop_periodic_blinking(void)
{
    if(!periodic_blinking_started)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led worker not started");
#endif
#endif
    }
    periodic_blinking_started = false;

    const int ret = k_work_cancel_delayable(&led_toggle_work);
    if(ret)
    {
#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
        platform_log("INTERFACE", LOG_LEVEL_ERR, "cancel led work err:%d", ret);
#endif
#endif
        return;
    }

#ifdef CONFIG_LOGGER_DRV
#ifdef CONFIG_INTERFACE_LOG
    platform_log("INTERFACE", LOG_LEVEL_DBG, "led blink work cancelled");
#endif
#endif
}
