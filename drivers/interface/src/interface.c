#include "interface.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "utils.h"

#ifdef CONFIG_INTERFACE_LOG
#include "logger.h"
#else
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif  // CONFIG_INTERFACE_LOG

static const struct gpio_dt_spec led_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static struct k_work_delayable led_toggle_work;

static uint16_t g_blinking_interval     = 0u;
static bool g_periodic_blinking_started = false;

static void
led_toggle_work_handler(struct k_work* work)
{
    gpio_pin_toggle_dt(&led_dev);
    reschedule_work(&led_toggle_work, K_MSEC(g_blinking_interval), "led blink");
}

static K_WORK_DELAYABLE_DEFINE(led_toggle_work, led_toggle_work_handler);

int
interface_init(void)
{
    if(!device_is_ready(led_dev.port))
    {
#ifdef CONFIG_INTERFACE_LOG
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led not ready");
#endif  // CONFIG_INTERFACE_LOG
        return -ENODEV;
    }

    int ret = 0;

    ret = gpio_pin_configure_dt(&led_dev, GPIO_OUTPUT_ACTIVE);

#ifdef CONFIG_INTERFACE_LOG
    if(ret)
    {
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led pins not ready");
        return ret;
    }
    platform_log("INTERFACE", LOG_LEVEL_INF, "led init finished");
#endif  // CONFIG_INTERFACE_LOG
    return ret;
}

void
led_start_periodic_blinking(uint16_t blinking_interval)
{
#ifdef CONFIG_INTERFACE_LOG
    if(g_periodic_blinking_started)
    {
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led worker already started");
    }
#endif  // CONFIG_INTERFACE_LOG

    g_periodic_blinking_started = true;

    g_blinking_interval = blinking_interval;
    reschedule_work(&led_toggle_work, K_NO_WAIT, "led blink");
}

void
led_stop_periodic_blinking(void)
{
#ifdef CONFIG_INTERFACE_LOG
    if(!g_periodic_blinking_started)
    {
        platform_log("INTERFACE", LOG_LEVEL_ERR, "led worker not started");
    }
#endif  // CONFIG_INTERFACE_LOG

    g_periodic_blinking_started = false;

    int const ret = k_work_cancel_delayable(&led_toggle_work);

#ifdef CONFIG_INTERFACE_LOG
    if(ret)
    {
        platform_log("INTERFACE", LOG_LEVEL_ERR, "cancel led work err:%d", ret);
        return;
    }
    platform_log("INTERFACE", LOG_LEVEL_DBG, "led blink work cancelled");
#endif  // CONFIG_INTERFACE_LOG
}
