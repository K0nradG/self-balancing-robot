#include "interface.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "logger.h"
#include "utils.h"

static Logging::Logger<IS_ENABLED(CONFIG_INTERFACE_LOG)> interface_logger("INTERFACE");

static const gpio_dt_spec led_dev = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static uint16_t g_blinking_interval     = 0u;
static bool g_periodic_blinking_started = false;

static void
led_toggle_work_handler(k_work* work);

static K_WORK_DELAYABLE_DEFINE(led_toggle_work, led_toggle_work_handler);

static void
led_toggle_work_handler(k_work* work)
{
    int const ret = gpio_pin_toggle_dt(&led_dev);

    if(ret != 0)
    {
        interface_logger.platform_log(Logging::LOG_LEVEL::ERR, "LED toggle failed, err: %d", ret);
        return;
    }
    reschedule_work(&led_toggle_work, K_MSEC(g_blinking_interval), "led blink");
}

int
interface_init()
{
    if(!device_is_ready(led_dev.port))
    {
        interface_logger.platform_log(Logging::LOG_LEVEL::ERR, "led not ready");
        return -ENODEV;
    }

    int const ret = gpio_pin_configure_dt(&led_dev, GPIO_OUTPUT_ACTIVE);

    if(ret != 0)
    {
        interface_logger.platform_log(Logging::LOG_LEVEL::ERR, "led pins not ready");
        return ret;
    }

    interface_logger.platform_log(Logging::LOG_LEVEL::INF, "led init finished");
    return ret;
}

void
led_start_periodic_blinking(uint16_t blinking_interval)
{
    if(g_periodic_blinking_started)
    {
        interface_logger.platform_log(Logging::LOG_LEVEL::ERR, "led worker already started");
    }

    g_periodic_blinking_started = true;

    g_blinking_interval = blinking_interval;
    reschedule_work(&led_toggle_work, K_NO_WAIT, "led blink");
}

void
led_stop_periodic_blinking()
{
    if(!g_periodic_blinking_started)
    {
        interface_logger.platform_log(Logging::LOG_LEVEL::ERR, "led worker not started");
    }

    g_periodic_blinking_started = false;

    int const ret = k_work_cancel_delayable(&led_toggle_work);
    if(ret != 0)
    {
        interface_logger.platform_log(Logging::LOG_LEVEL::ERR, "cancel led work err:%d", ret);
        return;
    }
    interface_logger.platform_log(Logging::LOG_LEVEL::DBG, "led blink work cancelled");
}