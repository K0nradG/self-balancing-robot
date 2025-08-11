#pragma GCC diagnostic ignored "-Wunused-variable"

#include "battery_level.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/init.h>
#include "utils.h"

#ifdef CONFIG_BATTERY_LEVEL_LOG
#include "logger.h"
#endif  // CONFIG_BATTERY_LEVEL_LOG

#define INIT_ERR -1

/* Below values in [Ω]*/
#define VOLTAGE_DIVIDER_RESISTOR_UP   100000
#define VOLTAGE_DIVIDER_RESISTOR_DOWN 200000

static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

battery_level_updated_cb_t new_battery_level_cb = NULL;

static int16_t adc_battery_buffer;

static struct adc_sequence sequence = {
    .buffer      = &adc_battery_buffer,
    .buffer_size = sizeof(adc_battery_buffer),
};

static struct k_work_delayable battery_measurement_work;

static uint16_t measurement_interval;
static bool periodic_measurement_started;

static int
init(void)
{
    int ret = 0;

    if(!adc_is_ready_dt(&adc_channel))
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "ADC not ready");
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return INIT_ERR;
    }

    ret = adc_channel_setup_dt(&adc_channel);
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "ADC channel setup failed: %d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return ret;
    }

    ret = adc_sequence_init_dt(&adc_channel, &sequence);
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "ADC sequence init failed: %d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return ret;
    }

    if((sequence.buffer == NULL) || (sequence.buffer_size == 0u))
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "Invalid ADC buffer");
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return INIT_ERR;
    }

#ifdef CONFIG_BATTERY_LEVEL_LOG
    platform_log("BATTERY_CONTROLLER", LOG_LEVEL_INF, "ADC init finished");
#endif  // CONFIG_BATTERY_LEVEL_LOG

    return ret;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static uint8_t
battery_charge_level(int16_t voltage_mv)
{
    static uint8_t previous_charge_level = 100;
    uint8_t charge_level                 = 0;
    int16_t slope                        = 0;
    int32_t intercept                    = 0;

    if(voltage_mv > CONFIG_MAX_BATTERY_LEVEL)  // 100%
    {
        charge_level = 100;
    }
    else if(voltage_mv > 7900)  // 100% - 80%
    {
        slope        = (100 - 80) / (CONFIG_MAX_BATTERY_LEVEL - 7900);
        intercept    = 100 - slope * CONFIG_MAX_BATTERY_LEVEL;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 7400)  // 80% - 60%
    {
        slope        = (80 - 60) / (7900 - 7400);
        intercept    = 80 - slope * 7900;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 7000)  // 60% - 30%
    {
        slope        = (60 - 30) / (7400 - 7000);
        intercept    = 60 - slope * 7400;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 6600)  // 30% - 10%
    {
        slope        = (30 - 10) / (7000 - 6600);
        intercept    = 30 - slope * 7000;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > CONFIG_MIN_BATTERY_LEVEL)  // 10% - 0%
    {
        slope        = (10 - 0) / (6600 - CONFIG_MIN_BATTERY_LEVEL);
        intercept    = 10 - slope * 6600;
        charge_level = slope * voltage_mv + intercept;
    }
    else  // 0%
    {
        charge_level = 0;
    }

    if(charge_level > 100)
    {
        charge_level = 100;
    }
    else if(charge_level < 0)
    {
        charge_level = 0;
    }

    return charge_level;
}

static int
get_sample(void)
{
    int ret = adc_sequence_init_dt(&adc_channel, &sequence);
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "init sequence err: %d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return ret;
    }

    ret = adc_read(adc_channel.dev, &sequence);
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "adc read err: %d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return ret;
    }

    int32_t battery_level_mv = adc_battery_buffer;

    ret = adc_raw_to_millivolts_dt(&adc_channel, &battery_level_mv);
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "adc raw to mv err: %d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return ret;
    }

    if(battery_level_mv > INT16_MAX)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "bat lvl range err");
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return -1;
    }
    int64_t const corrected_battery_level_mv = (int64_t)battery_level_mv *
                                               (VOLTAGE_DIVIDER_RESISTOR_UP + VOLTAGE_DIVIDER_RESISTOR_DOWN) /
                                               VOLTAGE_DIVIDER_RESISTOR_UP;

    static struct battery_level_data battery_level = {0};

    battery_level.battery_level_percent = battery_charge_level((int16_t)corrected_battery_level_mv);
    battery_level.battery_level_mv      = corrected_battery_level_mv;

    /* give the battery level to the user by registered callback*/
    if(new_battery_level_cb)
    {
        new_battery_level_cb(battery_level);
    }

    return ret;
}

static void
battery_measurement_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    int ret = get_sample();
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "get battery lvl err: %d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
    }
    /* Batter measurement work is calls itself every measurement_interval ms*/
    reschedule_work(&battery_measurement_work, K_MSEC(measurement_interval), "Battery level measurement");
}

static K_WORK_DELAYABLE_DEFINE(battery_measurement_work, battery_measurement_work_handler);

void
battery_start_periodic_measurement(uint16_t interval_ms)
{
    if(periodic_measurement_started)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "battery worker already started");
#endif  // CONFIG_BATTERY_LEVEL_LOG
    }
    periodic_measurement_started = true;

    measurement_interval = interval_ms;
    reschedule_work(&battery_measurement_work, K_NO_WAIT, "battery_level_measurement");
}

void
battery_stop_periodic_measurement(void)
{
    if(!periodic_measurement_started)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "battery worker not started");
#endif  // CONFIG_BATTERY_LEVEL_LOG
    }
    periodic_measurement_started = false;

    int const ret = k_work_cancel_delayable(&battery_measurement_work);
    if(ret != 0)
    {
#ifdef CONFIG_BATTERY_LEVEL_LOG
        platform_log("BATTERY_CONTROLLER", LOG_LEVEL_ERR, "cancel battery work err:%d", ret);
#endif  // CONFIG_BATTERY_LEVEL_LOG
        return;
    }

#ifdef CONFIG_BATTERY_LEVEL_LOG
    platform_log("BATTERY CONTROLLER", LOG_LEVEL_DBG, "Battery level measurement work cancelled");
#endif  // CONFIG_BATTERY_LEVEL_LOG
}

void
new_battery_level_cb_register(battery_level_updated_cb_t _new_battery_level_cb)
{
    if(_new_battery_level_cb)
    {
        new_battery_level_cb = _new_battery_level_cb;
    }
}
