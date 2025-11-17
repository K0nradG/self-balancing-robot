#include "battery_level.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/init.h>
#include "logger.h"
#include "utils.h"

#define INIT_ERR -1

#define VOLTAGE_DIVIDER_RESISTOR_UP   100000
#define VOLTAGE_DIVIDER_RESISTOR_DOWN 200000

static Logging::Logger<IS_ENABLED(CONFIG_BATTERY_LEVEL_LOG)> battery_logger("BATTERY_CONTROLLER");

static const adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

battery_level_updated_cb_t new_battery_level_cb = nullptr;

static int16_t adc_battery_buffer;

static adc_sequence sequence = {
    .buffer      = &adc_battery_buffer,
    .buffer_size = sizeof(adc_battery_buffer),
};

static uint16_t measurement_interval;
static bool periodic_measurement_started;

static void
battery_measurement_work_handler(k_work* work);

static K_WORK_DELAYABLE_DEFINE(battery_measurement_work, battery_measurement_work_handler);

int
battery_level_init()
{
    int ret = 0;

    if(!adc_is_ready_dt(&adc_channel))
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "ADC not ready");
        return INIT_ERR;
    }

    ret = adc_channel_setup_dt(&adc_channel);
    if(ret != 0)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "ADC channel setup failed: %d", ret);
        return ret;
    }

    ret = adc_sequence_init_dt(&adc_channel, &sequence);
    if(ret != 0)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "ADC sequence init failed: %d", ret);
        return ret;
    }

    if((sequence.buffer == nullptr) || (sequence.buffer_size == 0u))
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "Invalid ADC buffer");
        return INIT_ERR;
    }

    battery_logger.platform_log(Logging::LOG_LEVEL::INF, "ADC init finished");
    return ret;
}

static uint8_t
battery_charge_level(int16_t voltage_mv)
{
    uint8_t charge_level = 0u;
    int16_t slope        = 0u;
    int32_t intercept    = 0u;

    if(voltage_mv > CONFIG_MAX_BATTERY_LEVEL)
    {
        charge_level = 100;
    }
    else if(voltage_mv > 7900)
    {
        slope        = (100 - 80) / (CONFIG_MAX_BATTERY_LEVEL - 7900);
        intercept    = 100 - slope * CONFIG_MAX_BATTERY_LEVEL;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 7400)
    {
        slope        = (80 - 60) / (7900 - 7400);
        intercept    = 80 - slope * 7900;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 7000)
    {
        slope        = (60 - 30) / (7400 - 7000);
        intercept    = 60 - slope * 7400;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > 6600)
    {
        slope        = (30 - 10) / (7000 - 6600);
        intercept    = 30 - slope * 7000;
        charge_level = slope * voltage_mv + intercept;
    }
    else if(voltage_mv > CONFIG_MIN_BATTERY_LEVEL)
    {
        slope        = (10 - 0) / (6600 - CONFIG_MIN_BATTERY_LEVEL);
        intercept    = 10 - slope * 6600;
        charge_level = slope * voltage_mv + intercept;
    }
    else
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
get_sample()
{
    int ret = adc_sequence_init_dt(&adc_channel, &sequence);
    if(ret != 0)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "init sequence err: %d", ret);
        return ret;
    }

    ret = adc_read(adc_channel.dev, &sequence);
    if(ret != 0)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "adc read err: %d", ret);
        return ret;
    }

    int32_t battery_level_mv = adc_battery_buffer;

    ret = adc_raw_to_millivolts_dt(&adc_channel, &battery_level_mv);
    if(ret != 0)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "adc raw to mv err: %d", ret);
        return ret;
    }

    if(battery_level_mv > INT16_MAX)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "bat lvl range err");
        return -1;
    }

    int64_t const corrected_battery_level_mv = (int64_t)battery_level_mv *
                                               (VOLTAGE_DIVIDER_RESISTOR_UP + VOLTAGE_DIVIDER_RESISTOR_DOWN) /
                                               VOLTAGE_DIVIDER_RESISTOR_UP;

    static battery_level_data battery_level {};

    battery_level.battery_level_percent = battery_charge_level((int16_t)corrected_battery_level_mv);
    battery_level.battery_level_mv      = corrected_battery_level_mv;

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
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "get battery lvl err: %d", ret);
    }

    reschedule_work(&battery_measurement_work, K_MSEC(measurement_interval), "Battery level measurement");
}

void
battery_start_periodic_measurement(uint16_t interval_ms)
{
    if(periodic_measurement_started)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "battery worker already started");
    }
    periodic_measurement_started = true;

    measurement_interval = interval_ms;
    reschedule_work(&battery_measurement_work, K_NO_WAIT, "Battery level measurement");
}

void
battery_stop_periodic_measurement()
{
    if(!periodic_measurement_started)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "battery worker not started");
    }
    periodic_measurement_started = false;

    int const ret = k_work_cancel_delayable(&battery_measurement_work);
    if(ret != 0)
    {
        battery_logger.platform_log(Logging::LOG_LEVEL::ERR, "cancel battery work err:%d", ret);
        return;
    }

    battery_logger.platform_log(Logging::LOG_LEVEL::DBG, "Battery level measurement work cancelled");
}

void
new_battery_level_cb_register(battery_level_updated_cb_t _new_battery_level_cb)
{
    if(_new_battery_level_cb)
    {
        new_battery_level_cb = _new_battery_level_cb;
    }
}
