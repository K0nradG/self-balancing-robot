#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include "battery_level.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include "utils.h"

/* battery level driver init priority - lower values means earlier in initialization
main initialization has 100, so our driver is initialize right before main*/
#define APPLICATION_INIT_PRIORITY 99

/* Below values in [Ω]*/
#define VOLTAGE_DIVIDER_RESISTOR_UP 100000
#define VOLTAGE_DIVIDER_RESISTOR_DOWN 200000

LOG_MODULE_REGISTER(battery_level, CONFIG_BAT_LVL_LOG_LEVEL);

static const struct adc_dt_spec adc_dev = ADC_DT_SPEC_GET(DT_NODELABEL(adc));

battery_level_updated_cb_t new_battery_level_cb;

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

    const bool is_adc_ready = adc_is_ready_dt(&adc_dev);

    __ASSERT(is_adc_ready, "ADC controller device %s not ready", adc_dev.dev->name);

    ret = adc_channel_setup_dt(&adc_dev);
    __ASSERT(!ret, "Channel=%d setup failed with error: %d", adc_dev.channel_id, ret);

    ret = adc_sequence_init_dt(&adc_dev, &sequence);
    __ASSERT(!ret, "Sequence initialization failed with error: %d", ret);

    __ASSERT(sequence.buffer, "Uninitialized buffer");
    __ASSERT(sequence.buffer_size, "Buffer size equal to zero");

    return ret;
}

SYS_INIT(init, APPLICATION, APPLICATION_INIT_PRIORITY);

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

    if(previous_charge_level < charge_level)
    {
        charge_level = previous_charge_level;
    }
    previous_charge_level = charge_level;

    return charge_level;
}

static int
get_sample(void)
{
    int ret = adc_sequence_init_dt(&adc_dev, &sequence);
    if(ret < 0)
    {
        LOG_ERR("Can't init sequence");
        return ret;
    }

    ret = adc_read(adc_dev.dev, &sequence);
    if(ret < 0)
    {
        LOG_ERR("Can't read ADC sample for battery level");
        return ret;
    }

    int32_t battery_level_mv = 0;

    ret = adc_raw_to_millivolts_dt(&adc_dev, &battery_level_mv);
    if(ret < 0)
    {
        LOG_ERR("Can't convert battery level to mv - conversion non supported");
        return ret;
    }
    __ASSERT(battery_level_mv <= INT16_MAX, "Battery level out of range!");

    const int64_t corrected_battery_level_mv = (int64_t)battery_level_mv *
                                               (VOLTAGE_DIVIDER_RESISTOR_UP + VOLTAGE_DIVIDER_RESISTOR_DOWN) /
                                               VOLTAGE_DIVIDER_RESISTOR_DOWN;

    struct battery_level_data battery_level;

    battery_level.battery_level_percent = battery_charge_level((int16_t)corrected_battery_level_mv);
    battery_level.battery_level_mv      = corrected_battery_level_mv;

    /* give the battery level to the user by registered callback*/
    if(new_battery_level_cb)
    {
        new_battery_level_cb(battery_level);
    }

    return 0;
}

static void
battery_measurement_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    int ret = get_sample();
    if(ret)
    {
        LOG_ERR("Can't get battery level: %d", ret);
    }
    /* Batter measurement work is calls itself every measurement_interval ms*/
    reschedule_work(&battery_measurement_work, K_MSEC(measurement_interval), "Battery level measurement");
}

static K_WORK_DELAYABLE_DEFINE(battery_measurement_work, battery_measurement_work_handler);

void
battery_start_periodic_measurement(uint16_t interval_ms)
{
    __ASSERT(periodic_measurement_started, "Periodic measurement already started");
    periodic_measurement_started = true;

    measurement_interval = interval_ms;
    reschedule_work(&battery_measurement_work, K_NO_WAIT, "battery_level_measurement");
}

void
battery_stop_periodic_measurement(void)
{
    __ASSERT(!periodic_measurement_started, "Periodic measurement is not started");
    periodic_measurement_started = false;

    const int ret = k_work_cancel_delayable(&battery_measurement_work);
    if(ret)
    {
        LOG_ERR("Can't cancel delayable work: %d", ret);
        return;
    }

    LOG_DBG("Battery level measurement work cancelled");
}

void
new_battery_level_cb_register(battery_level_updated_cb_t _new_battery_level_cb)
{
    if(_new_battery_level_cb)
    {
        new_battery_level_cb = _new_battery_level_cb;
    }
}
