// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "proximity_sensor.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include "logger.h"

static Logger<IS_ENABLED(CONFIG_VL53L0X_LOG)> proximity_sensor_logger("PROXIMITY_SENSOR");

static const struct device* const proximity_sensor_dev = DEVICE_DT_GET_ONE(st_vl53l0x);

constexpr size_t PROXIMITY_THREAD_STACK_SIZE      = 2048u;
constexpr int PROXIMITY_THREAD_PRIORITY           = 4;
constexpr uint32_t PROXIMITY_THREAD_OPTIONS       = 0u;
constexpr int32_t PROXIMITY_THREAD_START_DELAY_MS = 0;

static float s_proximity_m = 1.0f;

static float
internal_get_proximity_m()
{
    int ret = sensor_sample_fetch(proximity_sensor_dev);
    if(ret != 0)
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "Failed to fetch sample, err: %d", ret);
        return -1.0f;
    }

    sensor_value val {};
    ret = sensor_channel_get(proximity_sensor_dev, SENSOR_CHAN_DISTANCE, &val);
    if(ret != 0)
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "Failed to get distance channel, err: %d", ret);
        return -1.0f;
    }

    static constexpr float conversion_factor = 1.0f / 1000000.0f;
    return (float)val.val1 + ((float)val.val2 * conversion_factor);
}

static void
thread_proximity_sensor(void* p1, void* p2, void* p3)
{
    while(true)
    {
        float const proximity_m = internal_get_proximity_m();
        s_proximity_m           = proximity_m;
    }
}

K_THREAD_DEFINE(
    proximity_tid, PROXIMITY_THREAD_STACK_SIZE, thread_proximity_sensor, nullptr, nullptr, nullptr,
    PROXIMITY_THREAD_PRIORITY, PROXIMITY_THREAD_OPTIONS, PROXIMITY_THREAD_START_DELAY_MS);

int
proximity_sensor_init()
{
    if(!device_is_ready(proximity_sensor_dev))
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "VL53L0X device not ready");
        return -ENODEV;
    }

    proximity_sensor_logger.platform_log(LOG_LEVEL::INF, "VL53L0X init finished");
    return 0;
}

float
_get_proximity_m()
{
    float const proximity_m = s_proximity_m;
    return proximity_m;
}