// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "proximity_sensor.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include "logger.h"

static Logger<IS_ENABLED(CONFIG_VL53L0X_LOG)> proximity_sensor_logger("PROXIMITY_SENSOR");

static const struct device* const proximity_sensor_dev = DEVICE_DT_GET_ONE(st_vl53l0x);

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
get_proximity_m()
{
    if(!device_is_ready(proximity_sensor_dev))
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "VL53L0X device not ready");
        return -1.0f;
    }

    int ret = sensor_sample_fetch(proximity_sensor_dev);
    if(ret != 0)
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "Failed to fetch sample, err: %d", ret);
        return -1.0f;
    }

    struct sensor_value val{};
    ret = sensor_channel_get(proximity_sensor_dev, SENSOR_CHAN_DISTANCE, &val);
    if(ret != 0)
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "Failed to get distance channel, err: %d", ret);
        return -1.0f;
    }

    return (float)val.val1 + ((float)val.val2 / 1000000.0f);
}

bool
is_proximity_safe()
{
    if(!device_is_ready(proximity_sensor_dev))
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "VL53L0X device not ready");
        return false;
    }

    int ret = sensor_sample_fetch(proximity_sensor_dev);
    if(ret != 0)
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "Failed to fetch sample, err: %d", ret);
        return false;
    }

    struct sensor_value val{};
    ret = sensor_channel_get(proximity_sensor_dev, SENSOR_CHAN_PROX, &val);
    if(ret != 0)
    {
        proximity_sensor_logger.platform_log(LOG_LEVEL::ERR, "Failed to get proximity channel, err: %d", ret);
        return false;
    }

    return (val.val1 == 0);
}