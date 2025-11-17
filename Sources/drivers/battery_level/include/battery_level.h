#pragma once

#include <stdint.h>

struct battery_level_data
{
    uint8_t battery_level_percent;
    uint16_t battery_level_mv;
};

/* This means that battery_level_updated_cb_t is a pointer to a function that takes the battery_level_data structure as
 * an argument and returns nothing (void).*/
typedef void (*battery_level_updated_cb_t)(battery_level_data battery_lvl_data);

int
battery_level_init();

void
new_battery_level_cb_register(battery_level_updated_cb_t _new_battery_level_cb);

void
battery_start_periodic_measurement(uint16_t interval_ms);

void
battery_stop_periodic_measurement();