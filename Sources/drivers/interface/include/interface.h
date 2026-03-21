// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdint.h>

int
interface_init();

void
led_start_periodic_blinking(uint16_t blinking_interval);

void
led_stop_periodic_blinking();