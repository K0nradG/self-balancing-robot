// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdbool.h>
#include <stdint.h>

struct MOTORS_DATA
{
    int8_t duty_cycle_percent_motor0;
    int8_t duty_cycle_percent_motor1;
    bool start;
};

int
motor_controller_init();

void
set_enable_controller(bool enable);

void
set_start_motors(bool start);

void
set_duty_cycle_value(int8_t duty_cycle_percent_motor0, int8_t duty_cycle_percent_motor1);

void
trigger_motors_update();

void
stop_motors();