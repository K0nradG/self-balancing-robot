// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdint.h>

namespace Robot_Control
{

struct Telemetry_Sample
{
    uint32_t timestamp_us;
    float balance_setpoint;
    float balance_angle;
    float rotation_setpoint;
    float rotation_angle;
    float target_speed_0;
    float target_speed_1;
    float measured_speed_0;
    float measured_speed_1;
    float pwm_0;
    float pwm_1;
};

// One 4-byte timestamp and ten 4-byte floats use 44 bytes in total.
static_assert(sizeof(Telemetry_Sample) == 44u);

void
telemetry_submit(Telemetry_Sample const& sample);

}  // namespace Robot_Control
