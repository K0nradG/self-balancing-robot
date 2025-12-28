#pragma once

#include <stdint.h>

constexpr uint8_t MAX_INPUT_DATA_SAMPLES = 10u;

struct Input_Data
{
    float pwm_values[MAX_INPUT_DATA_SAMPLES];
    float pwm_durations_s[MAX_INPUT_DATA_SAMPLES];
};

struct PWM_Sample
{
    float pwm0;
    float pwm1;
};

struct Identification_Data
{
    float dt {};
    float pwm {};
    float angle {};
    float angle_dt {};
    float position {};
    float position_dt {};
};

void
update(float dt);

void
activate_identification();

bool
identification_active();

void
new_regulator_data_for_identification(Identification_Data const& data);

PWM_Sample const
get_pwm_sample();

void
identification_data_nus_parser_callback(char const* data);
