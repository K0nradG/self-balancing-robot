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
    bool last_sample;
};

struct Identification_Data
{
    float dt;
    float pwm;
    float angle;
    float angle_dt;
    float position;
    float position_dt;
};

void
identification_data_nus_parser_callback(char const* data);

Input_Data const&
get_input_pwm_data();

PWM_Sample const
get_pwm_sample(uint32_t sample_index);

void
new_regulator_data_for_identification(Identification_Data const& data);

void
set_identification_data_status(bool status);
