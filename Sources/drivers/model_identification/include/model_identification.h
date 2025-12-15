#pragma once

#include <array>

struct input_data
{
    std::array<float, 10> pwm_values;
    std::array<float, 10> pwm_durations_s;
};

struct pwm_sample
{
    float pwm0;
    float pwm1;
    bool last_sample;
};

struct identification_data
{
    float dt;
    float pwm;
    float angle;
    float angle_dt;
    float position;
    float position_dt;
};

typedef void (*identification_process_cb_t)(char const* data);

void
identification_data_nus_parser_callback(char const* data);

input_data&
get_input_pwm_data();

pwm_sample get_pwm_sample(std::size_t);

void
new_regulator_data_for_identification(identification_data data);

void
set_identification_data_status(bool status);
