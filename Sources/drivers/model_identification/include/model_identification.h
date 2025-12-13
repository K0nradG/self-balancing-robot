#pragma once

struct identification_data
{
    float dt;
    float pwm;
    float angle;
    float angle_dt;
};

void
new_regulator_data_for_identification(identification_data data);
