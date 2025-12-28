#pragma once

#include <stdint.h>

struct encoder_data
{
    /*800 impulses per motor shaft rotate*/
    int32_t impulse_count;
    float shaft_rotate_count;
    float shaft_angle_rad;
    float distance_m;
    float angular_velocity_rad_s;
    float linear_velocity_m_s;
};

struct encoders_data
{
    encoder_data encoder_0;
    encoder_data encoder_1;
    float robot_angle_rad;
    float robot_distance_m;
    float robot_linear_speed;
};

int
encoders_init();

// _ added since the same naming was used in data manager - causes wrong calls.
encoders_data const&
_get_encoders_data();

//! @brief Resets entire encoders struct
void
reset_encoders();