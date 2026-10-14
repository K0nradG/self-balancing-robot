// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "protection_manager.h"

namespace Robot_Control
{

namespace
{
constexpr float pi             = 3.14159265358979323846f;
constexpr float radian_degrees = 180.0f;
}  // namespace

bool
Protection_Manager::validate_robot_angle(float balance_setpoint, float balance_angle) const
{
    static bool disable_motors_command           = false;
    static constexpr float safe_angle_margin     = 20.0f * (pi / radian_degrees);
    static constexpr float safe_angle_hysteresis = 0.5f * (pi / radian_degrees);

    float const upper_limit = balance_setpoint + safe_angle_margin;
    float const lower_limit = balance_setpoint - safe_angle_margin;

    if(!disable_motors_command && (balance_angle > upper_limit || balance_angle < lower_limit))
    {
        disable_motors_command = true;
        return disable_motors_command;
    }

    if(disable_motors_command && (balance_angle < (upper_limit - safe_angle_hysteresis)) &&
       (balance_angle > (lower_limit + safe_angle_hysteresis)))
    {
        disable_motors_command = false;
    }

    return disable_motors_command;
}

}  // namespace Robot_Control
