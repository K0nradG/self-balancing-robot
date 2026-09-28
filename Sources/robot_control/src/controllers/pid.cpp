// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "pid.h"
#include <math.h>
#include "zephyr/sys/util.h"

namespace Robot_Control
{

float
PID::calculate_output(float setpoint, float feedback, float dt, float feedback_dt)
{
    static constexpr float abs_diff = 1e-3f;
    static constexpr float dt_min   = 0.001f;
    static constexpr float dt_max   = 0.05f;
    dt                              = MAX(MIN(dt, dt_max), dt_min);

    float const error = setpoint - m_filter.filter(feedback);
    if(fabsf(error) < m_hysteresis)
    {
        m_integral   = 0.0f;
        m_prev_error = error;
        return 0.0f;
    }

    if(fabsf(m_parameters.Ki) < abs_diff)
    {
        m_integral = 0.0f;
    }
    else
    {
        m_integral += m_parameters.Ki * error * dt;
    }

    float derivative = 0.0f;

    if(m_use_feedback_dt)
    {
        derivative = m_parameters.Kd * feedback_dt;
    }
    else
    {
        float const error_difference = error - m_prev_error;
        m_prev_error                 = error;
        derivative                   = m_parameters.Kd * (error_difference / dt);
    }

    float const output           = m_parameters.Kp * error + m_integral + derivative;
    float const saturated_output = m_output_saturation.saturate(output);

    if((fabsf(output - saturated_output) > abs_diff) && ((output * error) > 0))
    {
        m_integral -= m_parameters.Ki * error * dt;  // Revert the integral update - wind-up occurred.
    }

    return saturated_output;
}

PID::Parameters
PID::get_parameters() const
{
    return m_parameters;
}

void
PID::set_parameters(Parameters parameters)
{
    m_parameters = parameters;
}

void
PID::reset()
{
    m_integral   = 0.0f;
    m_prev_error = 0.0f;
}

}  // namespace Robot_Control