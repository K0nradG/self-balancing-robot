#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "zephyr/kernel.h"

namespace Robot_Control
{

float
PID::calculate_output(float setpoint, float feedback, float feedback_dt)
{
    float const error = setpoint - m_filter.filter(feedback);
    if(fabsf(error) < m_hysteresis)
    {
        m_integral   = 0.0f;
        m_prev_error = error;
        return 0.0f;
    }

    int64_t const current_time = k_uptime_get();
    float const dt             = (m_last_time > 0) ? (current_time - m_last_time) / 1000.0f : 0.01f;
    m_last_time                = current_time;

    if(fabsf(m_parameters.Ki) < 1e-3f)
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

    float output = m_parameters.Kp * error + m_integral + derivative;

    if(fabsf(output) > m_output_saturation)
    {
        if((output * error) > 0)
        {
            m_integral -= m_parameters.Ki * error * dt;  // Revert the integral update - wind-up occurred.
        }
        output = MAX(output, -m_output_saturation);
        output = MIN(output, m_output_saturation);
    }

    return output;
}

#ifdef CONFIG_BLUETOOTH_DRV
void
PID::parse_nus_parameters(char const* data)
{
    if(data == nullptr)
    {
        return;
    }

    while(*data)
    {
        if(*data == 'k' || *data == 'i' || *data == 'd')
        {
            char key = *data;
            data++;
            char* next_data = nullptr;
            float value     = strtof(data, &next_data);

            if(data == next_data)
            {
                break;
            }
            data = next_data;

            switch(key)
            {
                case 'k':
                    m_parameters.Kp = value;
                    break;
                case 'i':
                    m_parameters.Ki = value;
                    break;
                case 'd':
                    m_parameters.Kd = value;
                    break;
                case 'f':
                    m_filter.set_alpha(value);
                    break;
                default:
                    break;
            }
        }
        else
        {
            data++;
        }
    }
}
#endif  // CONFIG_BLUETOOTH_DRV

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