#pragma once
#include <cstdint>
#include "low_pass_filter.h"

namespace Robot_Control
{

class PID
{
public:
    struct Parameters
    {
        float Kp = 0.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
    };

    PID(Parameters parameters, float output_saturation, float alpha)
        : m_parameters(parameters), m_output_saturation(output_saturation), m_filter(alpha)
    {
    }

    float
    calculate_output(float setpoint, float feedback);

#ifdef CONFIG_LOG_OVER_BLE
    void
    parse_nus_parameters(char const* data);
#endif  // CONFIG_LOG_OVER_BLE

    Parameters
    get_parameters() const;

    void
    set_parameters(Parameters parameters);

private:
    std::int64_t m_last_time {};
    float m_integral {};
    float m_prev_error {};
    Parameters m_parameters;
    float m_output_saturation;
    Low_Pass_Filter m_filter;
};

}  // namespace Robot_Control