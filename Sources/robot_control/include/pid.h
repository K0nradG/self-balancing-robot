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

    PID(Parameters parameters, float output_saturation, float alpha, bool use_feedback_dt = false)
        : m_parameters(parameters),
          m_output_saturation(output_saturation),
          m_filter(alpha),
          m_use_feedback_dt(use_feedback_dt)
    {
    }

    float
    calculate_output(float setpoint, float feedback, float feedback_dt = 0.0f);

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
    bool m_use_feedback_dt;
};

}  // namespace Robot_Control