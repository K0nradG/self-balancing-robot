#pragma once
#include <cstdint>

class PID
{
public:
    struct Parameters
    {
        float Kp = 0.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
    };

    PID(Parameters parameters) : m_parameters(parameters) {}

    float
    calculate_output(float error);

    void
    parse_nus_parameters(char const* data);

private:
    std::int64_t m_last_time {};
    float m_integral {};
    float m_prev_error {};
    Parameters m_parameters;
};