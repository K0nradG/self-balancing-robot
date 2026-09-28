// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once
#include <stdint.h>
#include "low_pass_filter.h"
#include "saturation.h"

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

    PID(Parameters parameters, Saturation output_saturation, float alpha, float hysteresis = 0.0f,
        bool use_feedback_dt = false)
        : m_parameters(parameters),
          m_output_saturation(output_saturation),
          m_filter(alpha),
          m_hysteresis(hysteresis),
          m_use_feedback_dt(use_feedback_dt)
    {
    }

    float
    calculate_output(float setpoint, float feedback, float dt, float feedback_dt = 0.0f);

    Parameters
    get_parameters() const;

    void
    set_parameters(Parameters parameters);

    void
    reset();

private:
    float m_integral {};
    float m_prev_error {};

    Parameters m_parameters;
    Saturation m_output_saturation;
    Low_Pass_Filter m_filter;
    float m_hysteresis;
    bool m_use_feedback_dt;
};

}  // namespace Robot_Control