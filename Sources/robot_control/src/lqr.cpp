// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "lqr.h"
#include "zephyr/sys/util.h"

namespace Robot_Control
{

float
LQR::calculate_output(float x, float y)
{
    float output = -(m_parameters.Kx * x + m_parameters.Ky * y);  // u = -Kx control law (x - state vector).
    output       = MAX(output, -m_output_saturation);
    output       = MIN(output, m_output_saturation);

    return output;
}

void
LQR::set_parameters(Parameters parameters)
{
    m_parameters = parameters;
}

LQR::Parameters
LQR::get_parameters() const
{
    return m_parameters;
}

}  // namespace Robot_Control