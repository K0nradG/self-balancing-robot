// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

namespace Robot_Control
{

class LQR
{
public:
    struct Parameters
    {
        float Kx = 0.0f;
        float Ky = 0.0f;
    };

    LQR(Parameters parameters, float output_saturation)
        : m_parameters(parameters), m_output_saturation(output_saturation)
    {
    }

    float
    calculate_output(float x, float y);

    void
    set_parameters(Parameters parameters);

    Parameters
    get_parameters() const;

private:
    Parameters m_parameters;
    float m_output_saturation;
};

}  // namespace Robot_Control
