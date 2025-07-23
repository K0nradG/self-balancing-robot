#pragma once

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

#ifdef CONFIG_LOG_OVER_BLE
    void
    parse_nus_parameters(char const* data);
#endif  // CONFIG_LOG_OVER_BLE

private:
    Parameters m_parameters;
    float m_output_saturation;
};