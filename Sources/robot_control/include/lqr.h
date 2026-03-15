#pragma once

namespace Robot_Control
{

class LQR
{
public:
    struct Parameters
    {
        float K_position;
        float K_position_derivative;
        float K_angle;
        float K_angle_derivative;
    };

    LQR(Parameters parameters, float output_saturation)
        : m_parameters(parameters), m_output_saturation(output_saturation)
    {
    }

    struct system_state
    {
        float position;
        float position_derivative;
        float angle;
        float angle_derivative;
    };

    float
    calculate_output(system_state state);

#ifdef CONFIG_BLUETOOTH_DRV
    void
    parse_nus_parameters(char const* data);
#endif  // CONFIG_BLUETOOTH_DRV

private:
    Parameters m_parameters;
    float m_output_saturation;
};

}  // namespace Robot_Control
