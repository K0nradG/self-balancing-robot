#include "lqr.h"
#include <stdlib.h>
#include <algorithm>
#include "logger.h"
#include "zephyr/sys/util.h"

namespace Robot_Control
{

static Logger<IS_ENABLED(CONFIG_ROBOT_CONTROL_LOG)> lqr_logger("LQR");

#include <algorithm>  // Do użycia std::clamp

float
LQR::calculate_output(LQR::system_state state)
{
    // 1. Zdefiniuj limity Twojego sprzętu (dostosuj do swojego sterownika PWM)
    const float MAX_PWM      = 100.0f;  // lub 255.0f jeśli używasz 8-bitowego PWM
    const float DEADBAND     = 3.0f;    // Tyle PWM potrzeba, żeby koło w ogóle ruszyło
    const float ANGLE_OFFSET = 0.0f;    // DOSTOSUJ: Wartość kąta, gdy robot stoi fizycznie w balansie!

    // 2. Oblicz uchyb kąta
    float error_angle = state.angle - ANGLE_OFFSET;

    // 3. Surowe równanie LQR
    float out =
        -(m_parameters.K_position * state.position + m_parameters.K_position_derivative * state.position_derivative +
          m_parameters.K_angle * error_angle + m_parameters.K_angle_derivative * state.angle_derivative);

    // 4. Kompensacja strefy nieczułości (Deadband) - zapobiega drżeniu wokół pionu
    if(out > 0.5f)
    {
        out += DEADBAND;
    }
    else if(out < -0.5f)
    {
        out -= DEADBAND;
    }
    else
    {
        out = 0.0f;
    }

    // 5. SATURACJA (Najważniejsze!)
    // Nie pozwala sterownikowi zwariować, ucina wynik do np. [-100, 100]
    out = std::clamp(out, -MAX_PWM, MAX_PWM);

    // lqr_logger.platform_log(
    //     LOG_LEVEL::DBG, "LQR state: pos=%f, d_pos=%f, ang=%f, d_ang=%f, out_limited=%f", state.position,
    //     state.position_derivative, state.angle, state.angle_derivative, out);

    return out;
}

#ifdef CONFIG_BLUETOOTH_DRV
void
LQR::parse_nus_parameters(char const* data)
{
    if(data == nullptr)
    {
        return;
    }

    while(*data)
    {
        if((*data == 'x') || (*data == 'y'))
        {
            char key = *data;
            data++;
            char* next_data;
            float value = strtof(data, &next_data);

            if(data == next_data)
            {
                break;
            }
            data = next_data;

            switch(key)
            {
                case 'x':
                    m_parameters.K_angle = value;
                    break;
                case 'y':
                    m_parameters.K_angle_derivative = value;
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

}  // namespace Robot_Control