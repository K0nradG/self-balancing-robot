#include "lqr.h"
#include <stdlib.h>
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
        if(*data == 'x' || *data == 'y')
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
                    m_parameters.Kx = value;
                    break;
                case 'y':
                    m_parameters.Ky = value;
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