#include "lqr.h"
#include <stdlib.h>
#include <string.h>
#include "regulator_utils.h"

static struct lqr_parameters lqr_parameters   = {.Kx = 0.0f, .Ky = 0.0f};
lqr_params_updated_cb_t new_lqr_parameters_cb = NULL;

void
new_lqr_parameters_cb_register(lqr_params_updated_cb_t _new_lqr_parameters_cb)
{
    if(_new_lqr_parameters_cb)
    {
        new_lqr_parameters_cb = _new_lqr_parameters_cb;
    }
}

float
calculate_regulator_output(float angle, float angle_dt)
{
    float const output =
        -(lqr_parameters.Kx * angle + lqr_parameters.Ky * angle_dt);  // u = -Kx control law (x - state vector).

    return limit(output, -(float)CONFIG_PWM_LIMIT, (float)CONFIG_PWM_LIMIT);
}

float
get_setpoint(void)
{
    return lqr_parameters.setpoint;
}

#ifdef CONFIG_LOG_OVER_BLE
void
parse_regulator_data(const char* data)
{
    const char* ptr = data;
    while(*ptr)
    {
        if(*ptr == 'x' || *ptr == 'y' || *ptr == 's')
        {
            char key = *ptr;
            ptr++;
            char* next_ptr;
            float value = strtof(ptr, &next_ptr);

            if(ptr == next_ptr)
            {
                break;
            }
            ptr = next_ptr;

            switch(key)
            {
                case 'x':
                    lqr_parameters.Kx = value;
                    break;
                case 'y':
                    lqr_parameters.Ky = value;
                    break;
                case 's':
                    lqr_parameters.setpoint = value;
                    break;
            }
            if(new_lqr_parameters_cb)
            {
                new_lqr_parameters_cb(lqr_parameters);
            }
        }
        else
        {
            ptr++;
        }
    }
    /*dont try logging data here!!! it causes dongle crash due to to big amount of time taken when nuc data received
    callback*/
}
#endif  // CONFIG_LOG_OVER_BLE