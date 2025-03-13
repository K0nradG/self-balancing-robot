#include "lqr.h"
#include <stdlib.h>
#include <string.h>
#include "regulator_utils.h"

#ifdef CONFIG_REGULATOR_LOG
#include "logger.h"
#endif  // CONFIG_REGULATOR_LOG

static struct lqr_parameters lqr_parameters   = {.Kx = 0.0f, .Ky = 0.0f};
lqr_params_updated_cb_t new_lqr_parameters_cb = NULL;

float
calculate_lqr_output(float angle, float angle_dt)
{
    float const output =
        -(lqr_parameters.Kx * angle + lqr_parameters.Ky * angle_dt);  // u = -Kx control law (x - state vector).

    return limit(output, -(float)CONFIG_PWM_LIMIT, (float)CONFIG_PWM_LIMIT);
}

void
new_lqr_parameters_cb_register(lqr_params_updated_cb_t _new_lqr_parameters_cb)
{
    if(_new_lqr_parameters_cb)
    {
        new_lqr_parameters_cb = _new_lqr_parameters_cb;
    }
}

#ifdef CONFIG_LOG_OVER_BLE
static void
parse_data(const char* data);

void
new_nus_parameters_received_for_lqr(const uint8_t* data, uint16_t len)
{
    if(len > BLE_NUS_MAX_DATA_LEN)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("APP", LOG_LEVEL_ERR, "Data length exceeds buffer size!");
#endif  // CONFIG_REGULATOR_LOG
        return;
    }

    static char received_data[BLE_NUS_MAX_DATA_LEN + 1];
    memset(received_data, 0, sizeof(received_data));

    memcpy(received_data, data, len);
    received_data[len] = '\0';
#ifdef CONFIG_REGULATOR_LOG
    platform_log("APP", LOG_LEVEL_ERR, "Received NUS data: %s", received_data);
#endif  // CONFIG_REGULATOR_LOG

    parse_data(data);
}

static void
parse_data(const char* data)
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

float
get_setpoint_lqr(void)
{
    return lqr_parameters.setpoint;
}