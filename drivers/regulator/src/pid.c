#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "regulator_utils.h"
#include "zephyr/kernel.h"

#ifdef CONFIG_REGULATOR_LOG
#include "logger.h"
#endif  // CONFIG_REGULATOR_LOG

static struct pid_regulator_parameters pid_regulator_parameters = {
    .Kp = 650.3f, .Ki = 4.1f, .Kd = 2.0f, .setpoint = 0.0f};

pid_params_updated_cb_t new_pid_parameters_cb = NULL;

float
calculate_regulator_output(float error)
{
    static int64_t last_time   = 0;
    int64_t const current_time = k_uptime_get();

    float const dt = (last_time > 0) ? (current_time - last_time) / 1000.0f : 0.01f;
    last_time      = current_time;

    float const proportional = pid_regulator_parameters.Kp * error;

    static float integral = 0;
    integral += pid_regulator_parameters.Ki * error * dt;

    static float last_error               = 0;
    float const error_difference_filtered = low_pass_filter(error - last_error);
    float const derivative                = pid_regulator_parameters.Kd * (error_difference_filtered / dt);
    last_error                            = error;

    float output = proportional + integral + derivative;

    if(fabsf(output) > (float)CONFIG_PWM_LIMIT)
    {
        if((output * error) > 0)
        {
            integral -= pid_regulator_parameters.Ki * error * dt;  // Revert the integral update - wind-up occurred.
        }
        output = limit(output, -(float)CONFIG_PWM_LIMIT, (float)CONFIG_PWM_LIMIT);
    }

    return output;
}

void
new_pid_parameters_cb_register(pid_params_updated_cb_t _new_pid_parameters_cb)
{
    if(_new_pid_parameters_cb)
    {
        new_pid_parameters_cb = _new_pid_parameters_cb;
    }
}

#ifdef CONFIG_LOG_OVER_BLE
static void
parse_data(const char* data);

void
new_nus_parameters_received_for_regulator(const uint8_t* data, uint16_t len)
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
        if(*ptr == 'k' || *ptr == 'i' || *ptr == 'd' || *ptr == 's')
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
                case 'k':
                    pid_regulator_parameters.Kp = value;
                    break;
                case 'i':
                    pid_regulator_parameters.Ki = value;
                    break;
                case 'd':
                    pid_regulator_parameters.Kd = value;
                    break;
                case 's':
                    pid_regulator_parameters.setpoint = value;
                    break;
            }
            if(new_pid_parameters_cb)
            {
                new_pid_parameters_cb(pid_regulator_parameters);
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
get_setpoint(void)
{
    return pid_regulator_parameters.setpoint;
}