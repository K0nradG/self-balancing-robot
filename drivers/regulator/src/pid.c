#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "regulator_utils.h"
#include "zephyr/kernel.h"

// the best
//     .Kp = 256.2f, .Ki = 1001.0f, .Kd = 8.22f, .setpoint = 0.0f

// great
//.Kp = 180.7f, .Ki = 1145.0f, .Kd = 4.24f, .setpoint = 0.0f}

static struct pid_regulator_parameters pid_regulator_parameters = {
    .Kp       = 550.0f,
    .Ki       = 500.0f,
    .Kd       = 5.0f,
    .setpoint = -7.4485f};  // k450 i0 d10  s2.3   //Kp = 550.0f, .Ki = 500.0f, .Kd = 5.0f, .setpoint = -7.4485f}
pid_params_updated_cb_t new_pid_parameters_cb = NULL;

static float const K_feedworward = 10.0f;

void
new_pid_parameters_cb_register(pid_params_updated_cb_t _new_pid_parameters_cb)
{
    if(_new_pid_parameters_cb)
    {
        new_pid_parameters_cb = _new_pid_parameters_cb;
    }
}

float
calculate_regulator_output(float error, float angle_dt)
{
    // if(error < 0)
    //{
    //     error *= 5;
    // }
    static int64_t last_time   = 0;
    int64_t const current_time = k_uptime_get();

    float const dt = (last_time > 0) ? (current_time - last_time) / 1000.0f : 0.01f;
    last_time      = current_time;

    float const proportional = pid_regulator_parameters.Kp * error;

    static float integral = 0;
    integral += pid_regulator_parameters.Ki * error * dt;

    static float last_error               = 0;
    float const error_difference_filtered = error - last_error;  // low_pass_filter(error - last_error);
    float const derivative                = pid_regulator_parameters.Kd * (error_difference_filtered / dt);
    last_error                            = error;

    float output = proportional + integral + derivative;  //- (K_feedworward * angle_dt);

    // if(error < 0)
    // {
    //     output *= 2;
    // }

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

float
get_setpoint(void)
{
    return pid_regulator_parameters.setpoint;
}

#ifdef CONFIG_LOG_OVER_BLE
void
parse_regulator_data(const char* data)
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