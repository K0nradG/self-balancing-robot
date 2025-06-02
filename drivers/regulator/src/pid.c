#include "pid.h"
#include <math.h>
#include <stdlib.h>
#include "regulator_utils.h"
#include "zephyr/kernel.h"

// For identification apply K = 1200.0f and setpoint set to -8.5f.

// PID tuner with a little hand tuning (lowering I part and increasing D):
// static struct pid_regulator_parameters g_pid_regulator_parameters = {
//     .Kp = 886.52735495982f, .Ki = 10000.0f, .Kd = 1.0f, /*.N = 3097.63192969837,*/ .setpoint = -7.4485f};

// Hand tuned.
// static struct pid_regulator_parameters g_pid_regulator_parameters = {
//     .Kp = 1200.0f, .Ki = 1000.0f, .Kd = 0.1f, .setpoint = -7.4485f};

// A little to aggressive (maybe hand tune?)
// static struct pid_regulator_parameters g_pid_regulator_parameters = {
//     .Kp       = 479.901562064471f,
//     .Ki       = 33282.2230885236f,
//     .Kd       = 0.304399729736007f,
//     .setpoint = -7.4485f};}

typedef struct
{
    int64_t last_time;
    float integral;
    float last_error;
    struct pid_regulator_parameters parameters;
} pid_regulator;

static pid_regulator g_balance_regulator = {
    .last_time  = 0,
    .integral   = 0.0f,
    .last_error = 0.0f,
    .parameters = {.Kp = 1300.0, .Ki = 800.0f, .Kd = 0.1f, .setpoint = -7.0f}};  //-6.8 , -7.4485

// TODO: Setpoint not larger than += 45 degrees from previous point works.
static pid_regulator g_rotation_regulator = {
    .last_time  = 0,
    .integral   = 0.0f,
    .last_error = 0.0f,
    .parameters = {.Kp = 80.0f, .Ki = 0.0f, .Kd = 0.0f, .setpoint = 0.0f}};

pid_params_updated_cb_t g_new_pid_parameters_cb = NULL;

void
new_pid_parameters_cb_register(pid_params_updated_cb_t new_pid_parameters_cb)
{
    if(new_pid_parameters_cb)
    {
        g_new_pid_parameters_cb = new_pid_parameters_cb;
    }
}

static float
calculate_pid_output(float error, pid_regulator* pid_regulator)
{
    if(pid_regulator == NULL)
    {
        return 0.0f;
    }

    int64_t const current_time = k_uptime_get();

    float const dt = (pid_regulator->last_time > 0) ? (current_time - pid_regulator->last_time) / 1000.0f : 0.01f;
    pid_regulator->last_time = current_time;

    float const proportional = pid_regulator->parameters.Kp * error;

    if(fabsf(pid_regulator->parameters.Ki) < 1e-3f)
    {
        pid_regulator->integral = 0.0f;
    }
    else
    {
        pid_regulator->integral += pid_regulator->parameters.Ki * error * dt;
    }

    static float last_error      = 0;
    float const error_difference = error - last_error;
    float const derivative       = pid_regulator->parameters.Kd * (error_difference / dt);
    last_error                   = error;

    float output = proportional + pid_regulator->integral + derivative;

    if(fabsf(output) > (float)CONFIG_PWM_LIMIT)
    {
        if((output * error) > 0)
        {
            pid_regulator->integral -=
                pid_regulator->parameters.Ki * error * dt;  // Revert the integral update - wind-up occurred.
        }
        output = limit(output, -(float)CONFIG_PWM_LIMIT, (float)CONFIG_PWM_LIMIT);
    }

    return output;
}

float
calculate_balance_regulator_output(float error)
{
    return calculate_pid_output(error, &g_balance_regulator);
}

float
calculate_rotation_regulator_output(float error)
{
    return calculate_pid_output(error, &g_rotation_regulator);
}

float
get_balance_setpoint(void)
{
    return g_balance_regulator.parameters.setpoint;
}

float
get_rotation_setpoint(void)
{
    return g_rotation_regulator.parameters.setpoint;
}

#ifdef CONFIG_LOG_OVER_BLE

static void
parse_pid_params(pid_regulator* pid_regulator, const char* ptr)
{
    ptr++;
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
                    pid_regulator->parameters.Kp = value;
                    break;
                case 'i':
                    pid_regulator->parameters.Ki = value;
                    break;
                case 'd':
                    pid_regulator->parameters.Kd = value;
                    break;
                case 's':
                    pid_regulator->parameters.setpoint = value;
                    break;
            }
            if(g_new_pid_parameters_cb)
            {
                g_new_pid_parameters_cb(pid_regulator->parameters);
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

void
parse_regulator_data(const char* data)
{
    const char* ptr = data;
    if(*ptr == 'b')
    {
        parse_pid_params(&g_balance_regulator, ptr);
    }
    else if(*ptr == 'r')
    {
        parse_pid_params(&g_rotation_regulator, ptr);
    }
}
#endif  // CONFIG_LOG_OVER_BLE