#include "regulator.h"
#include <math.h>
#include <stdlib.h>
#include "motor_controller.h"
#include "utils.h"

#ifdef CONFIG_REGULATOR_LOG
#include "logger.h"
#endif  // CONFIG_REGULATOR_LOG

#ifdef CONFIG_LOG_OVER_BLE
#define BLE_NUS_MAX_DATA_LEN 251
#endif  // CONFIG_LOG_OVER_BLE

#define ANGLE_ACCEPTABLE_OFFSET 1.0f
#define MAX_INTEGRAL 100.0f
#define MAX_DERIVATIVE 100.0f
#define MS_TO_SECONDS 0.001f

static bool automatic_control_started                            = false;
static float angle                                               = 0.0f;
static struct pid_regulator_parameters _pid_regulator_parameters = {
    .K = 3.5f, .I = 0.0f, .D = 0.2f, .setpoint = -95.0f};
static struct k_work_delayable regulator_work;

regulator_params_updated_cb_t new_pid_regulator_parameters_cb = NULL;

void
new_imu_angle_for_regulator(float _angle)
{
    angle = _angle;
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
    parse_data(received_data);
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
                    _pid_regulator_parameters.K = value;
                    break;
                case 'i':
                    _pid_regulator_parameters.I = value;
                    break;
                case 'd':
                    _pid_regulator_parameters.D = value;
                    break;
                case 's':
                    _pid_regulator_parameters.setpoint = value;
                    break;
            }
            if(new_pid_regulator_parameters_cb)
            {
                new_pid_regulator_parameters_cb(_pid_regulator_parameters);
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

static int
init(void)
{
    set_enable_controller(true);
    motor_controller_start();

#ifdef CONFIG_REGULATOR_LOG
    platform_log("REGULATOR", LOG_LEVEL_INF, "regulator init finished");
#endif  // CONFIG_REGULATOR_LOG
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void
new_pid_regulator_parameters_cb_register(regulator_params_updated_cb_t _new_pid_regulator_parameters_cb)
{
    if(_new_pid_regulator_parameters_cb)
    {
        new_pid_regulator_parameters_cb = _new_pid_regulator_parameters_cb;
    }
}

static float
calculate_pid_output(void);

static void
regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    float const output = calculate_pid_output();
#ifdef CONFIG_REGULATOR_LOG
    platform_log("REGULATOR", LOG_LEVEL_ERR, "Output: %d", (int)output);
#endif  // CONFIG_REGULATOR_LOG

    int pwm = (int)fabs((double)output);
    if(pwm > CONFIG_PWM_LIMIT)
    {
        pwm = CONFIG_PWM_LIMIT;
    }

    set_start_motors(true);
    set_duty_cycle_value(pwm);
    set_direction(output > 0 ? POSITIVE : NEGATIVE);

    reschedule_work(&regulator_work, K_MSEC(CONFIG_REGULATOR_SAMPLE_TIME), "automatic control");
}

static K_WORK_DELAYABLE_DEFINE(regulator_work, regulator_work_handler);

static float
calculate_pid_output(void)
{
    float error  = _pid_regulator_parameters.setpoint - angle;
    float output = 0.0f;

    if((float)fabs((double)error) >= ANGLE_ACCEPTABLE_OFFSET)
    {
        float dt = (float)CONFIG_REGULATOR_SAMPLE_TIME * MS_TO_SECONDS;

        static float integral = 0;
        integral += error * dt;

        if(integral > MAX_INTEGRAL)
        {
            integral = MAX_INTEGRAL;
        }

        if(integral < -MAX_INTEGRAL)
        {
            integral = -MAX_INTEGRAL;
        }

        static float last_error = 0;
        float derivative        = (error - last_error) / dt;

        if(derivative > MAX_DERIVATIVE)
        {
            derivative = MAX_DERIVATIVE;
        }

        if(derivative < -MAX_DERIVATIVE)
        {
            derivative = -MAX_DERIVATIVE;
        }
        last_error = error;

        output = _pid_regulator_parameters.K * error + _pid_regulator_parameters.I * integral +
                 _pid_regulator_parameters.D * derivative;
    }
    return output;
}

void
regulator_start_automatic_control(void)
{
    if(automatic_control_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator worker already started");
#endif  // CONFIG_REGULATOR_LOG
    }
    automatic_control_started = true;

    reschedule_work(&regulator_work, K_NO_WAIT, "automatic control");
}

void
regulator_stop_automatic_control(void)
{
    if(!automatic_control_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator worker not started");
#endif  // CONFIG_REGULATOR_LOG
    }
    automatic_control_started = false;

    const int ret = k_work_cancel_delayable(&regulator_work);
    if(ret)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "cancel regulator work err:%d", ret);
#endif  // CONFIG_REGULATOR_LOG
        return;
    }

#ifdef CONFIG_REGULATOR_LOG
    platform_log("REGULATOR", LOG_LEVEL_DBG, "automatic control work cancelled");
#endif  // CONFIG_REGULATOR_LOG
}
