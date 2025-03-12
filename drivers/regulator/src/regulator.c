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

#define MS_TO_SECONDS 0.001f
#define M_PI 3.14159265358979323846f
#define N (2.0f * M_PI * (float)CONFIG_FILTER_CUTOFF_FREQUENCY)         // Filter coefficient in [rad/s]
#define N_dt (N * (float)CONFIG_REGULATOR_SAMPLE_TIME * MS_TO_SECONDS)  // [rad]
#define ALPHA      \
    N_dt / (1.0f + \
            N_dt)  // Alpha coefficients to be used directly by the low-pass filter: alpha = (N * dt) / (1 + N * dt)

static bool automatic_control_started = false;
static float angle                    = 0.0f;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
static float angle_dt = 0.0f;
#endif

static struct pid_regulator_parameters _pid_regulator_parameters = {
    .Kp = 650.3f, .Ki = 4.1f, .Kd = 2.0f, .setpoint = 0.0f};
#define ANGLE_OFFSET 98

static struct k_work_delayable regulator_work;

regulator_params_updated_cb_t new_pid_regulator_parameters_cb = NULL;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#include "regulator.h"
regulator_data_updated_cb_t new_pwm_cb = NULL;

#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
void
new_imu_angle_for_regulator(struct identification_data data)
{
    angle    = data.angle;
    angle_dt = data.angle_dt;
}

#else
void
new_imu_angle_for_regulator(float _angle)
{
    angle = _angle;
}
#endif

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
                    _pid_regulator_parameters.Kp = value;
                    break;
                case 'i':
                    _pid_regulator_parameters.Ki = value;
                    break;
                case 'd':
                    _pid_regulator_parameters.Kd = value;
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

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
void
new_pwm_cb_register(regulator_data_updated_cb_t _new_pwm_cb)
{
    if(_new_pwm_cb)
    {
        new_pwm_cb = _new_pwm_cb;
    }
}
#endif

static float
calculate_pid_output(float error);

static void
regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);
    float const error =
        (_pid_regulator_parameters.setpoint) * (M_PI / 180.0f) - (angle + (ANGLE_OFFSET * (M_PI / 180.0f)));
    float const output = calculate_pid_output(error);
#ifdef CONFIG_REGULATOR_LOG
    platform_log("REGULATOR", LOG_LEVEL_ERR, "angle: %d  error %d", (int)angle, (int)error);
#endif  // CONFIG_REGULATOR_LOG

    int pwm = (int)fabsf(output);

    set_start_motors(true);
    set_duty_cycle_value(pwm);
    set_direction(error > 0 ? POSITIVE : NEGATIVE);
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    trigger_collecting_identification_data();
#endif

    trigger_motors_update();
    reschedule_work(&regulator_work, K_MSEC(CONFIG_REGULATOR_SAMPLE_TIME), "automatic control");
}

static K_WORK_DELAYABLE_DEFINE(regulator_work, regulator_work_handler);

static float
limit(float input, float lower_bound, float upper_bound);

static float
low_pass_filter(float input);

static float
calculate_pid_output(float error)
{
    static int64_t last_time   = 0;
    int64_t const current_time = k_uptime_get();

    float const dt = (last_time > 0) ? (current_time - last_time) / 1000.0f : 0.01f;
    last_time      = current_time;

    float const proportional = _pid_regulator_parameters.Kp * error;

    static float integral = 0;
    integral += _pid_regulator_parameters.Ki * error * dt;

    static float last_error               = 0;
    float const error_difference_filtered = low_pass_filter(error - last_error);
    float const derivative                = _pid_regulator_parameters.Kd * (error_difference_filtered / dt);
    last_error                            = error;

    float output = proportional + integral + derivative;

    if(fabsf(output) > (float)CONFIG_PWM_LIMIT)
    {
        if((output * error) > 0)
        {
            integral -= _pid_regulator_parameters.Ki * error * dt;  // Revert the integral update - wind-up occurred.
        }
        output = limit(output, -(float)CONFIG_PWM_LIMIT, (float)CONFIG_PWM_LIMIT);
    }
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV

    struct identification_regulator_data data;
    data.dt       = k_uptime_get();
    data.pwm      = output;
    data.angle    = angle;
    data.angle_dt = angle_dt;

    if(new_pwm_cb)
    {
        new_pwm_cb(data);
    }
#endif

    return output;
}

static float
limit(float input, float lower_bound, float upper_bound)
{
    if(input < lower_bound)
    {
        input = lower_bound;
    }
    if(input > upper_bound)
    {
        input = upper_bound;
    }
    return input;
}

static float
low_pass_filter(float input)
{
    static float last_output = 0.0f;
    float const output       = ALPHA * input + (1 - ALPHA) * last_output;
    last_output              = output;
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

    set_start_motors(false);

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
