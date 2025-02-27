#include "regulator.h"
#include "imu.h"
#include "motor_controller.h"

#ifdef CONFIG_REGULATOR_LOG
#include "logger.h"
#endif

#ifdef CONFIG_LOG_OVER_BLE
#include "ble_logger_service.h"

#define BLE_NUS_MAX_DATA_LEN 251
#endif

#define INIT_REGULATOR_PARAM_K 1
#define INIT_REGULATOR_PARAM_I 1
#define INIT_REGULATOR_PARAM_D 1
#define INIT_REGULATOR_PARAM_SETPOINT -90

#define ANGLE_ACCEPTABLE_OFFSET 1.0f

#define MAX_INTEGRAL 100
#define MAX_DERIVATIVE 100

static float integral   = 0;
static float last_error = 0;

struct imu_data new_imu_data;

struct pid_regulator_parameters _pid_regulator_parameters;

static struct k_work_delayable regulator_work;

static bool automatic_control_started;

regulator_params_updated_cb_t new_pid_regulator_parameters_cb;

static void
init_regulator_parameters()
{
    _pid_regulator_parameters.k        = INIT_REGULATOR_PARAM_K;
    _pid_regulator_parameters.i        = INIT_REGULATOR_PARAM_I;
    _pid_regulator_parameters.d        = INIT_REGULATOR_PARAM_D;
    _pid_regulator_parameters.setpoint = INIT_REGULATOR_PARAM_SETPOINT;
}

static void
new_imu_callback(struct imu_data data)
{
    new_imu_data = data;
}

void
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
                    _pid_regulator_parameters.k = value;
                    break;
                case 'i':
                    _pid_regulator_parameters.i = value;
                    break;
                case 'd':
                    _pid_regulator_parameters.d = value;
                    break;
                case 's':
                    _pid_regulator_parameters.setpiont = value;
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
    /*dont try logiing data here!!! it causes dongle crash due to to big amount of time taken when nuc data recived
    callback*/
}

void
new_nus_regulator_parameters_recived(const uint8_t* data, uint16_t len)
{
    if(len > BLE_NUS_MAX_DATA_LEN)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("APP", LOG_LEVEL_ERR, "Data length exceeds buffer size!");
#endif
        return;
    }

    static char received_data[BLE_NUS_MAX_DATA_LEN + 1];
    memset(received_data, 0, sizeof(received_data));

    memcpy(received_data, data, len);
    received_data[len] = '\0';

    platform_log("APP", LOG_LEVEL_ERR, "Received NUS data: %s", received_data);
    parse_data(received_data);
}

static int
init(void)
{
    init_regulator_parameters();
    new_imu_cb_register(new_imu_callback);

    new_nus_data_recived_cb_register(new_nus_regulator_parameters_recived);

    new_nus_data_recived_cb_register();

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

static void
regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    int angle_int_part   = new_imu_data.angle_data.angle_int;
    int angle_fract_part = new_imu_data.angle_data.angle_fract;

    float angle = angle_int_part + (angle_fract_part / 1000000.0f);

    float error = _pid_regulator_parameters.setpoint - angle;

    float output = 0.0f;

    if(fabs(error) >= ANGLE_ACCEPTABLE_OFFSET)
    {
        float dt = (float)CONFIG_REGULATOR_SAMPLE_TIME / 1000.0f;

        integral += error * dt;

        if(integral > MAX_INTEGRAL)
            integral = MAX_INTEGRAL;
        if(integral < -MAX_INTEGRAL)
            integral = -MAX_INTEGRAL;

        float derivative = (error - last_error) / dt;

        if(derivative > MAX_DERIVATIVE)
            derivative = MAX_DERIVATIVE;
        if(derivative < -MAX_DERIVATIVE)
            derivative = -MAX_DERIVATIVE;

        last_error = error;

        output = K * error + I * integral + D * derivative;
    }

    int pwm = (int)fabs(output);

    if(pwm > 100)
    {
        pwm = 100;
    }

    set_start_motors(true);
    set_duty_cycle_value(pwm);
    set_direction(output > 0 ? POSITIVE : NEGATIVE);

    reschedule_work(&regulator_work, K_MSEC(CONFIG_REGULATOR_SAMPLE_TIME), "automatic control");
}

static K_WORK_DELAYABLE_DEFINE(regulator_work, regulator_work_handler);

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
