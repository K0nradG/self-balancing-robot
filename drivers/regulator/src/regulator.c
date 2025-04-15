#include "regulator.h"
#include <math.h>
#include <stdlib.h>
#include "motor_controller.h"
#include "regulator_utils.h"
#include "utils.h"

#ifdef CONFIG_REGULATOR_LOG
#include "logger.h"
#endif  // CONFIG_REGULATOR_LOG

#ifdef CONFIG_LOG_OVER_BLE
#define BLE_NUS_MAX_DATA_LEN 251
#endif  // CONFIG_LOG_OVER_BLE

static bool automatic_control_started = false;
static float angle                    = 0.0f;

#if defined(CONFIG_MODEL_IDENTIFICATION_DRV) || defined(CONFIG_PID_ENABLED)
static float angle_dt = 0.0f;
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

static struct k_work_delayable regulator_work;
calculate_regulator_output_cb_t new_calculate_regulator_output_cb = NULL;
get_setpoint_cb_t new_get_setpoint_cb                             = NULL;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#include "regulator.h"
regulator_data_updated_cb_t new_pwm_cb = NULL;

#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#if defined(CONFIG_MODEL_IDENTIFICATION_DRV) || defined(CONFIG_PID_ENABLED)
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
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

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

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
void
new_pwm_cb_register(regulator_data_updated_cb_t _new_pwm_cb)
{
    if(_new_pwm_cb)
    {
        new_pwm_cb = _new_pwm_cb;
    }
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

static void
regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);
    float error = 0.0f - angle;  // low_pass_filter(angle);

    if(new_get_setpoint_cb)
    {
        error = new_get_setpoint_cb() * DEG_TO_RAD - (angle);
    }

    float output = 0.0f;
    if(new_calculate_regulator_output_cb)
    {
#ifdef CONFIG_PID_ENABLED
        output = new_calculate_regulator_output_cb(error, angle_dt);
#else
        output = new_calculate_regulator_output_cb(angle, angle_dt);
#endif  // CONFIG_PID_ENABLED
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
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_REGULATOR_LOG
    platform_log("REGULATOR", LOG_LEVEL_ERR, "angle_dt: %f  error %f out: %f", angle_dt, error, output);
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

void
new_calculate_regulator_output_cb_register(calculate_regulator_output_cb_t _new_calculate_regulator_output_cb)
{
    if(_new_calculate_regulator_output_cb)
    {
        new_calculate_regulator_output_cb = _new_calculate_regulator_output_cb;
    }
}

void
new_get_setpoint_cb_register(get_setpoint_cb_t _new_get_setpoint_cb)
{
    if(_new_get_setpoint_cb)
    {
        new_get_setpoint_cb = _new_get_setpoint_cb;
    }
}