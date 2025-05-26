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

static bool g_automatic_control_started = false;

static struct k_work_delayable balance_regulator_work;
static struct k_work_delayable rotation_regulator_work;

typedef struct
{
    get_setpoint_cb_t get_setpoint_cb;
    calculate_regulator_output_cb_t calculate_regulator_output_cb;
    float angle;
    float angle_dt;
} regulator_context_t;

static regulator_context_t g_balance_regulator_context  = {0};
static regulator_context_t g_rotation_regulator_context = {0};

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"

send_identification_data_cb_t g_send_identification_data_cb = NULL;
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

void
new_imu_data_for_regulator(imu_data imu_data)
{
    g_balance_regulator_context.angle    = imu_data.angle_balance;
    g_balance_regulator_context.angle_dt = imu_data.angle_balance_dt;
    g_rotation_regulator_context.angle   = imu_data.angle_rotation;
}

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
new_send_identification_data_cb_register(send_identification_data_cb_t new_send_identification_data_cb)
{
    if(new_send_identification_data_cb)
    {
        g_send_identification_data_cb = new_send_identification_data_cb;
    }
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

static void
send_motors_data(int pwm, DIRECTION direction)
{
    set_start_motors(true);
    set_duty_cycle_value(pwm);
    set_direction(direction);
}

static void
common_regulator_work_handler(regulator_context_t* regulator_context, float* out)
{
    if(regulator_context == NULL || out == NULL)
    {
        return;
    }

    float error = 0.0f - input_low_pass_filter(regulator_context->angle);

    if(regulator_context->get_setpoint_cb)
    {
        error = regulator_context->get_setpoint_cb() * DEG_TO_RAD - (regulator_context->angle);
    }

    float output = 0.0f;
    if(regulator_context->calculate_regulator_output_cb)
    {
#ifdef CONFIG_PID_ENABLED
        output = regulator_context->calculate_regulator_output_cb(error);
#else
        output =
            regulator_context->calculate_regulator_output_cb(regulator_context->angle, regulator_context->angle_dt);
#endif  // CONFIG_PID_ENABLED
    }

#ifdef CONFIG_REGULATOR_LOG
    platform_log("REGULATOR", LOG_LEVEL_ERR, "error %f out: %f", (double)error, (double)output);
#endif  // CONFIG_REGULATOR_LOG

    out[0] = error;
    out[1] = output;
}

static void
balance_regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    float out[2] = {0};
    common_regulator_work_handler(&g_balance_regulator_context, out);

    float const error  = out[0];
    float const output = out[1];

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV

    struct identification_data data = {
        .dt       = k_uptime_get(),
        .pwm      = output,
        .angle    = g_balance_regulator_context.angle,
        .angle_dt = g_balance_regulator_context.angle_dt};

    if(g_send_identification_data_cb)
    {
        g_send_identification_data_cb(data);
    }
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

    int pwm = (int)fabsf(output);

    send_motors_data(pwm, (error > 0) ? POSITIVE : NEGATIVE);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    trigger_collecting_identification_data();
#endif

    trigger_motors_update();
    reschedule_work(&balance_regulator_work, K_MSEC(CONFIG_REGULATOR_SAMPLE_TIME), "balance control");
}

static K_WORK_DELAYABLE_DEFINE(balance_regulator_work, balance_regulator_work_handler);

static void
rotation_regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    float out[2] = {0};
    common_regulator_work_handler(&g_rotation_regulator_context, out);

    float const error  = out[0];
    float const output = out[1];

#ifdef CONFIG_REGULATOR_LOG
    platform_log(
        "REGULATOR", LOG_LEVEL_ERR, "rot: %f [deg]", (double)g_rotation_regulator_context.angle * (double)RAD_TO_DEG);
#endif  // CONFIG_REGULATOR_LOG

    // Different behavior for rotation control:
    // int pwm = (int)fabsf(output);

    // send_motors_data(pwm, (error > 0) ? POSITIVE : NEGATIVE);

    //  trigger_motors_update();
    reschedule_work(&rotation_regulator_work, K_MSEC(CONFIG_REGULATOR_SAMPLE_TIME * 10), "rotation control");
}

static K_WORK_DELAYABLE_DEFINE(rotation_regulator_work, rotation_regulator_work_handler);

void
regulator_start_automatic_control(void)
{
    if(g_automatic_control_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator workers already started");
#endif  // CONFIG_REGULATOR_LOG
    }
    g_automatic_control_started = true;

    reschedule_work(&balance_regulator_work, K_NO_WAIT, "balance control");
    // reschedule_work(&rotation_regulator_work, K_NO_WAIT, "rotation control");
}

void
regulator_stop_automatic_control(void)
{
    if(!g_automatic_control_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator workers not started");
#endif  // CONFIG_REGULATOR_LOG
    }
    g_automatic_control_started = false;

    set_start_motors(false);

    int ret = k_work_cancel_delayable(&balance_regulator_work);
    ret     = k_work_cancel_delayable(&rotation_regulator_work);
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
new_calculate_balance_regulator_output_cb_register(calculate_regulator_output_cb_t new_calculate_regulator_output_cb)
{
    if(new_calculate_regulator_output_cb)
    {
        g_balance_regulator_context.calculate_regulator_output_cb = new_calculate_regulator_output_cb;
    }
}

void
new_calculate_rotation_regulator_output_cb_register(calculate_regulator_output_cb_t new_calculate_regulator_output_cb)
{
    if(new_calculate_regulator_output_cb)
    {
        g_rotation_regulator_context.calculate_regulator_output_cb = new_calculate_regulator_output_cb;
    }
}

void
new_get_balance_setpoint_cb_register(get_setpoint_cb_t new_get_setpoint_cb)
{
    if(new_get_setpoint_cb)
    {
        g_balance_regulator_context.get_setpoint_cb = new_get_setpoint_cb;
    }
}

void
new_get_rotation_setpoint_cb_register(get_setpoint_cb_t new_get_setpoint_cb)
{
    if(new_get_setpoint_cb)
    {
        g_rotation_regulator_context.get_setpoint_cb = new_get_setpoint_cb;
    }
}