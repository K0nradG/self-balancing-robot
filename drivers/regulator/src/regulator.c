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
static float g_angle_balance            = 0.0f;
static float g_angle_balance_dt         = 0.0f;
static float g_angle_rotation           = 0.0f;

static struct k_work_delayable regulator_work;
calculate_regulator_output_cb_t g_new_calculate_regulator_output_cb = NULL;
get_setpoint_cb_t g_new_get_setpoint_cb                             = NULL;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"

send_identification_data_cb_t g_send_identification_data_cb = NULL;
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

void
new_imu_data_for_regulator(imu_data imu_data)
{
    g_angle_balance    = imu_data.angle_balance;
    g_angle_balance_dt = imu_data.angle_balance_dt;
    g_angle_rotation   = imu_data.angle_rotation;
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
regulator_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);
    float error = 0.0f - input_low_pass_filter(g_angle_balance);

    if(g_new_get_setpoint_cb)
    {
        error = g_new_get_setpoint_cb() * DEG_TO_RAD - (g_angle_balance);
    }

    float output = 0.0f;
    if(g_new_calculate_regulator_output_cb)
    {
#ifdef CONFIG_PID_ENABLED
        output = g_new_calculate_regulator_output_cb(error);
#else
        output = g_new_calculate_regulator_output_cb(g_angle_balance, g_angle_balance_dt);
#endif  // CONFIG_PID_ENABLED
    }

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV

    struct identification_data data = {
        .dt = k_uptime_get(), .pwm = output, .angle = g_angle_balance, g_angle_balance_dt = g_angle_balance_dt};

    if(g_send_identification_data_cb)
    {
        g_send_identification_data_cb(data);
    }
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_REGULATOR_LOG
    // platform_log("REGULATOR", LOG_LEVEL_ERR, "error %f out: %f", (double)error, (double)output);
    platform_log("REGULATOR", LOG_LEVEL_ERR, "rot: %f [deg]", (double)g_angle_rotation * (double)RAD_TO_DEG);
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
    if(g_automatic_control_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator worker already started");
#endif  // CONFIG_REGULATOR_LOG
    }
    g_automatic_control_started = true;

    reschedule_work(&regulator_work, K_NO_WAIT, "automatic control");
}

void
regulator_stop_automatic_control(void)
{
    if(!g_automatic_control_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator worker not started");
#endif  // CONFIG_REGULATOR_LOG
    }
    g_automatic_control_started = false;

    set_start_motors(false);

    int const ret = k_work_cancel_delayable(&regulator_work);
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
new_calculate_regulator_output_cb_register(calculate_regulator_output_cb_t new_calculate_regulator_output_cb)
{
    if(new_calculate_regulator_output_cb)
    {
        g_new_calculate_regulator_output_cb = new_calculate_regulator_output_cb;
    }
}

void
new_get_setpoint_cb_register(get_setpoint_cb_t new_get_setpoint_cb)
{
    if(new_get_setpoint_cb)
    {
        g_new_get_setpoint_cb = new_get_setpoint_cb;
    }
}