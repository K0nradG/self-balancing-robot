#include "control_loop.h"
#include "motor_controller.h"
#include "robot_controller.h"
#include "utils.h"

static bool s_control_loop_started = true;
static Robot_Controller s_robot_controller {};

static void
control_loop_work_handler(struct k_work* work);

static K_WORK_DELAYABLE_DEFINE(s_control_work, control_loop_work_handler);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
send_identification_data_cb_t g_send_identification_data_cb = NULL;
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

static void
control_loop_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    s_robot_controller.control_motors();

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    if(g_send_identification_data_cb)
    {
        g_send_identification_data_cb(s_robot_controller.get_identification_data());
    }
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

    reschedule_work(&s_control_work, K_MSEC(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME), (char*)"balance control");
}

void
start_control_loop(void)
{
    if(s_control_loop_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator workers already started");
#endif  // CONFIG_REGULATOR_LOG
    }
    s_control_loop_started = true;

    reschedule_work(&s_control_work, K_NO_WAIT, (char*)"balance control");
}

void
stop_control_loop(void)
{
    if(!s_control_loop_started)
    {
#ifdef CONFIG_REGULATOR_LOG
        platform_log("REGULATOR", LOG_LEVEL_ERR, "regulator workers not started");
#endif  // CONFIG_REGULATOR_LOG
    }
    s_control_loop_started = false;

    set_start_motors(false);

    int ret = k_work_cancel_delayable(&s_control_work);
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

#ifdef CONFIG_LOG_OVER_BLE
void
nus_data_parse_callback(char const* data)
{
    s_robot_controller.parse_nus_data(data);
}
#endif  // CONFIG_LOG_OVER_BLE

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