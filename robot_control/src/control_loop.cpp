#include "control_loop.h"
#include "motor_controller.h"
#include "robot_controller.h"
#include "zephyr/kernel.h"

#ifdef CONFIG_ROBOT_CONTROL_LOG
#include "logger"
#endif  // CONFIG_ROBOT_CONTROL_LOG

#ifdef CONFIG_LOG_OVER_BLE
#include "ble_logger_service.h"
#endif

static Robot_Control::Robot_Controller s_robot_controller {};

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
send_identification_data_cb_t g_send_identification_data_cb = nullptr;
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_LOG_OVER_BLE
void
nus_data_parse_callback(char const* data)
{
    s_robot_controller.parse_nus_data(data);
}
#endif  // CONFIG_LOG_OVER_BLE

static int
init(void)
{
#ifdef CONFIG_LOG_OVER_BLE
    new_regulator_parameters_parser_cb_register(&nus_data_parse_callback);
#endif  // CONFIG_LOG_OVER_BLE

    set_enable_controller(true);

#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log("REGULATOR", LOG_LEVEL_INF, "regulator init finished");
#endif  // CONFIG_ROBOT_CONTROL_LOG
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
}

static K_WORK_DELAYABLE_DEFINE(s_control_work, control_loop_work_handler);

void
trigger_control_loop(void)
{
    k_work_submit(&s_control_work.work);
}

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