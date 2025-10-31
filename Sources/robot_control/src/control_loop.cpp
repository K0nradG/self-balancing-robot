#include "control_loop.h"
#include "drivers_initializer.h"
#include "interface.h"
#include "motor_controller.h"
#include "robot_controller.h"
#include "state_machine.h"
#include "zephyr/kernel.h"

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
#include "watchdog_controller.h"
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

#ifdef CONFIG_ROBOT_CONTROL_LOG
#include "logger.h"
#endif  // CONFIG_ROBOT_CONTROL_LOG

#ifdef CONFIG_BLUETOOTH_DRV
#include "ble_service.h"
#endif  // CONFIG_BLUETOOTH_DRV

static Robot_Control::Robot_Controller s_robot_controller {};
static Robot_Control::State_Machine s_state_machine {};

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
send_identification_data_cb_t g_send_identification_data_cb = nullptr;
s_state_machine.set_identification_state();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_BLUETOOTH_DRV
void
nus_data_parse_callback(char const* data)
{
    s_robot_controller.parse_nus_data(data);
}

void
parse_nus_commands_callback(char const* data)
{
    s_state_machine.parse_nus_commands(data);
}
#endif  // CONFIG_BLUETOOTH_DRV

int
control_loop_init(void)
{
    Drivers_Initializer::init();

#ifdef CONFIG_BLUETOOTH_DRV
    new_regulator_parameters_parser_cb_register(&nus_data_parse_callback);
    state_machine_commands_parser_cb_register(&parse_nus_commands_callback);
#endif  // CONFIG_BLUETOOTH_DRV

    set_enable_controller(true);

#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log("ROBOT_CONTROL", LOG_LEVEL_INF, "Robot control init finished");
#endif  // CONFIG_ROBOT_CONTROL_LOG
    return 0;
}

static void
control_loop_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);
#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
    feed_watchdog();
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

    s_state_machine.update();
    Robot_Control::State_Machine::State const state = s_state_machine.get_state();
    switch(state)
    {
        case Robot_Control::State_Machine::State::IDENTIFICATION:
        case Robot_Control::State_Machine::State::NORMAL_OPERATION:
        {
            bool const disable_motors_command = s_robot_controller.normal_motors_control();
            s_state_machine.set_disable_motors_command(disable_motors_command);
            break;
        }
        case Robot_Control::State_Machine::State::SOFT_STOP:
        {
            bool const motors_stopped = s_robot_controller.soft_stop_motors();
            s_state_machine.set_motors_stopped(motors_stopped);
            break;
        }
        case Robot_Control::State_Machine::State::RESET_AFTER_STOP:
            s_robot_controller.reset();
            break;
        default:
            break;
    }

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

void
stop_control_loop(void)
{
    set_enable_controller(false);
    led_stop_periodic_blinking();
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