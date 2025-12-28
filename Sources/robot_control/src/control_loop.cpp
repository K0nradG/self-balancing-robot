#include "control_loop.h"
#include "drivers_initializer.h"
#include "interface.h"
#include "logger.h"
#include "main_state_machine.h"
#include "motor_controller.h"
#include "robot_controller.h"
#include "zephyr/kernel.h"

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
#include "watchdog_controller.h"
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

#ifdef CONFIG_BLUETOOTH_DRV
#include "ble_service.h"
#endif  // CONFIG_BLUETOOTH_DRV

static Logger<IS_ENABLED(CONFIG_ROBOT_CONTROL_LOG)> robot_control_logger("ROBOT_CONTROL");

namespace Robot_Control
{

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
send_identification_data_cb_t g_send_identification_data_cb;
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_BLUETOOTH_DRV
void
nus_data_parse_callback(char const* data)
{
    Robot_Controller::instance().parse_nus_data(data);
}

void
parse_nus_commands_callback(char const* data)
{
    Main_State_Machine::instance().parse_nus_commands(data);
}
#endif  // CONFIG_BLUETOOTH_DRV

int
control_loop_init()
{
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    new_send_identification_data_cb_register(new_regulator_data_for_identification);
    identification_process_parser_cb_register(identification_data_nus_parser_callback);
    robot_control_logger.platform_log(LOG_LEVEL::INF, "Model identification driver is enabled.");
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

    Drivers_Initializer::init();

#ifdef CONFIG_BLUETOOTH_DRV
    new_regulator_parameters_parser_cb_register(&nus_data_parse_callback);
    state_machine_commands_parser_cb_register(&parse_nus_commands_callback);
#endif  // CONFIG_BLUETOOTH_DRV

    set_enable_controller(true);

    robot_control_logger.platform_log(LOG_LEVEL::INF, "Robot control init finished");

    return 0;
}

static void
control_loop_work_handler(k_work* work)
{
    ARG_UNUSED(work);
#ifdef CONFIG_WATCHDOG_CONTROLLER_DRV
    feed_watchdog();
#endif  // CONFIG_WATCHDOG_CONTROLLER_DRV

    using State = Main_State_Machine::State;

    Main_State_Machine::instance().update();
    State const state = Main_State_Machine::instance().get_state();
    switch(state)
    {
        case State::NORMAL_OPERATION:
        {
            bool const disable_motors_command = Robot_Controller::instance().normal_motors_control();

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
            if(g_send_identification_data_cb)
            {
                g_send_identification_data_cb(Robot_Controller::instance().get_identification_data());
            }
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

            Main_State_Machine::instance().set_disable_motors_command(disable_motors_command);

            break;
        }
        case State::SOFT_STOP:
        {
            bool const motors_stopped = Robot_Controller::instance().soft_stop_motors();
            Main_State_Machine::instance().set_motors_stopped(motors_stopped);
            break;
        }
        case State::RESET_AFTER_STOP:
#ifndef CONFIG_MODEL_IDENTIFICATION_DRV
            Robot_Controller::instance().reset();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
            break;
        default:
            break;
    }
}

static K_WORK_DELAYABLE_DEFINE(s_control_work, control_loop_work_handler);

void
trigger_control_loop()
{
    k_work_submit(&s_control_work.work);
}

void
stop_control_loop()
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

}  // namespace Robot_Control