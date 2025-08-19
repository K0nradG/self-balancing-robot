#include "control_loop.h"
#include "drivers_initializer.h"
#include "interface.h"
#include "motor_controller.h"
#include "robot_controller.h"
#include "state_machine.h"
#include "zephyr/kernel.h"

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

#else
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec button      = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback g_button_data_cb = {{0}};

static int
button_init();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_BLUETOOTH_DRV
void
nus_data_parse_callback(char const* data)
{
    s_robot_controller.parse_nus_data(data);
}
#endif  // CONFIG_BLUETOOTH_DRV

static int
init(void)
{
    Drivers_Initializer::init();

#ifdef CONFIG_BLUETOOTH_DRV
    new_regulator_parameters_parser_cb_register(&nus_data_parse_callback);
#endif  // CONFIG_BLUETOOTH_DRV

#ifndef CONFIG_MODEL_IDENTIFICATION_DRV
    int const error = button_init();

    if(error != 0)
    {
#ifdef CONFIG_ROBOT_CONTROL_LOG
        platform_log("ROBOT_CONTROL", LOG_LEVEL_ERR, "Button init failed ");
#endif  // CONFIG_ROBOT_CONTROL_LOG
        return -1;
    }
#endif  // not CONFIG_MODEL_IDENTIFICATION_DRV

    set_enable_controller(true);

#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log("ROBOT_CONTROL", LOG_LEVEL_INF, "Robot control init finished");
#endif  // CONFIG_ROBOT_CONTROL_LOG
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void
control_loop_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

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
            s_robot_controller.reset_pids();
            break;
        default:
            break;
    }
    s_state_machine.update();

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
#else

void
button_pressed(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    (void)dev;
    (void)cb;
    (void)pins;

    int32_t const current_time_ms             = k_uptime_get_32();
    static int32_t last_time_ms               = 0;
    static constexpr int32_t debounce_time_ms = 100;

    if((current_time_ms - last_time_ms) > debounce_time_ms)
    {
        s_state_machine.set_button_pressed(true);
        last_time_ms = current_time_ms;
    }
}

static int
button_init()
{
    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if(ret != 0)
    {
#ifdef CONFIG_ROBOT_CONTROL_LOG
        platform_log("ROBOT_CONTROL", LOG_LEVEL_ERR, "GPIO pin configuration failed, err: %d", ret);
#endif  // CONFIG_ROBOT_CONTROL_LOG
        return ret;
    }

    ret |= gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if(ret != 0)
    {
#ifdef CONFIG_ROBOT_CONTROL_LOG
        platform_log("ROBOT_CONTROL", LOG_LEVEL_ERR, "GPIO pin interrupt configuration failed, err: %d", ret);
#endif  // CONFIG_ROBOT_CONTROL_LOG
        return ret;
    }

    gpio_init_callback(&g_button_data_cb, button_pressed, BIT(button.pin));
    ret |= gpio_add_callback(button.port, &g_button_data_cb);

    if(ret != 0)
    {
#ifdef CONFIG_ROBOT_CONTROL_LOG
        platform_log("ROBOT_CONTROL", LOG_LEVEL_ERR, "GPIO callback add failed, err: %d", ret);
#endif  // CONFIG_ROBOT_CONTROL_LOG
        return ret;
    }
    return ret;
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV