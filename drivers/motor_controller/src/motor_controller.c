#include "motor_controller.h"
#include <hal/nrf_gpio.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include "utils.h"

#define APPLICATION_INIT_PRIORITY 99
#define INTERRUPT_INTERVAL 20  // [ms]
#define H_DRIVE_EN NRF_GPIO_PIN_MAP(1, 10)
#define A1_IN NRF_GPIO_PIN_MAP(0, 22)
#define A2_IN NRF_GPIO_PIN_MAP(0, 24)
#define B1_IN NRF_GPIO_PIN_MAP(1, 15)
#define B2_IN NRF_GPIO_PIN_MAP(0, 02)

LOG_MODULE_REGISTER(motor_controller, CONFIG_BAT_LVL_LOG_LEVEL);

static bool controller_enabled              = false;
static bool periodic_motors_control_started = false;
static MOTORS_DATA motors_data              = {.direction = POSITIVE, .pwm_value = 0, .start = false};
static struct k_work_delayable motor_controller_work;

static void
set_enable_controller(bool enable)
{
    controller_enabled = enable;
}

static void
set_start_motors(bool start)
{
    motors_data.start = start;
}

static void
set_direction(DIRECTION direction)
{
    motors_data.direction = direction;
}

static void
set_pwm_value(int pwm_value)
{
    // Some input control could be needed here.
    motors_data.pwm_value = pwm_value;
}

static int
init(void)
{
    nrf_gpio_cfg_output(H_DRIVE_EN);
    nrf_gpio_cfg_output(A1_IN);
    nrf_gpio_cfg_output(A2_IN);
    nrf_gpio_cfg_output(B1_IN);
    nrf_gpio_cfg_output(B2_IN);

    // Config PWM outputs:

    return 0;
}

SYS_INIT(init, APPLICATION, APPLICATION_INIT_PRIORITY);

static void
stop_motors(void)
{
    nrf_gpio_pin_clear(A1_IN);
    nrf_gpio_pin_clear(A2_IN);
    nrf_gpio_pin_clear(B1_IN);
    nrf_gpio_pin_clear(B2_IN);
}

static void
disable_controller(void)
{
    nrf_gpio_pin_clear(H_DRIVE_EN);
    stop_motors();
}

static void
enable_controller(void)
{
    nrf_gpio_pin_set(H_DRIVE_EN);
}

static void
run_motors_in_positive_direction(void)
{
    // Positive direction -> A1 and B1 - set, A2 and B2 - cleared.
    nrf_gpio_pin_set(A1_IN);
    nrf_gpio_pin_clear(A2_IN);
    nrf_gpio_pin_set(B1_IN);
    nrf_gpio_pin_clear(B2_IN);
}

static void
run_motors_in_negative_direction(void)
{
    // Negative direction -> A2 and B2 - set, A1 and B1 - cleared.
    nrf_gpio_pin_clear(A1_IN);
    nrf_gpio_pin_set(A2_IN);
    nrf_gpio_pin_clear(B1_IN);
    nrf_gpio_pin_set(B2_IN);
}

// First check start flag - stop the motors if false.
// Update motors direction and speed.
static int
update_motors_control(void)
{
    if(!motors_data.start)
    {
        stop_motors();
        return;
    }

    if(motors_data.direction == POSITIVE)
    {
        run_motors_in_positive_direction();
    }
    else
    {
        run_motors_in_negative_direction();
    }

    // Set PWM value:

    return 0;  // Maybe some check will need to be done for PWM
}

static void
motor_controller_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    if(!controller_enabled)
    {
        disable_controller();
    }
    else
    {
        enable_controller();

        int const ret = update_motors_control();
        if(ret)
        {
            LOG_ERR("Can't control the motors: %d", ret);
        }
    }

    reschedule_work(&motor_controller_work, K_MSEC(INTERRUPT_INTERVAL), "Motor control");
}

static K_WORK_DELAYABLE_DEFINE(motor_controller_work, motor_controller_work_handler);

void
motor_controller_start(uint16_t interval_ms)
{
    __ASSERT(periodic_motors_control_started, "Periodic motor control already started");
    periodic_motors_control_started = true;

    reschedule_work(&motor_controller_work, K_NO_WAIT, "motor_control");
}

void
motor_controller_stop(void)
{
    __ASSERT(!periodic_motors_control_started, "Periodic measurement is not started");
    periodic_motors_control_started = false;

    const int ret = k_work_cancel_delayable(&motor_controller_work);
    if(ret)
    {
        LOG_ERR("Can't cancel delayable work: %d", ret);
        return;
    }

    LOG_DBG("Motors control work cancelled");
}