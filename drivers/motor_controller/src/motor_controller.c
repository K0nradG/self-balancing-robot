#pragma GCC diagnostic ignored "-Wunused-variable"

#include "motor_controller.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include "utils.h"

#define APPLICATION_INIT_PRIORITY 99
#define INTERRUPT_INTERVAL 20  // [ms]
#define N_GPIO_PINS 5
#define DIRECTION_CONTROL_PINS_BEGIN_IDX 1

LOG_MODULE_REGISTER(motor_controller, CONFIG_BAT_LVL_LOG_LEVEL);

static bool controller_enabled              = false;
static bool periodic_motors_control_started = false;
static MOTORS_DATA motors_data              = {.direction = POSITIVE, .duty_cycle_f = 0, .start = false};
static struct k_work_delayable motor_controller_work;

static const struct gpio_dt_spec H_drive_en = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(H_drive_en_pin), gpios, {0});
static const struct gpio_dt_spec A1_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(A_in_pins), gpios, 0, {0});
static const struct gpio_dt_spec A2_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(A_in_pins), gpios, 1, {0});
static const struct gpio_dt_spec B1_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(B_in_pins), gpios, 0, {0});
static const struct gpio_dt_spec B2_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(B_in_pins), gpios, 1, {0});

static const struct gpio_dt_spec* gpio_pins[] = {&H_drive_en, &A1_in, &A2_in, &B1_in, &B2_in};

static const struct device* pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));

void
set_enable_controller(bool enable)
{
    controller_enabled = enable;
}

void
set_start_motors(bool start)
{
    motors_data.start = start;
}

void
set_direction(DIRECTION direction)
{
    motors_data.direction = direction;
}

void
set_duty_cycle_value(float duty_cycle_f)
{
    if(duty_cycle_f > 1.0f)
    {
        duty_cycle_f = 1.0f;
    }

    if(duty_cycle_f < 0.0f)
    {
        duty_cycle_f = 0.0f;
    }

    motors_data.duty_cycle_f = duty_cycle_f;
}

static int
configure_gpio_pin(const struct gpio_dt_spec* gpio_pin)
{
    int ret = 0;

    if(gpio_pin->port)
    {
        ret = gpio_pin_configure_dt(gpio_pin, GPIO_OUTPUT_INACTIVE);
    }
    else
    {
        __ASSERT(false, "Invalid GPIO: port is NULL!");
    }

    return ret;
}

static int
init(void)
{
    int ret = 0;

    for(uint8_t i = 0; i < N_GPIO_PINS; i++)
    {
        ret = configure_gpio_pin(gpio_pins[i]);

        bool const gpio_ready = gpio_is_ready_dt(gpio_pins[i]);
        __ASSERT(gpio_ready, "GPIO at index %d not ready!", i);
    }

    bool const is_pwm_device_ready = device_is_ready(pwm_dev);
    __ASSERT(is_pwm_device_ready, "PWM device not ready!");

    // Configuring PWM for all used channel:
    ret = pwm_set(pwm_dev, 17, CONFIG_PWM_PERIOD_NS, 0, PWM_POLARITY_NORMAL);  // Initial duty cycle set to 0.
    ret = pwm_set(pwm_dev, 20, CONFIG_PWM_PERIOD_NS, 0, PWM_POLARITY_NORMAL);

    return ret;
}

SYS_INIT(init, APPLICATION, APPLICATION_INIT_PRIORITY);

static void
stop_motors(void)
{
    int ret = 0;

    for(uint8_t i = DIRECTION_CONTROL_PINS_BEGIN_IDX; i < N_GPIO_PINS; i++)
    {
        ret = gpio_pin_set_dt(gpio_pins[i], 0);
        __ASSERT(!ret, "GPIO at index %d not cleared during stopping the motors!", i);
    }
}

static void
disable_controller(void)
{
    int ret = 0;

    ret = gpio_pin_set_dt(&H_drive_en, 0);
    __ASSERT(!ret, "H_drive_en not cleared!");

    stop_motors();
}

static void
enable_controller(void)
{
    int ret = 0;

    ret = gpio_pin_set_dt(&H_drive_en, 1);
    __ASSERT(!ret, "H_drive_en not set!");
}

static void
run_motors_in_direction(DIRECTION direction)
{
    /*
    Positive direction -> A1 and B1 - set, A2 and B2 - cleared.
    Negative direction -> A2 and B2 - set, A1 and B1 - cleared.
    */

    uint8_t first_direction_pin_set_value = 0;

    if(direction == POSITIVE)
    {
        first_direction_pin_set_value = 1;
    }

    int ret = 0;

    for(uint8_t i = DIRECTION_CONTROL_PINS_BEGIN_IDX; i < N_GPIO_PINS; i++)
    {
        if(i % 2 != 0)
        {
            ret = gpio_pin_set_dt(gpio_pins[i], first_direction_pin_set_value);
        }
        else
        {
            ret = gpio_pin_set_dt(gpio_pins[i], !first_direction_pin_set_value);
        }

        __ASSERT(!ret, "GPIO at index %d not set during spin direction change!", i);
    }
}

static void
set_new_duty_cycle_value(float duty_cycle_f)
{
    int ret                = 0;
    uint32_t duty_cycle_ns = 0u;

    duty_cycle_ns = (uint32_t)((float)CONFIG_PWM_PERIOD_NS * duty_cycle_f);
    ret           = pwm_set(pwm_dev, 17, CONFIG_PWM_PERIOD_NS, duty_cycle_ns, PWM_POLARITY_NORMAL);
    ret           = pwm_set(pwm_dev, 20, CONFIG_PWM_PERIOD_NS, duty_cycle_ns, PWM_POLARITY_NORMAL);
    __ASSERT(!ret, "New duty cycle value not set!");
}

// First check start flag - stop the motors if false.
// Update motors direction and speed.
static void
update_motors_control(void)
{
    if(!motors_data.start)
    {
        stop_motors();
        return;
    }

    run_motors_in_direction(motors_data.direction);
    set_new_duty_cycle_value(motors_data.duty_cycle_f);
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
        update_motors_control();
    }

    reschedule_work(&motor_controller_work, K_MSEC(INTERRUPT_INTERVAL), "Motor control");
}

static K_WORK_DELAYABLE_DEFINE(motor_controller_work, motor_controller_work_handler);

void
motor_controller_start(void)
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

    int const ret = k_work_cancel_delayable(&motor_controller_work);
    if(ret)
    {
        LOG_ERR("Can't cancel delayable work: %d", ret);
        return;
    }

    LOG_DBG("Motors control work cancelled");
}