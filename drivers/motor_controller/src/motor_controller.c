#include "motor_controller.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include "utils.h"

#define APPLICATION_INIT_PRIORITY 99
#define INTERRUPT_INTERVAL 20  // [ms]
#define N_GPIO_PINS 5
#define DIRECTION_CONTROL_PINS_BEGIN_IDX 1
#define DIRECTION_CONTROL_PINS_END_IDX 4

LOG_MODULE_REGISTER(motor_controller, CONFIG_BAT_LVL_LOG_LEVEL);

static bool controller_enabled              = false;
static bool periodic_motors_control_started = false;
static MOTORS_DATA motors_data              = {.direction = POSITIVE, .pwm_value = 0, .start = false};
static struct k_work_delayable motor_controller_work;

static const struct gpio_dt_spec H_drive_en = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(H_drive_en_pin), gpios, {0});
static const struct gpio_dt_spec A1_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(A_in_pins), gpios, 0, {0});
static const struct gpio_dt_spec A2_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(A_in_pins), gpios, 1, {0});
static const struct gpio_dt_spec B1_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(B_in_pins), gpios, 0, {0});
static const struct gpio_dt_spec B2_in      = GPIO_DT_SPEC_GET_BY_IDX_OR(DT_NODELABEL(B_in_pins), gpios, 1, {0});

static const struct gpio_dt_spec* gpio_pins[] = {&H_drive_en, &A1_in, &A2_in, &B1_in, &B2_in};

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
configure_gpio_pin(struct gpio_dt_spec* gpio_pin)
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

    // Config PWM outputs:

    return ret;
}

SYS_INIT(init, APPLICATION, APPLICATION_INIT_PRIORITY);

static void
stop_motors(void)
{
    int ret = 0;

    for(uint8_t i = DIRECTION_CONTROL_PINS_BEGIN_IDX; i < DIRECTION_CONTROL_PINS_BEGIN_IDX + 1; i++)
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

    for(uint8_t i = DIRECTION_CONTROL_PINS_BEGIN_IDX; i < DIRECTION_CONTROL_PINS_BEGIN_IDX + 1; i++)
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

    run_motors_in_direction(motors_data.direction);

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