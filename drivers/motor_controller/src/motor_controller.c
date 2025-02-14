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
static MOTORS_DATA motors_data              = {.direction = POSITIVE, .duty_cycle_percent = 0, .start = false};
static struct k_work_delayable motor_controller_work;

static struct gpio_dt_spec a_in1  = GPIO_DT_SPEC_GET(DT_NODELABEL(a_in1), gpios);
static struct gpio_dt_spec a_in2  = GPIO_DT_SPEC_GET(DT_NODELABEL(a_in2), gpios);
static struct gpio_dt_spec b_in1  = GPIO_DT_SPEC_GET(DT_NODELABEL(b_in1), gpios);
static struct gpio_dt_spec b_in2  = GPIO_DT_SPEC_GET(DT_NODELABEL(b_in2), gpios);
static struct gpio_dt_spec h_b_en = GPIO_DT_SPEC_GET(DT_NODELABEL(h_b_en), gpios);

static const struct gpio_dt_spec* gpio_pins[] = {&h_b_en, &a_in1, &a_in2, &b_in1, &b_in2};

// static const struct device* pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));

static const struct pwm_dt_spec pwm_dc_1 = PWM_DT_SPEC_GET(DT_NODELABEL(dc_1));
static const struct pwm_dt_spec pwm_dc_2 = PWM_DT_SPEC_GET(DT_NODELABEL(dc_2));

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
set_duty_cycle_value(uint32_t duty_cycle_percent)
{
    if(duty_cycle_percent > 100)
    {
        duty_cycle_percent = 100;
    }

    if(duty_cycle_percent < 0)
    {
        duty_cycle_percent = 0;
    }

    motors_data.duty_cycle_percent = duty_cycle_percent;
}

static int
init(void)
{
    const bool is_a_in1_ready = device_is_ready(a_in1.port);
    __ASSERT(is_a_in1_ready, "A_IN_1 device not ready");

    const bool is_a_in2_ready = device_is_ready(a_in2.port);
    __ASSERT(is_a_in2_ready, "A_IN_2 device not ready");

    const bool is_b_in1_ready = device_is_ready(b_in1.port);
    __ASSERT(is_b_in1_ready, "B_IN_1 device not ready");

    const bool is_b_in2_ready = device_is_ready(b_in2.port);
    __ASSERT(is_b_in2_ready, "B_IN_2 device not ready");

    const bool is_h_b_en_ready = device_is_ready(h_b_en.port);
    __ASSERT(is_h_b_en_ready, "H_BRIDGE_EN device not ready");

    int ret = gpio_pin_configure_dt(&a_in1, GPIO_OUTPUT_ACTIVE);
    __ASSERT(!ret, "A_IN_1 device configuration failed");

    ret = gpio_pin_configure_dt(&a_in2, GPIO_OUTPUT_ACTIVE);
    __ASSERT(!ret, "A_IN_2 device configuration failed");

    ret = gpio_pin_configure_dt(&b_in1, GPIO_OUTPUT_ACTIVE);
    __ASSERT(!ret, "B_IN_1 device configuration failed");

    ret = gpio_pin_configure_dt(&b_in2, GPIO_OUTPUT_ACTIVE);
    __ASSERT(!ret, "B_IN_2 device configuration failed");

    ret = gpio_pin_configure_dt(&h_b_en, GPIO_OUTPUT_ACTIVE);
    __ASSERT(!ret, "H_BRIDGE_EN device configuration failed");

    const bool is_pwm_dc_1_ready = pwm_is_ready_dt(&pwm_dc_1);
    __ASSERT(is_pwm_dc_1_ready, "PWM DC 1 device not ready");
    const bool is_pwm_dc_2_ready = pwm_is_ready_dt(&pwm_dc_2);
    __ASSERT(is_pwm_dc_2_ready, "PWM DC 2 device not ready");

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

    ret = gpio_pin_set_dt(&h_b_en, 0);
    __ASSERT(!ret, "H_drive_en not cleared!");

    stop_motors();
}

static void
enable_controller(void)
{
    int ret = gpio_pin_set_dt(&h_b_en, 1);
    __ASSERT(!ret, "H_drive_en not set!");
}

void
run_motors_in_direction(DIRECTION direction)
{
    /*
    Positive direction -> A1 and B1 - set, A2 and B2 - cleared.
    Negative direction -> A2 and B2 - set, A1 and B1 - cleared.
    */

    switch(direction)
    {
        case POSITIVE:
            gpio_pin_set_dt(&a_in1, 1);
            gpio_pin_set_dt(&a_in2, 0);
            gpio_pin_set_dt(&b_in1, 1);
            gpio_pin_set_dt(&b_in2, 0);
            break;

        case NEGATIVE:
            gpio_pin_set_dt(&a_in1, 0);
            gpio_pin_set_dt(&a_in2, 1);
            gpio_pin_set_dt(&b_in1, 0);
            gpio_pin_set_dt(&b_in2, 1);
            break;

        default:
            stop_motors();
            break;
    }
}

static void
set_new_duty_cycle_value(uint8_t duty_cycle_percent)
{
    uint32_t duty_cycle_ns = (CONFIG_PWM_PERIOD_NS * duty_cycle_percent) / 100;
    int err                = pwm_set_dt(&pwm_dc_1, CONFIG_PWM_PERIOD_NS, duty_cycle_ns);
    err                    = pwm_set_dt(&pwm_dc_2, CONFIG_PWM_PERIOD_NS, duty_cycle_ns);
    __ASSERT(!err, "New duty cycle value not set!");
}

// First check start flag - stop the motors if false.
// Update motors direction and speed.
static void
update_motors_control(void)
{
    if(!motors_data.start)
    {
        stop_motors();
    }
    else
    {
        run_motors_in_direction(motors_data.direction);
        set_new_duty_cycle_value(motors_data.duty_cycle_percent);
    }
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
