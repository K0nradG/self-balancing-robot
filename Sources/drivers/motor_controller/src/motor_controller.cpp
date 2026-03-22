// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "motor_controller.h"
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/init.h>
#include "logger.h"
#include "zephyr/kernel.h"

#define N_GPIO_PINS                      4u
#define DIRECTION_CONTROL_PINS_BEGIN_IDX 1u
#define PWM_PERIOD_NS                    PWM_USEC(CONFIG_PWM_PERIOD_US)

static Logger<IS_ENABLED(CONFIG_MOTOR_CONTROLLER_LOG)> motor_controller_logger("MOTOR_CONTROLLER");

static bool g_controller_enabled = false;
static MOTORS_DATA g_motors_data = {.duty_cycle_percent_motor0 = 0u, .duty_cycle_percent_motor1 = 0u, .start = false};

static gpio_dt_spec a_in1 = GPIO_DT_SPEC_GET(DT_NODELABEL(a_in1), gpios);
static gpio_dt_spec a_in2 = GPIO_DT_SPEC_GET(DT_NODELABEL(a_in2), gpios);
static gpio_dt_spec b_in1 = GPIO_DT_SPEC_GET(DT_NODELABEL(b_in1), gpios);
static gpio_dt_spec b_in2 = GPIO_DT_SPEC_GET(DT_NODELABEL(b_in2), gpios);

static const gpio_dt_spec* gpio_pins[] = {&a_in1, &a_in2, &b_in1, &b_in2};

static const pwm_dt_spec pwm_dc_1 = PWM_DT_SPEC_GET(DT_NODELABEL(dc_1));
static const pwm_dt_spec pwm_dc_2 = PWM_DT_SPEC_GET(DT_NODELABEL(dc_2));

static void
motor_controller_work_handler(k_work* work);

static K_WORK_DELAYABLE_DEFINE(motor_controller_work, motor_controller_work_handler);

void
set_enable_controller(bool controller_enabled)
{
    g_controller_enabled = controller_enabled;
}

void
set_start_motors(bool start)
{
    g_motors_data.start = start;
}

void
set_duty_cycle_value(int8_t duty_cycle_percent_motor0, int8_t duty_cycle_percent_motor1)
{
    if(duty_cycle_percent_motor0 > CONFIG_PWM_LIMIT)
    {
        duty_cycle_percent_motor0 = CONFIG_PWM_LIMIT;
    }

    if(duty_cycle_percent_motor0 < -CONFIG_PWM_LIMIT)
    {
        duty_cycle_percent_motor0 = -CONFIG_PWM_LIMIT;
    }

    if(duty_cycle_percent_motor1 > CONFIG_PWM_LIMIT)
    {
        duty_cycle_percent_motor1 = CONFIG_PWM_LIMIT;
    }

    if(duty_cycle_percent_motor1 < -CONFIG_PWM_LIMIT)
    {
        duty_cycle_percent_motor1 = -CONFIG_PWM_LIMIT;
    }

    g_motors_data.duty_cycle_percent_motor0 = duty_cycle_percent_motor0;

    g_motors_data.duty_cycle_percent_motor1 = duty_cycle_percent_motor1;
}

int
motor_controller_init()
{
    if(!device_is_ready(a_in1.port) || !device_is_ready(a_in2.port) || !device_is_ready(b_in1.port) ||
       !device_is_ready(b_in2.port) || !pwm_is_ready_dt(&pwm_dc_2))
    {
        motor_controller_logger.platform_log(LOG_LEVEL::ERR, "motors pwm not ready");
        return -ENODEV;
    }

    int ret = 0;
    ret |= gpio_pin_configure_dt(&a_in1, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&a_in2, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&b_in1, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&b_in2, GPIO_OUTPUT_ACTIVE);

    if(ret != 0)
    {
        motor_controller_logger.platform_log(LOG_LEVEL::ERR, "motors pins not ready");
        return ret;
    }

    motor_controller_logger.platform_log(LOG_LEVEL::INF, "motors init finished");
    return ret;
}

void
stop_motors()
{
    for(uint8_t i = DIRECTION_CONTROL_PINS_BEGIN_IDX; i < N_GPIO_PINS; i++)
    {
        int const ret = gpio_pin_set_dt(gpio_pins[i], 0);

        if(ret != 0)
        {
            motor_controller_logger.platform_log(LOG_LEVEL::ERR, "stop motors failed");
        }
    }
}

static void
disable_controller()
{
    stop_motors();
}

static void
run_backward_motor(gpio_dt_spec* in1, gpio_dt_spec* in2)
{
    int err = 0;
    err |= gpio_pin_set_dt(in1, 0);
    err |= gpio_pin_set_dt(in2, 1);

    if(err != 0)
    {
        motor_controller_logger.platform_log(LOG_LEVEL::ERR, "Running motors backward failed");
    }
}

static void
run_forward_motor(gpio_dt_spec* in1, gpio_dt_spec* in2)
{
    int err = 0;
    err |= gpio_pin_set_dt(in1, 1);
    err |= gpio_pin_set_dt(in2, 0);

    if(err != 0)
    {
        motor_controller_logger.platform_log(LOG_LEVEL::ERR, "Running motors forward failed");
    }
}

static void
set_new_duty_cycle_value(int8_t duty_cycle_percent_motor0, int8_t duty_cycle_percent_motor1)
{
    if(duty_cycle_percent_motor0 < 0)
    {
        run_backward_motor(&a_in1, &a_in2);
    }
    if(duty_cycle_percent_motor0 >= 0)
    {
        run_forward_motor(&a_in1, &a_in2);
    }

    if(duty_cycle_percent_motor1 < 0)
    {
        run_backward_motor(&b_in1, &b_in2);
    }
    if(duty_cycle_percent_motor1 >= 0)
    {
        run_forward_motor(&b_in1, &b_in2);
    }

    uint8_t const duty_cycle_percent_motor0_scaled = (uint8_t)abs(duty_cycle_percent_motor0);
    uint8_t const duty_cycle_percent_motor1_scaled = (uint8_t)abs(duty_cycle_percent_motor1);

    uint32_t const duty_cycle_ns_motor0 =
        (PWM_PERIOD_NS * duty_cycle_percent_motor0_scaled) / (uint32_t)CONFIG_PWM_LIMIT;
    uint32_t const duty_cycle_ns_motor1 =
        (PWM_PERIOD_NS * duty_cycle_percent_motor1_scaled) / (uint32_t)CONFIG_PWM_LIMIT;

    int err = pwm_set_dt(&pwm_dc_1, PWM_PERIOD_NS, duty_cycle_ns_motor0);
    err     = pwm_set_dt(&pwm_dc_2, PWM_PERIOD_NS, duty_cycle_ns_motor1);

    if(err != 0)
    {
        motor_controller_logger.platform_log(LOG_LEVEL::ERR, "set pwm failed");
    }
}

// First check start flag - stop the motors if false.
// Update motors direction and speed.
static void
update_motors_control()
{
    if(!g_motors_data.start)
    {
        stop_motors();
    }
    else
    {
        set_new_duty_cycle_value(g_motors_data.duty_cycle_percent_motor0, g_motors_data.duty_cycle_percent_motor1);
    }
}

static void
motor_controller_work_handler(k_work* work)
{
    ARG_UNUSED(work);

    if(!g_controller_enabled)
    {
        disable_controller();
    }
    else
    {
        update_motors_control();
    }
}

void
trigger_motors_update()
{
    int const err = k_work_submit(&motor_controller_work.work);

    if(err != 0)
    {
        motor_controller_logger.platform_log(LOG_LEVEL::ERR, "Motors update trigger failed");
    }
}