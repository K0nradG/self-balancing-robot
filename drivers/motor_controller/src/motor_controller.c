#pragma GCC diagnostic ignored "-Wunused-variable"

#include "motor_controller.h"
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/init.h>
#include "zephyr/kernel.h"

#ifdef CONFIG_MOTOR_CONTROLLER_LOG
#include "logger.h"
#endif  // CONFIG_MOTOR_CONTROLLER_LOG

#define N_GPIO_PINS                      5u
#define DIRECTION_CONTROL_PINS_BEGIN_IDX 1u
#define PWM_PERIOD_NS                    PWM_USEC(CONFIG_PWM_PERIOD_US)

static bool g_controller_enabled = false;
static MOTORS_DATA g_motors_data = {.duty_cycle_percent_motor0 = 0u, .duty_cycle_percent_motor1 = 0u, .start = false};
static struct k_work_delayable motor_controller_work;

static struct gpio_dt_spec a_in1  = GPIO_DT_SPEC_GET(DT_NODELABEL(a_in1), gpios);
static struct gpio_dt_spec a_in2  = GPIO_DT_SPEC_GET(DT_NODELABEL(a_in2), gpios);
static struct gpio_dt_spec b_in1  = GPIO_DT_SPEC_GET(DT_NODELABEL(b_in1), gpios);
static struct gpio_dt_spec b_in2  = GPIO_DT_SPEC_GET(DT_NODELABEL(b_in2), gpios);
static struct gpio_dt_spec h_b_en = GPIO_DT_SPEC_GET(DT_NODELABEL(h_b_en), gpios);

static const struct gpio_dt_spec* gpio_pins[] = {&h_b_en, &a_in1, &a_in2, &b_in1, &b_in2};

static const struct pwm_dt_spec pwm_dc_1 = PWM_DT_SPEC_GET(DT_NODELABEL(dc_1));
static const struct pwm_dt_spec pwm_dc_2 = PWM_DT_SPEC_GET(DT_NODELABEL(dc_2));

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

static int
init(void)
{
    if(!device_is_ready(a_in1.port) || !device_is_ready(a_in2.port) || !device_is_ready(b_in1.port) ||
       !device_is_ready(b_in2.port) || !device_is_ready(h_b_en.port) || !pwm_is_ready_dt(&pwm_dc_2))
    {
#ifdef CONFIG_MOTOR_CONTROLLER_LOG
        platform_log("MOTOR_CONTROLLER", LOG_LEVEL_ERR, "motors pwm not ready");
#endif  // CONFIG_MOTOR_CONTROLLER_LOG
        return -ENODEV;
    }

    int ret = 0;
    ret |= gpio_pin_configure_dt(&a_in1, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&a_in2, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&b_in1, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&b_in2, GPIO_OUTPUT_ACTIVE);
    ret |= gpio_pin_configure_dt(&h_b_en, GPIO_OUTPUT_ACTIVE);

#ifdef CONFIG_MOTOR_CONTROLLER_LOG
    if(ret)
    {
        platform_log("MOTOR_CONTROLLER", LOG_LEVEL_ERR, "motors pins not ready");
        return ret;
    }
    platform_log("MOTOR_CONTROLLER", LOG_LEVEL_INF, "motors init finished");
#endif  // CONFIG_MOTOR_CONTROLLER_LOG
    return ret;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void
stop_motors(void)
{
    for(uint8_t i = DIRECTION_CONTROL_PINS_BEGIN_IDX; i < N_GPIO_PINS; i++)
    {
        int const ret = gpio_pin_set_dt(gpio_pins[i], 0);
#ifdef CONFIG_MOTOR_CONTROLLER_LOG
        if(ret)
        {
            platform_log("MOTOR_CONTROLLER", LOG_LEVEL_ERR, "stop motors failed");
        }
#endif  // CONFIG_MOTOR_CONTROLLER_LOG
    }
}

static void
disable_controller(void)
{
    int const ret = gpio_pin_set_dt(&h_b_en, 0);
#ifdef CONFIG_MOTOR_CONTROLLER_LOG
    if(ret)
    {
        platform_log("MOTOR_CONTROLLER", LOG_LEVEL_ERR, "stop controller failed");
    }
#endif  // CONFIG_MOTOR_CONTROLLER_LOG
    stop_motors();
}

static void
enable_controller(void)
{
    int const ret = gpio_pin_set_dt(&h_b_en, 1);
#ifdef CONFIG_MOTOR_CONTROLLER_LOG
    if(ret)
    {
        platform_log("MOTOR_CONTROLLER", LOG_LEVEL_ERR, "enable controller failed");
    }
#endif  // CONFIG_MOTOR_CONTROLLER_LOG
}

static void
run_backward_motor(struct gpio_dt_spec* in1, struct gpio_dt_spec* in2)
{
    gpio_pin_set_dt(in1, 0);
    gpio_pin_set_dt(in2, 1);
}

static void
run_forward_motor(struct gpio_dt_spec* in1, struct gpio_dt_spec* in2)
{
    gpio_pin_set_dt(in1, 1);
    gpio_pin_set_dt(in2, 0);
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

    uint8_t duty_cycle_percent_motor0_scaled = (int)abs(duty_cycle_percent_motor0);
    uint8_t duty_cycle_percent_motor1_scaled = (int)abs(duty_cycle_percent_motor1);

    uint32_t duty_cycle_ns_motor0 = (PWM_PERIOD_NS * duty_cycle_percent_motor0_scaled) / CONFIG_PWM_LIMIT;
    uint32_t duty_cycle_ns_motor1 = (PWM_PERIOD_NS * duty_cycle_percent_motor1_scaled) / CONFIG_PWM_LIMIT;

    int err = pwm_set_dt(&pwm_dc_1, PWM_PERIOD_NS, duty_cycle_ns_motor0);
    err     = pwm_set_dt(&pwm_dc_2, PWM_PERIOD_NS, duty_cycle_ns_motor1);

#ifdef CONFIG_MOTOR_CONTROLLER_LOG
    if(err)
    {
        platform_log("MOTOR_CONTROLLER", LOG_LEVEL_ERR, "set pwm failed");
    }
#endif  // CONFIG_MOTOR_CONTROLLER_LOG
}

// First check start flag - stop the motors if false.
// Update motors direction and speed.
static void
update_motors_control(void)
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
motor_controller_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    if(!g_controller_enabled)
    {
        disable_controller();
    }
    else
    {
        enable_controller();
        update_motors_control();
    }
}

static K_WORK_DELAYABLE_DEFINE(motor_controller_work, motor_controller_work_handler);

void
trigger_motors_update(void)
{
    k_work_submit(&motor_controller_work.work);
}