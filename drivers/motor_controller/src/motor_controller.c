#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

#include "motor_controller.h"
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include "utils.h"

#define APPLICATION_INIT_PRIORITY 99
#define INTERRUPT_INTERVAL 20  // [ms]

LOG_MODULE_REGISTER(motor_controller, CONFIG_BAT_LVL_LOG_LEVEL);

// static const struct adc_dt_spec adc_dev = ADC_DT_SPEC_GET(DT_NODELABEL(adc));
static bool controller_enabled              = false;
static bool periodic_motors_control_started = false;
static MOTORS_DATA motors_data              = {.direction = PLUS, .pwm_value = 0, .start = false};
static struct k_work_delayable motor_controller_work;

static void
enable_controller(bool enable)
{
    controller_enabled = enable;
}

static void
start_stop_motors(bool start)
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
    motors_data.pwm_value = pwm_value;
}

static int
init(void)
{
    int ret = 0;
    // Enable PWM and appropriate GPIO pins.

    // const bool is_adc_ready = adc_is_ready_dt(&adc_dev);

    // __ASSERT(is_adc_ready, "ADC controller device %s not ready", adc_dev.dev->name);

    return ret;
}

SYS_INIT(init, APPLICATION, APPLICATION_INIT_PRIORITY);

// Update motors direction and speed.
// First check start flag - stop the motors if false.
static int
update_motors_control(void)
{
    int ret = 0;

    return ret;
}

static void
motor_controller_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    int const ret = update_motors_control();
    if(ret)
    {
        LOG_ERR("Can't control the motors: %d", ret);
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