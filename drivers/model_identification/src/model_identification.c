#include "model_identification.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include "identification_data_send.h"
#include "motor_controller.h"
#include "utils.h"

#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
#include "logger.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

static struct ring_buf buffers[BUFFER_COUNT];
static uint8_t buffer_data[BUFFER_COUNT][BUFFER_SIZE * sizeof(float)];
static uint16_t buffer_index[BUFFER_COUNT] = {0};
static bool is_full[BUFFER_COUNT];

static bool model_identification_started = false;
static struct k_work_delayable model_identification_work;

typedef struct parameters
{
    float angle;
    float angle_dt;
    float duty_cycle;

} parameters;

static parameters identification_parameters = {.angle = 0.0f, .angle_dt = 0.0f, .duty_cycle = 0.0f};

void
new_imu_data_for_identification(struct identification_data data)
{
    identification_parameters.angle    = data.angle;
    identification_parameters.angle_dt = data.angle_dt;
}

void
button_pressed(const struct device* dev, struct gpio_callback* cb, uint32_t pins);

static int
init(void)
{
    if(!device_is_ready(button.port))
    {
        return -1;
    }

    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if(ret < 0)
    {
        return -1;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    for(int i = 0; i < BUFFER_COUNT; i++)
    {
        ring_buf_init(&buffers[i], BUFFER_SIZE * sizeof(float), buffer_data[i]);
        buffer_index[i] = 0;
        is_full[i]      = false;
    }

    set_enable_controller(true);
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static bool
buffer_all_full(void);

static bool
buffer_put(uint8_t buffer_id, float data);

static void
generate_control(void);

static void
model_identification_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    if(!buffer_all_full())
    {
        buffer_put(ANGLE_BUFFER_ID, identification_parameters.angle);
        buffer_put(ANGLE_DT_BUFFER_ID, identification_parameters.angle_dt);
        buffer_put(U_BUFFER_ID, identification_parameters.duty_cycle);
        buffer_put(TIME_BUFFER_ID, k_uptime_get());
    }

    generate_control();
    trigger_motors_update();

#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
    platform_log("IDENTIFICATION", LOG_LEVEL_INF, "PWM %d", (int)identification_parameters.duty_cycle);
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG
    reschedule_work(
        &model_identification_work, K_MSEC(CONFIG_MODEL_IDENTIFICATION_SAMPLE_TIME), "model identification");
}

static K_WORK_DELAYABLE_DEFINE(model_identification_work, model_identification_work_handler);

void
model_identification_start(void)
{
    if(model_identification_started)
    {
#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
        platform_log("IDENTIFICATION", LOG_LEVEL_ERR, "model identification worker already started");
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG
    }
    model_identification_started = true;

    reschedule_work(&model_identification_work, K_NO_WAIT, "model identification");
}

void
model_identification_stop(void)
{
    if(!model_identification_started)
    {
#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
        platform_log("IDENTIFICATION", LOG_LEVEL_ERR, "model identification worker not started");
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG
    }
    model_identification_started = false;

    const int ret = k_work_cancel_delayable(&model_identification_work);
    if(ret)
    {
#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
        platform_log("IDENTIFICATION", LOG_LEVEL_ERR, "cancel model identification work err:%d", ret);
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG
        return;
    }

#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
    platform_log("IDENTIFICATION", LOG_LEVEL_DBG, "model identification work cancelled");
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG

    set_start_motors(false);
    stop_motors();
}

static bool
buffer_put(uint8_t buffer_id, float data)
{
    if(buffer_id >= BUFFER_COUNT)
        return false;

    if(is_full[buffer_id])
        return false;

    int ret = ring_buf_put(&buffers[buffer_id], (uint8_t*)&data, sizeof(float));
    if(ret == sizeof(float))
    {
        buffer_index[buffer_id]++;
        if(buffer_index[buffer_id] >= BUFFER_SIZE)
        {
            is_full[buffer_id] = true;
        }
        return true;
    }
    return false;
}

uint16_t
buffer_get(uint8_t buffer_id, float* data, uint16_t max_len)
{
    if(buffer_id >= BUFFER_COUNT)
        return 0;

    uint16_t len = (buffer_index[buffer_id] < max_len) ? buffer_index[buffer_id] : max_len;
    for(uint16_t i = 0; i < len; i++)
    {
        ring_buf_get(&buffers[buffer_id], (uint8_t*)&data[i], sizeof(float));
    }

    buffer_index[buffer_id] -= len;
    return len;
}

static bool
buffer_all_full(void)
{
    for(int i = 0; i < BUFFER_COUNT; i++)
    {
        if(!is_full[i])
        {
            return false;
        }
    }
    return true;
}

void
button_pressed(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    if(buffer_all_full())
    {
        trigger_identification_data_sending();
    }
}

static void
generate_control(void)
{
    static uint16_t const cycles_delay    = 100u;
    static uint16_t cycles                = 0u;
    static int const duty_cycle_amplitude = 40;
    static DIRECTION direction            = POSITIVE;

    set_start_motors(true);
    set_direction(direction);
    set_duty_cycle_value(duty_cycle_amplitude);
    identification_parameters.duty_cycle = (float)duty_cycle_amplitude;

    if(direction == NEGATIVE)
    {
        identification_parameters.duty_cycle *= -1.0f;
    }

    cycles++;
    if(cycles > cycles_delay)
    {
        cycles = 0u;

        if(direction == POSITIVE)
        {
            direction = NEGATIVE;
        }
        else
        {
            direction = POSITIVE;
        }
    }
}