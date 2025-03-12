#include "model_identification.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include "identification_data_send.h"
#include "regulator.h"

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

typedef struct imu_parameters
{
    float angle;
    float angle_dt;

} imu_parameters;

typedef struct regulator_parameters
{
    float dt;
    float pwm;

} regulator_parameters;

typedef enum identification_state
{
    IDENTIFICATION_STOPPED,
    IDENTIFICATION_STARTED,
    TRIGGER_SENDING
} identification_state;

static identification_state state = IDENTIFICATION_STOPPED;

static struct identification_regulator_data identification_data;

void
new_regulator_data_for_identification(struct identification_regulator_data data)
{
    identification_data.dt       = data.dt;
    identification_data.pwm      = data.pwm;
    identification_data.angle    = data.angle;
    identification_data.angle_dt = data.angle_dt;
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
model_identification_stop(void);

static void
model_identification_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    if(!buffer_all_full())
    {
        buffer_put(ANGLE_BUFFER_ID, identification_data.angle);
        buffer_put(ANGLE_DT_BUFFER_ID, identification_data.angle_dt);
        buffer_put(U_BUFFER_ID, identification_data.pwm);
        buffer_put(TIME_BUFFER_ID, identification_data.dt);
    }
    else
    {
        if(state == IDENTIFICATION_STARTED)
        {
            state = TRIGGER_SENDING;
            model_identification_stop();
            return;
        }
    }

#ifdef CONFIG_MODEL_IDENTIFICATION_LOG
    platform_log("IDENTIFICATION", LOG_LEVEL_INF, "PWM %d", (int)identification_data.pwm);
#endif  // CONFIG_MODEL_IDENTIFICATION_LOG
}

static K_WORK_DELAYABLE_DEFINE(model_identification_work, model_identification_work_handler);

void
trigger_collecting_identification_data()
{
    k_work_submit(&model_identification_work.work);
}

void
model_identification_start(void)
{
    regulator_start_automatic_control();
}

static void
model_identification_stop(void)
{
    regulator_stop_automatic_control();
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

static void
state_machine_update(void);

void
button_pressed(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    state_machine_update();
}

static void
buffers_reset(void);

static void
state_machine_update(void)
{
    switch(state)
    {
        case IDENTIFICATION_STOPPED:  // Comeback to this state is handled by identification_data_send.
        {
            state = IDENTIFICATION_STARTED;
            buffers_reset();
            model_identification_start();
            break;
        }
        case IDENTIFICATION_STARTED:
        {
            break;
        }
        case TRIGGER_SENDING:
        {
            trigger_identification_data_sending();
            break;
        }
        default:
        {
            break;
        }
    }
}

void
notify_data_sent(void)  // Used by identification_data_send to show that data has already been sent.
{
    state = IDENTIFICATION_STOPPED;
}

static void
buffers_reset(void)
{
    for(int i = 0; i < BUFFER_COUNT; i++)
    {
        ring_buf_reset(&buffers[i]);
        buffer_index[i] = 0;
        is_full[i]      = false;
    }
}
