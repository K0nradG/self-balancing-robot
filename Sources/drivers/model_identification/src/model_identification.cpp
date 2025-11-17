#include "model_identification.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include "control_loop.h"
#include "identification_data_send.h"
#include "logger.h"

static Logging::Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> model_identification_logger("MODEL");

static const  gpio_dt_spec button      = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static  gpio_callback g_button_data_cb {};

static  ring_buf g_buffers[BUFFER_COUNT]                          {};
static uint8_t g_buffer_data[BUFFER_COUNT][BUFFER_SIZE * sizeof(float)] {};
static uint16_t g_buffer_index[BUFFER_COUNT]                            {};
static bool g_is_full[BUFFER_COUNT]                                     {};

static void
model_identification_work_handler( k_work* work);

static K_WORK_DELAYABLE_DEFINE(model_identification_work, model_identification_work_handler);

enum identification_state
{
    IDENTIFICATION_STOPPED,
    IDENTIFICATION_STARTED,
    TRIGGER_SENDING
};

static identification_state state = IDENTIFICATION_STOPPED;

static identification_data g_identification_data {};

void
new_regulator_data_for_identification(identification_data data)
{
    g_identification_data.dt       = data.dt;
    g_identification_data.pwm      = data.pwm;
    g_identification_data.angle    = data.angle;
    g_identification_data.angle_dt = data.angle_dt;
}

void
button_pressed(const  device* dev,  gpio_callback* cb, uint32_t pins);

int
identification_init()
{
    if(!device_is_ready(button.port))
    {
        return -1;
    }

    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if(ret != 0)
    {
        model_identification_logger.platform_log( Logging::LOG_LEVEL::ERR, "GPIO pin configuration failed, err: %d", ret);
        return ret;
    }

    ret |= gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if(ret != 0)
    {
        model_identification_logger.platform_log( Logging::LOG_LEVEL::ERR, "GPIO pin interrupt configuration failed, err: %d", ret);
        return ret;
    }

    gpio_init_callback(&g_button_data_cb, button_pressed, BIT(button.pin));
    ret |= gpio_add_callback(button.port, &g_button_data_cb);

    if(ret != 0)
    {
        model_identification_logger.platform_log( Logging::LOG_LEVEL::ERR, "GPIO callback add failed, err: %d", ret);
        return ret;
    }

    for(int i = 0; i < BUFFER_COUNT; i++)
    {
        ring_buf_init(&g_buffers[i], BUFFER_SIZE * sizeof(float), g_buffer_data[i]);
        g_buffer_index[i] = 0;
        g_is_full[i]      = false;
    }

    return ret;
}

static bool
buffer_all_full();

static bool
buffer_put(uint8_t buffer_id, float data);

static void
model_identification_stop();

static void
model_identification_work_handler( k_work* work)
{
    ARG_UNUSED(work);

    if(!buffer_all_full())
    {
#if defined(CONFIG_MODEL_IDENTIFICATION_DRV)
        buffer_put(ANGLE_BUFFER_ID, g_identification_data.angle);
        buffer_put(ANGLE_DT_BUFFER_ID, g_identification_data.angle_dt);
        buffer_put(U_BUFFER_ID, g_identification_data.pwm);
        buffer_put(TIME_BUFFER_ID, g_identification_data.dt);
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
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

    model_identification_logger.platform_log(Logging::LOG_LEVEL::INF, "PWM %d", (int)g_identification_data.pwm);
}

void
trigger_collecting_identification_data()
{
    int const err = k_work_submit(&model_identification_work.work);
    if(err != 0)
    {
        model_identification_logger.platform_log( Logging::LOG_LEVEL::ERR, "Identification data collecting failed, err: %d", err);
    }
}

void
model_identification_start()
{
    start_control_loop();
}

static void
model_identification_stop()
{
    stop_control_loop();
}

static bool
buffer_put(uint8_t buffer_id, float data)
{
    if(buffer_id >= BUFFER_COUNT)
        return false;

    if(g_is_full[buffer_id])
        return false;

    int ret = ring_buf_put(&g_buffers[buffer_id], (uint8_t*)&data, sizeof(float));
    if(ret == sizeof(float))
    {
        g_buffer_index[buffer_id]++;
        if(g_buffer_index[buffer_id] >= BUFFER_SIZE)
        {
            g_is_full[buffer_id] = true;
        }
        return true;
    }
    return false;
}

uint16_t
buffer_get(uint8_t buffer_id, float* data, uint16_t max_len)
{
    if(buffer_id >= BUFFER_COUNT)
    {
        return 0u;
    }

    uint16_t const len = (g_buffer_index[buffer_id] < max_len) ? g_buffer_index[buffer_id] : max_len;
    for(uint16_t i = 0; i < len; i++)
    {
        ring_buf_get(&g_buffers[buffer_id], (uint8_t*)&data[i], sizeof(float));
    }

    g_buffer_index[buffer_id] -= len;
    return len;
}

static bool
buffer_all_full()
{
    for(int i = 0; i < BUFFER_COUNT; i++)
    {
        if(!g_is_full[i])
        {
            return false;
        }
    }
    return true;
}

static void
state_machine_update();

void
button_pressed(const  device* dev,  gpio_callback* cb, uint32_t pins)
{
    state_machine_update();
}

static void
buffers_reset();

static void
state_machine_update()
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
notify_data_sent()  // Used by identification_data_send to show that data has already been sent.
{
    state = IDENTIFICATION_STOPPED;
}

static void
buffers_reset()
{
    for(int i = 0; i < BUFFER_COUNT; i++)
    {
        ring_buf_reset(&g_buffers[i]);
        g_buffer_index[i] = 0;
        g_is_full[i]      = false;
    }
}