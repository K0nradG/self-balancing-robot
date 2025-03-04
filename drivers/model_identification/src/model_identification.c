#include "model_identification.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include "identyfication_data_send.h"

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

static struct ring_buf buffers[BUFFER_COUNT];
static uint8_t buffer_data[BUFFER_COUNT][BUFFER_SIZE * sizeof(float)];
static uint16_t buffer_index[BUFFER_COUNT] = {0};
static bool is_full[BUFFER_COUNT];

void
button_pressed(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    if(buffer_all_full())
    {
        trigger_identification_data_sending();
    }
}

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
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

bool
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

bool
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