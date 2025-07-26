#include "encoder.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "utils.h"

#define M_PI 3.14159265358979323846f
#define WRAP_TO_2PI (2.0f * M_PI / CONFIG_IMPULSE_TO_SHAFT_ROTATION)

#define MM_TO_M 1 / 1000

static const struct gpio_dt_spec encoder_0_a = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_0), channel_a_gpios);
static const struct gpio_dt_spec encoder_0_b = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_0), channel_b_gpios);
static const struct gpio_dt_spec encoder_1_a = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_1), channel_a_gpios);
static const struct gpio_dt_spec encoder_1_b = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_1), channel_b_gpios);

static struct gpio_callback encoder_0_a_data;
static struct gpio_callback encoder_0_b_data;
static struct gpio_callback encoder_1_a_data;
static struct gpio_callback encoder_1_b_data;

encoders_data g_encoders_data = {0};

encoder_data_updated_cb_t encoder_data_updated_cb = NULL;

static struct k_work_delayable encoder_data_update_work;

// Lookup table: 16 entries for all possible transitions
// -1 = CCW, +1 = CW, 0 = invalid/bounce
static const int8_t transition_table[16] = {
    0,   // 0000
    -1,  // 0001
    1,   // 0010
    0,   // 0011 (invalid)
    1,   // 0100
    0,   // 0101
    0,   // 0110 (invalid)
    -1,  // 0111
    -1,  // 1000
    0,   // 1001 (invalid)
    0,   // 1010
    1,   // 1011
    0,   // 1100 (invalid)
    1,   // 1101
    -1,  // 1110
    0    // 1111
};

void
encoder_0_gpio_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    static uint8_t prev_state;

    bool chan_a_state = gpio_pin_get_dt(&encoder_0_a);
    bool chan_b_state = gpio_pin_get_dt(&encoder_0_b);

    uint8_t curr_state = (chan_a_state << 1) | chan_b_state;
    uint8_t transition = (prev_state << 2) | curr_state;

    int8_t delta = transition_table[transition];
    g_encoders_data.encoder_0.impulse_count += delta;

    prev_state = curr_state;
}

void
encoder_1_gpio_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    static uint8_t prev_state;

    bool chan_a_state = gpio_pin_get_dt(&encoder_1_a);
    bool chan_b_state = gpio_pin_get_dt(&encoder_1_b);

    uint8_t curr_state = (chan_a_state << 1) | chan_b_state;
    uint8_t transition = (prev_state << 2) | curr_state;

    int8_t delta = transition_table[transition];
    g_encoders_data.encoder_1.impulse_count += delta;

    prev_state = curr_state;
}

static int
init(void)
{
    if(!device_is_ready(encoder_0_a.port) || !device_is_ready(encoder_0_b.port) || !device_is_ready(encoder_1_a.port) ||
       !device_is_ready(encoder_1_b.port))
    {
#ifdef CONFIG_ENCODER_LOG
        platform_log("ENCODER", LOG_LEVEL_ERR, "encoder not ready");
#endif  // CONFIG_ENCODER_LOG
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&encoder_0_a, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&encoder_0_b, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&encoder_1_a, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&encoder_1_b, GPIO_INPUT);

#ifdef CONFIG_ENCODER_LOG
    if(ret)
    {
        platform_log("ENCODER", LOG_LEVEL_ERR, "encoder pins not ready");
        return ret;
    }
    platform_log("ENCODER", LOG_LEVEL_INF, "encoder init finished");
#endif  // CONFIG_ENCODER_LOG

    gpio_pin_interrupt_configure_dt(&encoder_0_a, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&encoder_0_b, GPIO_INT_EDGE_BOTH);

    gpio_pin_interrupt_configure_dt(&encoder_1_a, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&encoder_1_b, GPIO_INT_EDGE_BOTH);

    gpio_init_callback(&encoder_0_a_data, encoder_0_gpio_callback, BIT(encoder_0_a.pin));
    gpio_add_callback(encoder_0_a.port, &encoder_0_a_data);
    gpio_init_callback(&encoder_0_b_data, encoder_0_gpio_callback, BIT(encoder_0_b.pin));
    gpio_add_callback(encoder_0_b.port, &encoder_0_b_data);

    gpio_init_callback(&encoder_1_a_data, encoder_1_gpio_callback, BIT(encoder_1_a.pin));
    gpio_add_callback(encoder_1_a.port, &encoder_1_a_data);
    gpio_init_callback(&encoder_1_b_data, encoder_1_gpio_callback, BIT(encoder_1_b.pin));
    gpio_add_callback(encoder_1_b.port, &encoder_1_b_data);

    return ret;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void
encoder_data_update_work_handler(struct k_work* work)
{
    ARG_UNUSED(work);

    static float prev_angle_rad_encoder_0;
    static float prev_angle_rad_encoder_1;
    static int64_t last_timestamp_ms = 0;

    int64_t now_ms    = k_uptime_get();
    float dt          = (last_timestamp_ms > 0) ? (now_ms - last_timestamp_ms) / 1000.0f : 0.0f;
    last_timestamp_ms = now_ms;

    if(encoder_data_updated_cb && dt > 0.0f)
    {
        g_encoders_data.encoder_0.shaft_rotate_count =
            g_encoders_data.encoder_0.impulse_count / (float)CONFIG_IMPULSE_TO_SHAFT_ROTATION;
        g_encoders_data.encoder_0.shaft_angle_rad = g_encoders_data.encoder_0.impulse_count * WRAP_TO_2PI;
        g_encoders_data.encoder_0.distance_m =
            g_encoders_data.encoder_0.shaft_rotate_count * (M_PI * CONFIG_WHEEL_DIAMETER_MM) * MM_TO_M;

        float delta_angle_0 = g_encoders_data.encoder_0.shaft_angle_rad - prev_angle_rad_encoder_0;
        g_encoders_data.encoder_0.angular_velocity_rad_s = delta_angle_0 / dt;
        g_encoders_data.encoder_0.linear_velocity_m_s =
            g_encoders_data.encoder_0.angular_velocity_rad_s * (CONFIG_WHEEL_DIAMETER_MM / 2.0f) * MM_TO_M;

        prev_angle_rad_encoder_0 = g_encoders_data.encoder_0.shaft_angle_rad;

        g_encoders_data.encoder_1.shaft_rotate_count =
            g_encoders_data.encoder_1.impulse_count / (float)CONFIG_IMPULSE_TO_SHAFT_ROTATION;
        g_encoders_data.encoder_1.shaft_angle_rad = g_encoders_data.encoder_1.impulse_count * WRAP_TO_2PI;
        g_encoders_data.encoder_1.distance_m =
            g_encoders_data.encoder_1.shaft_rotate_count * (M_PI * CONFIG_WHEEL_DIAMETER_MM) * MM_TO_M;

        float delta_angle_1 = g_encoders_data.encoder_1.shaft_angle_rad - prev_angle_rad_encoder_1;
        g_encoders_data.encoder_1.angular_velocity_rad_s = delta_angle_1 / dt;
        g_encoders_data.encoder_1.linear_velocity_m_s =
            g_encoders_data.encoder_1.angular_velocity_rad_s * (CONFIG_WHEEL_DIAMETER_MM / 2.0f) * MM_TO_M;

        prev_angle_rad_encoder_1 = g_encoders_data.encoder_1.shaft_angle_rad;

        encoder_data_updated_cb(g_encoders_data);
    }
}

static K_WORK_DELAYABLE_DEFINE(encoder_data_update_work, encoder_data_update_work_handler);

void
new_encoder_data_updated_cb_register(encoder_data_updated_cb_t _new_encoder_data_cb)
{
    if(_new_encoder_data_cb)
    {
        encoder_data_updated_cb = _new_encoder_data_cb;
    }
}

void
get_encoders_data(void)
{
    k_work_submit(&encoder_data_update_work.work);
}