#include "encoder.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#ifdef CONFIG_ENCODER_LOG
#include "logger.h"
#endif  // CONFIG_ENCODER_LOG

#define M_PI        3.14159265358979323846f
#define WRAP_TO_2PI (2.0f * M_PI / CONFIG_IMPULSES_FOR_SHAFT_ROTATION)
#define MILLI_TO_SI 1.0f / 1000.0f

#define WHEEL_BASE_WIDTH_M (float)CONFIG_WHEEL_BASE_WIDTH_MM* MILLI_TO_SI
#define WHEEL_DIAMETER_M   (float)CONFIG_WHEEL_DIAMETER_MM* MILLI_TO_SI
#define WHEEL_RADIUS_M     WHEEL_DIAMETER_M / 2.0f

static const struct gpio_dt_spec encoder_0_a = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_0), channel_a_gpios);
static const struct gpio_dt_spec encoder_0_b = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_0), channel_b_gpios);
static const struct gpio_dt_spec encoder_1_a = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_1), channel_a_gpios);
static const struct gpio_dt_spec encoder_1_b = GPIO_DT_SPEC_GET(DT_NODELABEL(encoder_1), channel_b_gpios);

static struct gpio_callback encoder_0_a_data;
static struct gpio_callback encoder_0_b_data;
static struct gpio_callback encoder_1_a_data;
static struct gpio_callback encoder_1_b_data;

encoders_data g_encoders_data = {0};

// Lookup table: 16 entries for all possible transitions
// -1 = CCW, +1 = CW, 0 = invalid/bounce
static int8_t const transition_table[16u] = {
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
    static uint8_t prev_state = 0u;

    bool const chan_a_state = gpio_pin_get_dt(&encoder_0_a);
    bool const chan_b_state = gpio_pin_get_dt(&encoder_0_b);

    uint8_t const curr_state = (chan_a_state << 1) | chan_b_state;
    uint8_t const transition = (prev_state << 2) | curr_state;

    int8_t const delta = transition_table[transition];
    g_encoders_data.encoder_0.impulse_count += delta;

    prev_state = curr_state;
}

void
encoder_1_gpio_callback(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
    static uint8_t prev_state = 0u;

    bool const chan_a_state = gpio_pin_get_dt(&encoder_1_a);
    bool const chan_b_state = gpio_pin_get_dt(&encoder_1_b);

    uint8_t const curr_state = (chan_a_state << 1) | chan_b_state;
    uint8_t const transition = (prev_state << 2) | curr_state;

    int8_t const delta = transition_table[transition];
    g_encoders_data.encoder_1.impulse_count += delta;

    prev_state = curr_state;
}

int
encoders_init(void)
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

    if(ret)
    {
#ifdef CONFIG_ENCODER_LOG
        platform_log("ENCODER", LOG_LEVEL_ERR, "encoder pins not ready");
#endif  // CONFIG_ENCODER_LOG
        return ret;
    }
#ifdef CONFIG_ENCODER_LOG
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

static void
update_encoder(encoder_data* encoder, float prev_angle_rad, float dt)
{
    encoder->shaft_rotate_count = (float)encoder->impulse_count / (float)CONFIG_IMPULSES_FOR_SHAFT_ROTATION;
    encoder->shaft_angle_rad    = (float)encoder->impulse_count * WRAP_TO_2PI;
    encoder->distance_m         = encoder->shaft_rotate_count * M_PI * WHEEL_DIAMETER_M;

    float const delta_angle_0       = encoder->shaft_angle_rad - prev_angle_rad;
    encoder->angular_velocity_rad_s = delta_angle_0 / dt;
    encoder->linear_velocity_m_s    = encoder->angular_velocity_rad_s * WHEEL_RADIUS_M;
}

encoders_data
_get_encoders_data(void)
{
    static float prev_angle_rad_encoder_0 = 0.0f;
    static float prev_angle_rad_encoder_1 = 0.0f;
    static int64_t last_timestamp_ms      = 0;

    int64_t const now_ms = k_uptime_get();
    float const dt       = (last_timestamp_ms > 0) ? (now_ms - last_timestamp_ms) * MILLI_TO_SI : 0.01f;
    last_timestamp_ms    = now_ms;

    update_encoder(&g_encoders_data.encoder_0, prev_angle_rad_encoder_0, dt);
    prev_angle_rad_encoder_0 = g_encoders_data.encoder_0.shaft_angle_rad;

    update_encoder(&g_encoders_data.encoder_1, prev_angle_rad_encoder_1, dt);
    prev_angle_rad_encoder_1 = g_encoders_data.encoder_1.shaft_angle_rad;

    g_encoders_data.robot_angle_rad =
        (g_encoders_data.encoder_1.shaft_angle_rad - g_encoders_data.encoder_0.shaft_angle_rad) *
        (WHEEL_DIAMETER_M / (2.0f * WHEEL_BASE_WIDTH_M));

    return g_encoders_data;
}