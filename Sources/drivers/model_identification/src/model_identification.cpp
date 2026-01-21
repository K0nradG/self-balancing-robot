#include "model_identification.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <cstdlib>
#include <cstring>
#include "logger.h"

static Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> logger("MODEL");
static const struct device* uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

struct __packed IdentificationFrame
{
    uint32_t magic = 0xDEADBEEF;
    float dt;
    float angle;
    float angle_dt;
    float pwm;
    float pos;
    float pos_dt;
};

IdentificationFrame binary_frame;

void
Model_Identification::update(float dt)
{
    m_pwm_timer += dt;

    if(m_pwm_timer >= m_input_data.pwm_durations_s[m_current_pwm_sample])
    {
        m_pwm_timer = 0.0f;

        if(m_current_pwm_sample < (MAX_INPUT_DATA_SAMPLES - 1u))
        {
            m_current_pwm_sample++;
        }
        else
        {
            m_identification_active = false;
            m_current_pwm_sample    = 0u;
        }
    }
}

void
Model_Identification::activate_identification()
{
    m_identification_active = true;
}

bool
Model_Identification::identification_active()
{
    return m_identification_active;
}

void
Model_Identification::acknowledge_identification_stop()
{
    if(!m_identification_active)
    {
        logger.platform_log(LOG_LEVEL::INF, "Identification stop");
    }
}

void
Model_Identification::new_regulator_data_for_identification(Identification_Data const& data)
{
    binary_frame.angle    = data.angle;
    binary_frame.angle_dt = data.angle_dt;
    binary_frame.pos      = data.position;
    binary_frame.pos_dt   = data.position_dt;
    binary_frame.pwm      = data.pwm;
}

float
Model_Identification::get_pwm_sample()
{
    float pwm_sample = 0.0f;

    if(m_current_pwm_sample < MAX_INPUT_DATA_SAMPLES)
    {
        pwm_sample = m_input_data.pwm_values[m_current_pwm_sample];
    }
    return pwm_sample;
}

void
Model_Identification::identification_data_nus_parser_callback(char const* data)
{
    if(!data || (*data == '\0'))
    {
        return;
    }

    // Skip IDENTIFICATION_PREFIX 'I'
    data++;

    char const* p_start = std::strchr(data, 'p');
    char const* t_start = std::strchr(data, 't');

    if(!p_start || !t_start || (t_start < p_start))
    {
        return;
    }

    p_start++;
    t_start++;

    char buffer[32];

    // PWM values parsing
    char const* ptr = p_start;
    for(size_t i = 0; i < MAX_INPUT_DATA_SAMPLES; ++i)
    {
        char const* underscore = std::strchr(ptr, '_');
        size_t len             = underscore ? (size_t)(underscore - ptr) : std::strlen(ptr);
        if(len >= sizeof(buffer))
        {
            len = sizeof(buffer) - 1;
        }

        std::memcpy(buffer, ptr, len);
        buffer[len]                = '\0';
        m_input_data.pwm_values[i] = std::strtof(buffer, nullptr);

        ptr = underscore ? underscore + 1 : ptr + len;
    }

    // PWM durations parsing
    ptr = t_start;
    for(size_t i = 0; i < MAX_INPUT_DATA_SAMPLES; ++i)
    {
        const char* underscore = std::strchr(ptr, '_');
        size_t len             = underscore ? (size_t)(underscore - ptr) : std::strlen(ptr);
        if(len >= sizeof(buffer))
        {
            len = sizeof(buffer) - 1;
        }

        std::memcpy(buffer, ptr, len);
        buffer[len]                     = '\0';
        m_input_data.pwm_durations_s[i] = std::strtof(buffer, nullptr);

        ptr = underscore ? underscore + 1 : ptr + len;
    }
}

// send packed struct instead:
// https://gemini.google.com/app/fdfd07ad8ceed938

static void
identification_logger_thread(void*, void*, void*)
{
    static int64_t last_time_ms = 0;
    binary_frame.magic          = 0xDEADBEEF;

    while(true)
    {
        int64_t const now_ms = k_uptime_get();

        if(last_time_ms != 0)
        {
            binary_frame.dt = static_cast<float>(now_ms - last_time_ms) / 1000.0f;
        }

        last_time_ms = now_ms;

        if(Model_Identification::instance().identification_active())
        {
            uint8_t* raw_ptr = reinterpret_cast<uint8_t*>(&binary_frame);
            for(size_t i = 0; i < sizeof(IdentificationFrame); ++i)
            {
                uart_poll_out(uart_dev, raw_ptr[i]);
            }
        }

        k_sleep(K_MSEC(CONFIG_MODEL_IDENTIFICATION_SAMPLE_TIME));
    }
}

K_THREAD_DEFINE(ident_logger_tid, 8196, identification_logger_thread, nullptr, nullptr, nullptr, 1, 0, 0);
