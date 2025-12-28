#include "model_identification.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <cstdlib>
#include <cstring>
#include "logger.h"

static Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> logger("MODEL");

static Identification_Data g_identification_data {};
static Input_Data g_input_data {};
static bool g_identification_active = false;

static size_t current_pwm_sample = 0u;
static float pwm_timer           = 0.0f;

void
update(float dt)
{
    pwm_timer += dt;

    if(pwm_timer >= g_input_data.pwm_durations_s[current_pwm_sample])
    {
        pwm_timer = 0.0f;

        if(current_pwm_sample < (MAX_INPUT_DATA_SAMPLES - 1u))
        {
            current_pwm_sample++;
        }
        else
        {
            g_identification_active = false;
            current_pwm_sample      = 0u;
        }
    }
}

void
activate_identification()
{
    g_identification_active = true;
}

bool
identification_active()
{
    return g_identification_active;
}

void
new_regulator_data_for_identification(Identification_Data const& data)
{
    g_identification_data.angle       = data.angle;
    g_identification_data.angle_dt    = data.angle_dt;
    g_identification_data.position    = data.position;
    g_identification_data.position_dt = data.position_dt;
    g_identification_data.pwm         = data.pwm;
}

PWM_Sample const
get_pwm_sample()
{
    PWM_Sample sample {0.0f, 0.0f};

    if(current_pwm_sample < MAX_INPUT_DATA_SAMPLES)
    {
        sample.pwm0 = g_input_data.pwm_values[current_pwm_sample];
        sample.pwm1 = g_input_data.pwm_values[current_pwm_sample];
    }

    return sample;
}

static void
identification_logger_thread(void*, void*, void*)
{
    static int64_t last_time_ms = 0;

    while(true)
    {
        int64_t const now_ms = k_uptime_get();

        if(last_time_ms != 0)
        {
            g_identification_data.dt = static_cast<float>(now_ms - last_time_ms) / 1000.0f;
        }

        last_time_ms = now_ms;

        if(g_identification_active)
        {
            logger.platform_log(
                LOG_LEVEL::INF, "dt=%.4f angle=%.4f angle_dt=%.4f pwm=%.4f pos=%.4f pos_dt=%.4f",
                (double)g_identification_data.dt, (double)g_identification_data.angle,
                (double)g_identification_data.angle_dt, (double)g_identification_data.pwm,
                (double)g_identification_data.position, (double)g_identification_data.position_dt);
        }

        k_sleep(K_MSEC(CONFIG_MODEL_IDENTIFICATION_SAMPLE_TIME));
    }
}

K_THREAD_DEFINE(ident_logger_tid, 8196, identification_logger_thread, nullptr, nullptr, nullptr, 1, 0, 0);

void
identification_data_nus_parser_callback(char const* data)
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
        g_input_data.pwm_values[i] = std::strtof(buffer, nullptr);

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
        g_input_data.pwm_durations_s[i] = std::strtof(buffer, nullptr);

        ptr = underscore ? underscore + 1 : ptr + len;
    }
}
