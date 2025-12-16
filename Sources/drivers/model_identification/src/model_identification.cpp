#include "model_identification.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "logger.h"
#include "main_state_machine.h"
#include "robot_controller.h"

static identification_data g_identification_data {};
static input_data g_input_data;
static bool g_send_status = false;

input_data&
get_input_pwm_data()
{
    return g_input_data;
}

pwm_sample
get_pwm_sample(std::size_t index)
{
    pwm_sample sample {0.0f, 0.0f, false};

    if(index >= g_input_data.pwm_values.size())
    {
        sample.pwm0        = g_input_data.pwm_values.back();
        sample.pwm1        = g_input_data.pwm_values.back();
        sample.last_sample = true;
        return sample;
    }

    sample.pwm0        = g_input_data.pwm_values[index];
    sample.pwm1        = g_input_data.pwm_values[index];
    sample.last_sample = (index == g_input_data.pwm_values.size() - 1);

    return sample;
}

void
identification_data_nus_parser_callback(char const* data)
{
    if(!data || *data == '\0')
        return;

    // Skip IDENTIFICATION_PREFIX 'I'
    data++;

    const char* p_start = std::strchr(data, 'p');
    const char* t_start = std::strchr(data, 't');

    if(!p_start || !t_start || t_start < p_start)
        return;

    p_start++;
    t_start++;

    char buffer[32];

    // PWM values parsing
    const char* ptr = p_start;
    for(size_t i = 0; i < g_input_data.pwm_values.size(); ++i)
    {
        const char* underscore = std::strchr(ptr, '_');
        size_t len             = underscore ? (size_t)(underscore - ptr) : std::strlen(ptr);
        if(len >= sizeof(buffer))
            len = sizeof(buffer) - 1;
        std::memcpy(buffer, ptr, len);
        buffer[len]                = '\0';
        g_input_data.pwm_values[i] = std::strtof(buffer, nullptr);

        ptr = underscore ? underscore + 1 : ptr + len;
    }

    // PWM durations parsing
    ptr = t_start;
    for(size_t i = 0; i < g_input_data.pwm_durations_s.size(); ++i)
    {
        const char* underscore = std::strchr(ptr, '_');
        size_t len             = underscore ? (size_t)(underscore - ptr) : std::strlen(ptr);
        if(len >= sizeof(buffer))
            len = sizeof(buffer) - 1;
        std::memcpy(buffer, ptr, len);
        buffer[len]                     = '\0';
        g_input_data.pwm_durations_s[i] = std::strtof(buffer, nullptr);

        ptr = underscore ? underscore + 1 : ptr + len;
    }
}

void
new_regulator_data_for_identification(identification_data data)
{
    g_identification_data.angle       = data.angle;
    g_identification_data.angle_dt    = data.angle_dt;
    g_identification_data.position    = data.position;
    g_identification_data.position_dt = data.position_dt;
}

static Logger<IS_ENABLED(CONFIG_MODEL_IDENTIFICATION_LOG)> logger("MODEL");

void
set_identification_data_status(bool status)
{
    g_send_status = status;
}

static void
identification_logger_thread(void*, void*, void*)
{
    static int64_t last_time_ms = 0;

    while(true)
    {
        int64_t now_ms = k_uptime_get();

        if(last_time_ms != 0)
        {
            g_identification_data.dt = (now_ms - last_time_ms) / 1000.0;
        }

        last_time_ms = now_ms;

        if(g_send_status)
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

K_THREAD_DEFINE(ident_logger_tid, 1024, identification_logger_thread, nullptr, nullptr, nullptr, 1, 0, 0);
