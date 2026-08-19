// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "model_identification.h"
#include <math.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include "ble_protocol.h"
#include "ble_service.h"
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
        ble_send_packet(BLE_Protocol::Message_Type::IDENTIFICATION_COMPLETE, nullptr, 0u);
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

bool
Model_Identification::set_identification_profile(uint8_t const* payload, uint16_t payload_length)
{
    constexpr uint16_t expected_length = MAX_INPUT_DATA_SAMPLES * 2u * sizeof(float);
    if((payload == nullptr) || (payload_length != expected_length))
    {
        return false;
    }

    Input_Data input_data {};
    for(size_t i = 0u; i < MAX_INPUT_DATA_SAMPLES; ++i)
    {
        input_data.pwm_values[i] = BLE_Protocol::get_float(payload + (i * sizeof(float)));
        input_data.pwm_durations_s[i] =
            BLE_Protocol::get_float(payload + ((MAX_INPUT_DATA_SAMPLES + i) * sizeof(float)));
        if(!isfinite(input_data.pwm_values[i]) || !isfinite(input_data.pwm_durations_s[i]) ||
           (input_data.pwm_durations_s[i] <= 0.0f))
        {
            return false;
        }
    }

    m_input_data = input_data;
    return true;
}

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
