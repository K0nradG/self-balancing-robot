// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "model_identification.h"
#include <math.h>
#include "ble_payload_reader.h"
#include "ble_service.h"

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
    }
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
    BLE_Protocol::Payload_Reader reader(payload, payload_length);
    for(size_t i = 0u; i < MAX_INPUT_DATA_SAMPLES; ++i)
    {
        reader.get_float(input_data.pwm_values[i]);
    }
    for(size_t i = 0u; i < MAX_INPUT_DATA_SAMPLES; ++i)
    {
        reader.get_float(input_data.pwm_durations_s[i]);
        if(!isfinite(input_data.pwm_values[i]) || !isfinite(input_data.pwm_durations_s[i]) ||
           (input_data.pwm_durations_s[i] <= 0.0f))
        {
            return false;
        }
    }
    if(!reader.done())
    {
        return false;
    }

    m_input_data = input_data;
    return true;
}
