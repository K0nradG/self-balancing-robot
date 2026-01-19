#pragma once

#include <stdint.h>

class Model_Identification
{
    static constexpr uint8_t MAX_INPUT_DATA_SAMPLES = 10u;

public:
    struct Identification_Data
    {
        float dt {};
        float pwm {};
        float angle {};
        float angle_dt {};
        float position {};
        float position_dt {};
    };

    static Model_Identification&
    instance()
    {
        static Model_Identification s_model_identification {};
        return s_model_identification;
    }

    void
    update(float dt);

    void
    activate_identification();

    bool
    identification_active();

    void
    acknowledge_identification_stop();

    void
    new_regulator_data_for_identification(Identification_Data const& data);

    float
    get_pwm_sample();

    void
    identification_data_nus_parser_callback(char const* data);

    void
    set_current_dt(float dt);

    Identification_Data const&
    get_identification_data() const;

private:
    Model_Identification()                            = default;
    Model_Identification(Model_Identification const&) = delete;
    Model_Identification&
    operator=(Model_Identification const&) = delete;

    struct Input_Data
    {
        float pwm_values[MAX_INPUT_DATA_SAMPLES];
        float pwm_durations_s[MAX_INPUT_DATA_SAMPLES];
    };

    Identification_Data m_identification_data {};
    Input_Data m_input_data {};

    bool m_identification_active  = false;
    uint32_t m_current_pwm_sample = 0u;
    float m_pwm_timer             = 0.0f;
};
