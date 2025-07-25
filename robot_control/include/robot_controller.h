#pragma once

#include "pid.h"

#ifndef CONFIG_PID_ENABLED
#include "lqr.h"
#endif  // not CONFIG_PID_ENABLED

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

namespace Robot_Control
{

class Robot_Controller
{
    static constexpr float pi               = 3.14159265358979323846f;
    static constexpr float radian_degrees   = 180.0f;
    static constexpr float balance_setpoint = -11.7f * (pi / radian_degrees);

#ifdef CONFIG_PID_ENABLED
    static constexpr PID::Parameters balance_pid_parameters = {.Kp = 300.0, .Ki = 35.0f, .Kd = 2.3f};  // kp = 270
    static constexpr float balance_pid_filter_alpha         = 0.9f;
#else
    static constexpr LQR::Parameters balance_lqr_parameters = {.Kx = 0.0, .Ky = 0.0f};
#endif  // CONFIG_PID_ENABLED

    static constexpr PID::Parameters wheel_speed_pid_parameters = {.Kp = 0.80f, .Ki = 0.0f, .Kd = 0.0f};
    static constexpr float speed_pid_filter_alpha               = 1.0f;  // No filtering.

public:
    Robot_Controller();

    void
    control_motors();

#ifdef CONFIG_LOG_OVER_BLE
    void
    parse_nus_data(char const* data);
#endif  // CONFIG_LOG_OVER_BLE

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    identification_data
    get_identification_data() const;

#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

private:
    float m_balance_setpoint;

#ifdef CONFIG_PID_ENABLED
    PID m_balance_pid;
#else
    LQR m_balance_lqr;
#endif  // CONFIG_PID_ENABLED

    PID m_wheel0_speed_pid;
    PID m_wheel1_speed_pid;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    identification_data m_identification_data {};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

    void
    send_motors_data(int pwm_motor0, int pwm_motor1);
};

}  // namespace Robot_Control