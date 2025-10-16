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
    static constexpr float pi             = 3.14159265358979323846f;
    static constexpr float radian_degrees = 180.0f;

    static constexpr float rotate_setpoint  = 0.0f * (pi / radian_degrees);
    static constexpr float balance_setpoint = -16.5f * (pi / radian_degrees);

    static constexpr PID::Parameters wheel_speed_pid_parameters = {.Kp = 0.80f, .Ki = 0.1f, .Kd = 0.001f};  // 0.002
    static constexpr float max_speed_rad_s                      = 70.0f;
    static constexpr float speed_pid_filter_alpha               = 1.0f;  // No filtering.

    static constexpr PID::Parameters rotate_pid_parameters = {.Kp = 25.0f, .Ki = 40.0f, .Kd = 0.1f};
    static constexpr float rotate_pid_filter_alpha         = 1.0f;  // No filtering.
    static constexpr float rotate_pid_hysteresis           = 0.5f * (pi / radian_degrees);

#ifdef CONFIG_PID_ENABLED
    // static constexpr PID::Parameters balance_pid_parameters = {.Kp = 60.0, .Ki = 900.0f, .Kd = 3.9f};  // kp = 270
    static constexpr PID::Parameters balance_pid_parameters = {.Kp = 60.0, .Ki = 900.0f, .Kd = 3.9f};  // kp = 270
    static constexpr float balance_pid_filter_alpha         = 0.9f;
#else
    static constexpr LQR::Parameters balance_lqr_parameters = {.Kx = 0.0, .Ky = 0.0f};
#endif  // CONFIG_PID_ENABLED

public:
    Robot_Controller();

    bool
    normal_motors_control();

    bool
    soft_stop_motors();

    void
    reset_pids();

#ifdef CONFIG_BLUETOOTH_DRV
    void
    parse_nus_data(char const* data);
#endif  // CONFIG_BLUETOOTH_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    identification_data
    get_identification_data() const;
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

private:
    float m_rotate_setpoint;
    float m_balance_setpoint;

    float m_pwm0 {};
    float m_pwm1 {};

    PID m_wheel0_speed_pid;
    PID m_wheel1_speed_pid;

    PID m_rotate_pid;

#ifdef CONFIG_PID_ENABLED
    PID m_balance_pid;
#else
    LQR m_balance_lqr;
#endif  // CONFIG_PID_ENABLED

    void
    send_motors_data(float pwm_motor0, float pwm_motor1);

    bool
    validate_robot_angle(float balance_angle);

    bool
    ramp_pwm_to_stop(float& pwm);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    identification_data m_identification_data {};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
};

}  // namespace Robot_Control