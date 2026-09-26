// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include "ble_payload_reader.h"
#include "ble_protocol_types.h"
#include "constants.h"
#include "pid.h"

#ifndef CONFIG_PID_ENABLED
#include "lqr.h"
#endif  // not CONFIG_PID_ENABLED

namespace Robot_Control
{

class Control_Loop
{
    static constexpr PID::Parameters distance_pid_parameters = {.Kp = 2.0f, .Ki = 0.1f, .Kd = 0.0f};
    static constexpr float distance_pid_filter_alpha         = 0.9f;
    static constexpr float max_linear_speed                  = 0.5f;  // [m/s]
    static constexpr float distance_pid_hysteresis           = 0.01f;

    static constexpr PID::Parameters linear_speed_pid_parameters = {.Kp = 0.175, .Ki = 0.0f, .Kd = 0.0f};
    static constexpr float linear_speed_pid_filter_alpha         = 0.1f;
    static constexpr float angle_backward_max_deviation          = -3.0f * (PI / RADIAN_IN_DEGREES);
    static constexpr float angle_forward_max_deviation           = 3.0f * (PI / RADIAN_IN_DEGREES);
    static constexpr float max_speed_rad_s                       = 90.0f;

#ifdef CONFIG_PID_ENABLED
    static constexpr PID::Parameters balance_pid_parameters = {.Kp = 60.0, .Ki = 900.0f, .Kd = 3.9f};
    static constexpr float balance_pid_filter_alpha         = 0.9f;
#else
    static constexpr LQR::Parameters balance_lqr_parameters = {.Kx = 0.0, .Ky = 0.0f};
#endif  // CONFIG_PID_ENABLED

    static constexpr PID::Parameters rotate_pid_parameters = {.Kp = 50.0f, .Ki = 25.0f, .Kd = 0.0f};
    static constexpr float rotate_pid_filter_alpha         = 1.0f;  // No filtering.
    static constexpr float rotate_pid_hysteresis           = 0.5f * (PI / RADIAN_IN_DEGREES);

    static constexpr PID::Parameters wheel_speed_pid_parameters = {.Kp = 1.5f, .Ki = 0.1f, .Kd = 0.0f};
    static constexpr float wheel_speed_pid_filter_alpha         = 1.0f;  // No filtering.

public:
    struct Setpoints
    {
        float distance;
        float balance;
        float rotate;
    };

    struct Feedback
    {
        float robot_distance_m;
        float robot_linear_speed;
        float angle_balance;
#ifndef CONFIG_PID_ENABLED
        float angle_balance_dt;
#endif  // CONFIG_PID_ENABLED
        float rotation_angle;
        float angular_velocity0_rad_s;
        float angular_velocity1_rad_s;
    };

    struct Output
    {
        float target_speed0;
        float target_speed1;
        float pwm0;
        float pwm1;
    };

    static Control_Loop&
    instance()
    {
        static Control_Loop s_control_loop {};
        return s_control_loop;
    }

    Output
    update(Setpoints const& setpoints, Feedback const& feedback, float time_dt);

    void
    send_PID_controllers_parameters();

    BLE_Protocol::Command_Status
    set_PID_parameters(BLE_Protocol::Payload_Reader& reader);

#ifndef CONFIG_PID_ENABLED
    BLE_Protocol::Command_Status
    set_LQR_parameters(BLE_Protocol::Payload_Reader& reader);
#endif  // not CONFIG_PID_ENABLED

    void
    reset();

private:
    Control_Loop();

    Control_Loop(Control_Loop const&) = delete;

    Control_Loop&
    operator=(Control_Loop const&) = delete;

    PID m_distance_pid;

    PID m_linear_speed_pid;

#ifdef CONFIG_PID_ENABLED
    PID m_balance_pid;
#else
    LQR m_balance_lqr;
#endif  // CONFIG_PID_ENABLED

    PID m_rotate_pid;

    PID m_wheel0_speed_pid;
    PID m_wheel1_speed_pid;
};

}  // namespace Robot_Control