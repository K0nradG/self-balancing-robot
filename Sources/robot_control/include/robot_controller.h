// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include "pid.h"
#include "ramp.h"
#include "trajectory_manager.h"

#ifndef CONFIG_PID_ENABLED
#include "lqr.h"
#endif  // not CONFIG_PID_ENABLED

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_BLUETOOTH_DRV
#include "ble_protocol.h"
#endif

namespace Robot_Control
{

enum class Control_Mode : uint8_t
{
    STANDARD   = 0,
    FREE_DRIVE = 1
};

class Robot_Controller
{
    static constexpr float pi             = 3.14159265358979323846f;
    static constexpr float radian_degrees = 180.0f;

    static constexpr float balance_setpoint     = -14.5f * (pi / radian_degrees);  // [rad]
    static constexpr float rotate_setpoint_rate = 180.0f * (pi / radian_degrees);  // [rad/s]

    static constexpr float angular_speed_setpoint_ramp_rate = 1080.0f * (pi / radian_degrees);  // [rad/s^2]
    static constexpr float linear_speed_setpoint_ramp_rate  = 0.5f;                             // [m/s^2]

    static constexpr PID::Parameters distance_pid_parameters = {.Kp = 2.0f, .Ki = 0.1f, .Kd = 0.0f};
    static constexpr float distance_pid_filter_alpha         = 0.9f;
    static constexpr float max_linear_speed                  = 0.5f;  // [m/s]
    static constexpr float distance_pid_hysteresis           = 0.01f;

    static constexpr PID::Parameters linear_speed_pid_parameters = {.Kp = 0.2350, .Ki = 0.0f, .Kd = 0.0f};
    static constexpr float linear_speed_pid_filter_alpha         = 0.1f;
    static constexpr float angle_backward_max_deviation          = -5.0f * (pi / radian_degrees);
    static constexpr float angle_forward_max_deviation           = 5.0f * (pi / radian_degrees);
    static constexpr float max_speed_rad_s                       = 180.0f;

#ifdef CONFIG_PID_ENABLED
    static constexpr PID::Parameters balance_pid_parameters = {.Kp = 80.0, .Ki = 900.0f, .Kd = 3.9f};
    static constexpr float balance_pid_filter_alpha         = 0.9f;
#else
    static constexpr LQR::Parameters balance_lqr_parameters = {.Kx = 0.0, .Ky = 0.0f};
#endif  // CONFIG_PID_ENABLED

    static constexpr PID::Parameters rotate_pid_parameters = {.Kp = 80.0f, .Ki = 25.0f, .Kd = 0.0f};
    static constexpr float rotate_pid_filter_alpha         = 1.0f;  // No filtering.
    static constexpr float rotate_pid_hysteresis           = 0.5f * (pi / radian_degrees);

    static constexpr PID::Parameters wheel_speed_pid_parameters = {.Kp = 1.5f, .Ki = 0.1f, .Kd = 0.0f};
    static constexpr float wheel_speed_pid_filter_alpha         = 1.0f;  // No filtering.

public:
    static Robot_Controller&
    instance()
    {
        static Robot_Controller s_robot_controller {};
        return s_robot_controller;
    }

#ifndef CONFIG_MODEL_IDENTIFICATION_DRV
    bool
    normal_motors_control();
#else
    Model_Identification::Identification_Data const
    model_identification();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

    bool
    soft_stop_motors();

    void
    reset();

#ifdef CONFIG_BLUETOOTH_DRV
    void
    handle_ble_packet(BLE_Protocol::received_packet const& received_packet);

    void
    send_PID_controllers_parameters();
#endif  // CONFIG_BLUETOOTH_DRV

private:
    Robot_Controller();

    Robot_Controller(Robot_Controller const&) = delete;

    Robot_Controller&
    operator=(Robot_Controller const&) = delete;

    float m_distance_setpoint;
    float m_balance_setpoint;
    Ramp m_rotate_setpoint_ramp;
    Ramp m_angular_speed_setpoint_ramp;
    Ramp m_linear_speed_setpoint_ramp;

    Trajectory_Manager m_trajectory_manager;

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

    Control_Mode m_current_mode = Control_Mode::STANDARD;

    bool m_regulator_message_sending_in_progress;

    float m_pwm0 {};
    float m_pwm1 {};

    void
    send_motors_data(float pwm_motor0, float pwm_motor1);

    bool
    validate_robot_angle(float balance_angle);

    bool
    ramp_pwm_to_stop(float& pwm);
};

}  // namespace Robot_Control