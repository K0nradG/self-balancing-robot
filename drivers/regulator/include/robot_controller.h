#pragma once
#include "imu.h"
#include "pid_controller.h"
#include "regulator_utils.h"

class Robot_Controller
{
    static constexpr float balance_setpoint                 = -12.0f * DEG_TO_RAD;
    static constexpr PID::Parameters balance_pid_parameters = {.Kp = 1300.0, .Ki = 800.0f, .Kd = 0.1f};

public:
    Robot_Controller() : m_balance_setpoint(balance_setpoint), m_balance_pid(balance_pid_parameters) {}

    void
    update_imu_data();

    void
    control_motors();

    void
    parse_pid_params(char const* data);  // Will be registered as callback for NUS.

private:
    imu_data m_imu_data = {0};

    float m_balance_setpoint;
    // Here will be defines determining PID or LQR for balance. There will also be more regulators e.g. for rotation.
    PID m_balance_pid;

    void
    send_motors_data(int pwm_motor0, int pwm_motor1);
};