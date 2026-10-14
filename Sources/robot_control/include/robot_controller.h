// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include "ble_payload_reader.h"
#include "ble_protocol_types.h"
#include "constants.h"
#include "ramp.h"
#include "trajectory_manager.h"

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

namespace Robot_Control
{

class Robot_Controller
{
    static constexpr float balance_setpoint     = -14.5f * (PI / RADIAN_IN_DEGREES);  // [rad]
    static constexpr float rotate_setpoint_rate = 180.0f * (PI / RADIAN_IN_DEGREES);  // [rad/s]

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

    BLE_Protocol::Command_Status
    set_setpoints(BLE_Protocol::Payload_Reader& reader);

    BLE_Protocol::Command_Status
    set_trajectory_command(BLE_Protocol::Payload_Reader& reader);

private:
    Robot_Controller();

    Robot_Controller(Robot_Controller const&) = delete;

    Robot_Controller&
    operator=(Robot_Controller const&) = delete;

    float m_distance_setpoint;
    float m_balance_setpoint;
    Ramp m_rotate_setpoint_ramp;

    Trajectory_Manager m_trajectory_manager;

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