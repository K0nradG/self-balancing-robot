// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "robot_controller.h"
#include <math.h>
#include "ble_payload_reader.h"
#include "ble_protocol_types.h"
#include "constants.h"
#include "control_loop.h"
#include "data_manager.h"
#include "logger.h"
#include "main_state_machine.h"
#include "motor_controller.h"

#if defined(CONFIG_ROBOT_CONTROL_LOG) && defined(CONFIG_BLUETOOTH_DRV)
#include "telemetry.h"
#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

namespace Robot_Control
{

static Logger<IS_ENABLED(CONFIG_ROBOT_CONTROL_LOG)> robot_control_logger("ROBOT_CONTROL");

static void
send_command_result(BLE_Protocol::Received_Packet const& received_packet, BLE_Protocol::Command_Status status)
{
    uint8_t payload[6] {};
    BLE_Protocol::Payload_Writer writer(payload, sizeof(payload));
    writer.put_u32(received_packet.packet_number);
    writer.put_u8(static_cast<uint8_t>(received_packet.type));
    writer.put_u8(static_cast<uint8_t>(status));
    ble_send_packet(BLE_Protocol::Message_Type::COMMAND_RESULT, writer);
}

Robot_Controller::Robot_Controller()
    : m_distance_setpoint(0.0f),
      m_balance_setpoint(balance_setpoint),
      m_rotate_setpoint_ramp(rotate_setpoint_rate),
      m_trajectory_manager(m_distance_setpoint, m_rotate_setpoint_ramp),
      m_regulator_message_sending_in_progress(false)
{
}

#ifndef CONFIG_MODEL_IDENTIFICATION_DRV
bool
Robot_Controller::normal_motors_control()
{
    DataManager::instance().update();
    imu_data const imu_data            = DataManager::instance().get_imu_data();
    encoders_data const& encoders_data = DataManager::instance().get_encoders_data();

#ifdef CONFIG_VALIDATE_ROBOT_ANGLE
    bool const disable_motors_command = validate_robot_angle(imu_data.angle_balance);
#else
    bool const disable_motors_command = false;
#endif  // CONFIG_VALIDATE_ROBOT_ANGLE

    if(!disable_motors_command)
    {
        float const rotation_angle = DataManager::instance().get_rotation_angle();
        m_trajectory_manager.update(rotation_angle, encoders_data.robot_distance_m);
        m_rotate_setpoint_ramp.update(imu_data.time_dt);

        Control_Loop::Setpoints const setpoints = {
            .distance = m_distance_setpoint,
            .balance  = m_balance_setpoint,
            .rotate   = m_rotate_setpoint_ramp.get_current_value()};

        Control_Loop::Feedback const feedback = {
            .robot_distance_m   = encoders_data.robot_distance_m,
            .robot_linear_speed = encoders_data.robot_linear_speed,
            .angle_balance      = imu_data.angle_balance,
#ifndef CONFIG_PID_ENABLED
            .angle_balance_dt = imu_data.angle_balance_dt,
#endif  // CONFIG_PID_ENABLED
            .rotation_angle          = rotation_angle,
            .angular_velocity0_rad_s = imu_data.angle_balance_dt,
            .angular_velocity1_rad_s = imu_data.angle_balance_dt};

        Control_Loop::Output const control_loop_output =
            Control_Loop::instance().update(setpoints, feedback, imu_data.time_dt);
        m_pwm0 = control_loop_output.pwm0;
        m_pwm1 = control_loop_output.pwm1;

#if defined(CONFIG_ROBOT_CONTROL_LOG) && defined(CONFIG_BLUETOOTH_DRV)
        if(!m_trajectory_manager.stop_logs())
        {
            Telemetry_Sample const telemetry_sample = {
                .timestamp_us      = k_uptime_get_32() * 1000u,
                .balance_setpoint  = m_balance_setpoint * RADIAN_IN_DEGREES / PI,
                .balance_angle     = imu_data.angle_balance * RADIAN_IN_DEGREES / PI,
                .rotation_setpoint = m_rotate_setpoint_ramp.get_current_value() * RADIAN_IN_DEGREES / PI,
                .rotation_angle    = rotation_angle * RADIAN_IN_DEGREES / PI,
                .target_speed_0    = control_loop_output.target_speed0,
                .target_speed_1    = control_loop_output.target_speed1,
                .measured_speed_0  = encoders_data.encoder_0.angular_velocity_rad_s,
                .measured_speed_1  = encoders_data.encoder_1.angular_velocity_rad_s,
                .pwm_0             = m_pwm0,
                .pwm_1             = m_pwm1,
            };
            telemetry_submit(telemetry_sample);
        }
#endif
    }

    send_motors_data(m_pwm0, m_pwm1);
    trigger_motors_update();

    return disable_motors_command;
}

#else
Model_Identification::Identification_Data const
Robot_Controller::model_identification()
{
    DataManager::instance().update();
    imu_data const imu_data            = DataManager::instance().get_imu_data();
    encoders_data const& encoders_data = DataManager::instance().get_encoders_data();

    float const pwm_sample = Model_Identification::instance().get_pwm_sample();
    m_pwm0                 = pwm_sample;
    m_pwm1                 = pwm_sample;

    Model_Identification::Identification_Data const identification_data = {
        .dt          = imu_data.time_dt,
        .pwm         = pwm_sample,
        .angle       = imu_data.angle_balance,
        .angle_dt    = imu_data.angle_balance_dt,
        .position    = encoders_data.robot_distance_m,
        .position_dt = encoders_data.robot_linear_speed};

    Model_Identification::instance().update(imu_data.time_dt);

    send_motors_data(m_pwm0, m_pwm1);
    trigger_motors_update();

    return identification_data;
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

bool
Robot_Controller::soft_stop_motors()
{
    bool const motors_stopped = (ramp_pwm_to_stop(m_pwm0) && ramp_pwm_to_stop(m_pwm1));
    send_motors_data(m_pwm0, m_pwm1);
    trigger_motors_update();

    return motors_stopped;
}

void
Robot_Controller::reset()
{
    m_distance_setpoint = 0.0f;
    m_rotate_setpoint_ramp.reset();
    DataManager::instance().reset();
    m_trajectory_manager.reset();
    Control_Loop::instance().reset();
}

#ifdef CONFIG_BLUETOOTH_DRV
void
Robot_Controller::handle_ble_packet(BLE_Protocol::Received_Packet const& received_packet)
{
    using BLE_Protocol::Command_Status;
    using BLE_Protocol::Controller_Id;
    using BLE_Protocol::Message_Type;

    Command_Status status = Command_Status::OK;
    BLE_Protocol::Payload_Reader reader(received_packet.payload, received_packet.payload_length);
    switch(received_packet.type)
    {
        case Message_Type::STATE_COMMAND:
        {
            uint8_t action {};
            if(!reader.get_u8(action) || !reader.done())
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            bool const applied =
                Main_State_Machine::instance().apply_command(static_cast<BLE_Protocol::State_Action>(action));
            status = applied ? Command_Status::OK : Command_Status::INVALID_STATE;
            break;
        }
        case Message_Type::GET_PID_STATE:
        {
            if(!reader.done())
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            if(!m_regulator_message_sending_in_progress)
            {
                m_regulator_message_sending_in_progress = true;
                Control_Loop::instance().send_PID_controllers_parameters();
                m_regulator_message_sending_in_progress = false;
            }
            break;
        }
        case Message_Type::SET_PID:
        {
            status = Control_Loop::instance().set_PID_parameters(reader);
            break;
        }
        case Message_Type::SET_SETPOINT:
        {
            uint8_t controller_value {};
            float value {};
            if(!reader.get_u8(controller_value) || !reader.get_float(value) || !reader.done())
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            Controller_Id const controller = static_cast<Controller_Id>(controller_value);
            if(!isfinite(value))
            {
                status = Command_Status::INVALID_VALUE;
                break;
            }
            switch(controller)
            {
                case Controller_Id::DISTANCE:
                    if(m_trajectory_manager.trajectory_started())
                    {
                        status = Command_Status::INVALID_STATE;
                    }
                    else
                    {
                        m_distance_setpoint = value;
                    }
                    break;
                case Controller_Id::BALANCE:
                    m_balance_setpoint = value * (PI / RADIAN_IN_DEGREES);
                    break;
                case Controller_Id::ROTATE:
                    if(m_trajectory_manager.trajectory_started())
                    {
                        status = Command_Status::INVALID_STATE;
                    }
                    else
                    {
                        m_rotate_setpoint_ramp.set_target(value * (PI / RADIAN_IN_DEGREES));
                    }
                    break;
                default:
                    status = Command_Status::INVALID_VALUE;
                    break;
            }
            break;
        }
        case Message_Type::TRAJECTORY_COMMAND:
        {
            float rotation_degrees {};
            float distance_m {};
            if(!reader.get_float(rotation_degrees) || !reader.get_float(distance_m) || !reader.done())
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            bool const accepted = m_trajectory_manager.set_trajectory_point(rotation_degrees, distance_m);
            status              = accepted ? Command_Status::OK : Command_Status::INVALID_STATE;
            break;
        }
        case Message_Type::SET_LQR:
        {
#ifndef CONFIG_PID_ENABLED
            status = Control_Loop::instance().set_LQR_parameters(reader);
#else
            status = Command_Status::UNSUPPORTED_MESSAGE;
#endif
            break;
        }
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
        case Message_Type::IDENTIFICATION_CONFIG:
            if(received_packet.payload_length != (10u * 2u * sizeof(float)))
            {
                status = Command_Status::INVALID_LENGTH;
            }
            else
            {
                status = Model_Identification::instance().set_identification_profile(
                             received_packet.payload, received_packet.payload_length) ?
                             Command_Status::OK :
                             Command_Status::INVALID_VALUE;
            }
            break;
#endif
        default:
            status = Command_Status::UNSUPPORTED_MESSAGE;
            break;
    }

    send_command_result(received_packet, status);
}

#endif  // CONFIG_BLUETOOTH_DRV

void
Robot_Controller::send_motors_data(float pwm_motor0, float pwm_motor1)
{
    set_start_motors(true);
    set_duty_cycle_value(static_cast<int>(pwm_motor0), static_cast<int>(pwm_motor1));
}

#ifdef CONFIG_VALIDATE_ROBOT_ANGLE
bool
Robot_Controller::validate_robot_angle(float balance_angle)
{
    static bool disable_motors_command           = false;
    static constexpr float safe_angle_margin     = 20.0f * (PI / RADIAN_IN_DEGREES);
    static constexpr float safe_angle_hysteresis = 0.5f * (PI / RADIAN_IN_DEGREES);

    float const upper_limit = m_balance_setpoint + safe_angle_margin;
    float const lower_limit = m_balance_setpoint - safe_angle_margin;

    if(!disable_motors_command && (balance_angle > upper_limit || balance_angle < lower_limit))
    {
        disable_motors_command = true;
        return disable_motors_command;
    }

    if(disable_motors_command && (balance_angle < (upper_limit - safe_angle_hysteresis)) &&
       (balance_angle > (lower_limit + safe_angle_hysteresis)))
    {
        disable_motors_command = false;
    }
    return disable_motors_command;
}
#endif  // CONFIG_VALIDATE_ROBOT_ANGLE

bool
Robot_Controller::ramp_pwm_to_stop(float& pwm)
{
    static constexpr float pwm_stop_target = 0.0f;
    static constexpr float pwm_ramp_step   = 0.5f;
    float const pwm_diff                   = pwm_stop_target - pwm;

    bool motor_stopped = false;
    if(pwm_diff > pwm_ramp_step)
    {
        pwm += pwm_ramp_step;
    }
    else if(pwm_diff < -pwm_ramp_step)
    {
        pwm -= pwm_ramp_step;
    }
    else
    {
        pwm           = pwm_stop_target;
        motor_stopped = true;
    }

    return motor_stopped;
}

}  // namespace Robot_Control
