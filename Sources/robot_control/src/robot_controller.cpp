// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "robot_controller.h"
#include <math.h>
#include "ble_protocol.h"
#include "ble_service.h"
#include "data_manager.h"
#include "logger.h"
#include "main_state_machine.h"
#include "motor_controller.h"
#include "saturation.h"
#include "zephyr/kernel.h"

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
send_command_result(BLE_Protocol::Packet_View const& packet, BLE_Protocol::Command_Status status)
{
    uint8_t payload[6] {};
    BLE_Protocol::put_u32(payload, packet.sequence);
    payload[4] = static_cast<uint8_t>(packet.type);
    payload[5] = static_cast<uint8_t>(status);
    ble_send_packet(BLE_Protocol::Message_Type::COMMAND_RESULT, payload, sizeof(payload));
}

static void
PID_controllers_data_sending_work_handler(k_work* work)
{
    ARG_UNUSED(work);

    Robot_Controller::instance().send_PID_controllers_parameters();
}

static K_WORK_DELAYABLE_DEFINE(s_PID_controllers_data_sending_work, PID_controllers_data_sending_work_handler);

Robot_Controller::Robot_Controller()
    : m_distance_setpoint(0.0f),
      m_balance_setpoint(balance_setpoint),
      m_rotate_setpoint_ramp(rotate_setpoint_rate),
      m_trajectory_manager(m_distance_setpoint, m_rotate_setpoint_ramp),
      m_distance_pid(
          distance_pid_parameters, Saturation(-max_linear_speed, max_linear_speed), distance_pid_filter_alpha,
          distance_pid_hysteresis),
      m_linear_speed_pid(
          linear_speed_pid_parameters, Saturation(angle_backward_max_deviation, angle_forward_max_deviation),
          linear_speed_pid_filter_alpha),
#ifdef CONFIG_PID_ENABLED
      m_balance_pid(balance_pid_parameters, Saturation(-max_speed_rad_s, max_speed_rad_s), balance_pid_filter_alpha),
#else
      m_balance_lqr(balance_lqr_parameters, max_speed_rad_s),
#endif  // not CONFIG_PID_ENABLED
      m_rotate_pid(
          rotate_pid_parameters, Saturation(-max_speed_rad_s, max_speed_rad_s), rotate_pid_filter_alpha,
          rotate_pid_hysteresis),
      m_wheel0_speed_pid(
          wheel_speed_pid_parameters,
          Saturation(-static_cast<float>(CONFIG_PWM_LIMIT), static_cast<float>(CONFIG_PWM_LIMIT)),
          wheel_speed_pid_filter_alpha),
      m_wheel1_speed_pid(
          wheel_speed_pid_parameters,
          Saturation(-static_cast<float>(CONFIG_PWM_LIMIT), static_cast<float>(CONFIG_PWM_LIMIT)),
          wheel_speed_pid_filter_alpha),
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

    float const rotation_angle = DataManager::instance().get_rotation_angle();

#ifdef CONFIG_VALIDATE_ROBOT_ANGLE
    bool const disable_motors_command = validate_robot_angle(imu_data.angle_balance);
#else
    bool const disable_motors_command = false;
#endif  // CONFIG_VALIDATE_ROBOT_ANGLE

    if(!disable_motors_command)
    {
        m_trajectory_manager.update(rotation_angle, encoders_data.robot_distance_m);

        float const target_linear_speed =
            m_distance_pid.calculate_output(m_distance_setpoint, encoders_data.robot_distance_m, imu_data.time_dt);

        float const balance_angle_deviation = m_linear_speed_pid.calculate_output(
            target_linear_speed, encoders_data.robot_linear_speed, imu_data.time_dt);

#ifdef CONFIG_PID_ENABLED
        float const target_speed_balance = m_balance_pid.calculate_output(
            m_balance_setpoint - balance_angle_deviation, imu_data.angle_balance, imu_data.time_dt);
#else
        ARG_UNUSED(balance_angle_deviation);
        float const target_speed_balance =
            m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif  // CONFIG_PID_ENABLED

        m_rotate_setpoint_ramp.update(imu_data.time_dt);
        float const target_speed_rotate =
            m_rotate_pid.calculate_output(m_rotate_setpoint_ramp.get_current_value(), rotation_angle, imu_data.time_dt);

        static Saturation const target_wheel_speed_saturation {-max_speed_rad_s, max_speed_rad_s};
        float const target_speed0 = target_wheel_speed_saturation.saturate(target_speed_balance - target_speed_rotate);
        float const target_speed1 = target_wheel_speed_saturation.saturate(target_speed_balance + target_speed_rotate);

        m_pwm0 = m_wheel0_speed_pid.calculate_output(
            target_speed0, encoders_data.encoder_0.angular_velocity_rad_s, imu_data.time_dt);
        m_pwm1 = m_wheel1_speed_pid.calculate_output(
            target_speed1, encoders_data.encoder_1.angular_velocity_rad_s, imu_data.time_dt);

        if(!m_trajectory_manager.stop_logs())
        {
#if defined(CONFIG_ROBOT_CONTROL_LOG) && defined(CONFIG_BLUETOOTH_DRV)
            Telemetry_Sample const telemetry_sample = {
                .timestamp_us      = k_uptime_get_32() * 1000u,
                .balance_setpoint  = m_balance_setpoint * radian_degrees / pi,
                .balance_angle     = imu_data.angle_balance * radian_degrees / pi,
                .rotation_setpoint = m_rotate_setpoint_ramp.get_current_value() * radian_degrees / pi,
                .rotation_angle    = rotation_angle * radian_degrees / pi,
                .target_speed_0    = target_speed0,
                .target_speed_1    = target_speed1,
                .measured_speed_0  = encoders_data.encoder_0.angular_velocity_rad_s,
                .measured_speed_1  = encoders_data.encoder_1.angular_velocity_rad_s,
                .pwm_0             = m_pwm0,
                .pwm_1             = m_pwm1,
            };
            telemetry_submit(telemetry_sample);
#else
            static float log_timer_ms = 0.0f;
            log_timer_ms += imu_data.time_dt * 1000;

            if(log_timer_ms >= CONFIG_ROBOT_CONTROL_LOG_NUS_PERIOD_MS)
            {
                log_timer_ms = 0.0f;

                robot_control_logger.platform_log(
                    LOG_LEVEL::INF,
                    "bs: %f, ab: %f, rs: %f, ar: %f, ts0: %f, ts1: %f, s0: %f, s1: %f, pwm0: %f, pwm1: %f",
                    (double)(m_balance_setpoint * radian_degrees / pi),
                    (double)(imu_data.angle_balance * radian_degrees / pi),
                    (double)(m_rotate_setpoint_ramp.get_current_value() * radian_degrees / pi),
                    (double)(rotation_angle * radian_degrees / pi), (double)target_speed0, (double)target_speed1,
                    (double)encoders_data.encoder_0.angular_velocity_rad_s,
                    (double)encoders_data.encoder_1.angular_velocity_rad_s, (double)m_pwm0, (double)m_pwm1);
            }
#endif
        }
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

    m_wheel0_speed_pid.reset();
    m_wheel1_speed_pid.reset();
    m_rotate_pid.reset();
#ifdef CONFIG_PID_ENABLED
    m_balance_pid.reset();
#endif
    m_trajectory_manager.reset();
}

#ifdef CONFIG_BLUETOOTH_DRV
void
Robot_Controller::handle_ble_packet(BLE_Protocol::Packet_View const& packet)
{
    using BLE_Protocol::Command_Status;
    using BLE_Protocol::Controller_Id;
    using BLE_Protocol::Message_Type;

    Command_Status status = Command_Status::OK;
    switch(packet.type)
    {
        case Message_Type::STATE_COMMAND:
        {
            if(packet.payload_length != 1u)
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            bool const applied = Main_State_Machine::instance().apply_command(
                static_cast<BLE_Protocol::State_Action>(packet.payload[0]));
            status = applied ? Command_Status::OK : Command_Status::INVALID_STATE;
            break;
        }
        case Message_Type::GET_PID_STATE:
        {
            if(packet.payload_length != 0u)
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            if(!m_regulator_message_sending_in_progress)
            {
                m_regulator_message_sending_in_progress = true;
                k_work_submit(&s_PID_controllers_data_sending_work.work);
            }
            break;
        }
        case Message_Type::SET_PID:
        {
            if(packet.payload_length != 13u)
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            Controller_Id const controller   = static_cast<Controller_Id>(packet.payload[0]);
            PID::Parameters const parameters = {
                .Kp = BLE_Protocol::get_float(packet.payload + 1u),
                .Ki = BLE_Protocol::get_float(packet.payload + 5u),
                .Kd = BLE_Protocol::get_float(packet.payload + 9u),
            };
            if(!isfinite(parameters.Kp) || !isfinite(parameters.Ki) || !isfinite(parameters.Kd))
            {
                status = Command_Status::INVALID_VALUE;
                break;
            }
            switch(controller)
            {
                case Controller_Id::DISTANCE:
                    m_distance_pid.set_parameters(parameters);
                    break;
                case Controller_Id::LINEAR_SPEED:
                    m_linear_speed_pid.set_parameters(parameters);
                    break;
                case Controller_Id::BALANCE:
#ifdef CONFIG_PID_ENABLED
                    m_balance_pid.set_parameters(parameters);
#else
                    status = Command_Status::UNSUPPORTED_MESSAGE;
#endif  // CONFIG_PID_ENABLED
                    break;
                case Controller_Id::ROTATE:
                    m_rotate_pid.set_parameters(parameters);
                    break;
                case Controller_Id::WHEEL_SPEED:
                    m_wheel0_speed_pid.set_parameters(parameters);
                    m_wheel1_speed_pid.set_parameters(parameters);
                    break;
                default:
                    status = Command_Status::INVALID_VALUE;
                    break;
            }
            break;
        }
        case Message_Type::SET_SETPOINT:
        {
            if(packet.payload_length != 5u)
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            Controller_Id const controller = static_cast<Controller_Id>(packet.payload[0]);
            float const value              = BLE_Protocol::get_float(packet.payload + 1u);
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
                    m_balance_setpoint = value * (pi / radian_degrees);
                    break;
                case Controller_Id::ROTATE:
                    if(m_trajectory_manager.trajectory_started())
                    {
                        status = Command_Status::INVALID_STATE;
                    }
                    else
                    {
                        m_rotate_setpoint_ramp.set_target(value * (pi / radian_degrees));
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
            if(packet.payload_length != 8u)
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            bool const accepted = m_trajectory_manager.set_trajectory_point(
                BLE_Protocol::get_float(packet.payload), BLE_Protocol::get_float(packet.payload + 4u));
            status = accepted ? Command_Status::OK : Command_Status::INVALID_STATE;
            break;
        }
        case Message_Type::SET_LQR:
        {
#ifndef CONFIG_PID_ENABLED
            if(packet.payload_length != 8u)
            {
                status = Command_Status::INVALID_LENGTH;
                break;
            }
            LQR::Parameters const parameters = {
                .Kx = BLE_Protocol::get_float(packet.payload),
                .Ky = BLE_Protocol::get_float(packet.payload + 4u),
            };
            if(!isfinite(parameters.Kx) || !isfinite(parameters.Ky))
            {
                status = Command_Status::INVALID_VALUE;
            }
            else
            {
                m_balance_lqr.set_parameters(parameters);
            }
#else
            status = Command_Status::UNSUPPORTED_MESSAGE;
#endif
            break;
        }
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
        case Message_Type::IDENTIFICATION_CONFIG:
            if(packet.payload_length != (10u * 2u * sizeof(float)))
            {
                status = Command_Status::INVALID_LENGTH;
            }
            else
            {
                status =
                    Model_Identification::instance().set_identification_profile(packet.payload, packet.payload_length) ?
                        Command_Status::OK :
                        Command_Status::INVALID_VALUE;
            }
            break;
#endif
        default:
            status = Command_Status::UNSUPPORTED_MESSAGE;
            break;
    }

    send_command_result(packet, status);
}

void
Robot_Controller::send_PID_controllers_parameters()
{
    PID::Parameters const distance_pid_parameters     = m_distance_pid.get_parameters();
    PID::Parameters const linear_speed_pid_parameters = m_linear_speed_pid.get_parameters();
#ifdef CONFIG_PID_ENABLED
    PID::Parameters const balance_pid_parameters = m_balance_pid.get_parameters();
#else
    PID::Parameters const balance_pid_parameters {};
#endif
    PID::Parameters const rotate_pid_parameters      = m_rotate_pid.get_parameters();
    PID::Parameters const wheel_speed_pid_parameters = m_wheel0_speed_pid.get_parameters();

    PID::Parameters const parameters[] = {
        distance_pid_parameters, linear_speed_pid_parameters, balance_pid_parameters,
        rotate_pid_parameters,   wheel_speed_pid_parameters,
    };
    uint8_t payload[sizeof(parameters)] {};
    size_t offset = 0u;
    for(PID::Parameters const& parameter: parameters)
    {
        BLE_Protocol::put_float(payload + offset, parameter.Kp);
        BLE_Protocol::put_float(payload + offset + 4u, parameter.Ki);
        BLE_Protocol::put_float(payload + offset + 8u, parameter.Kd);
        offset += 12u;
    }
    ble_send_packet(BLE_Protocol::Message_Type::PID_STATE, payload, sizeof(payload));
#ifndef CONFIG_PID_ENABLED
    LQR::Parameters const lqr_parameters = m_balance_lqr.get_parameters();
    uint8_t lqr_payload[8] {};
    BLE_Protocol::put_float(lqr_payload, lqr_parameters.Kx);
    BLE_Protocol::put_float(lqr_payload + 4u, lqr_parameters.Ky);
    ble_send_packet(BLE_Protocol::Message_Type::LQR_STATE, lqr_payload, sizeof(lqr_payload));
#endif
    m_regulator_message_sending_in_progress = false;
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
    static constexpr float safe_angle_margin     = 20.0f * (pi / radian_degrees);
    static constexpr float safe_angle_hysteresis = 0.5f * (pi / radian_degrees);

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
