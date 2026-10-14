// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "control_loop.h"
#include <math.h>
#include "ble_payload_writer.h"
#include "ble_protocol_constants.h"
#include "ble_transfer_handler.h"
#include "zephyr/sys/util.h"

namespace Robot_Control
{

Control_Loop::Control_Loop()
    : m_distance_pid(
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
          wheel_speed_pid_filter_alpha)
{
}

Control_Loop::Output
Control_Loop::update(Setpoints const& setpoints, Feedback const& feedback, float time_dt)
{
#ifdef CONFIG_PID_ENABLED
    float const target_linear_speed =
        m_distance_pid.calculate_output(setpoints.distance, feedback.robot_distance_m, time_dt);

    float const balance_angle_deviation =
        m_linear_speed_pid.calculate_output(target_linear_speed, feedback.robot_linear_speed, time_dt);

    float const target_speed_balance =
        m_balance_pid.calculate_output(setpoints.balance - balance_angle_deviation, feedback.angle_balance, time_dt);
#else
    float const target_speed_balance =
        m_balance_lqr.calculate_output(feedback.angle_balance, feedback.angle_balance_dt);
#endif  // CONFIG_PID_ENABLED

    float const target_speed_rotate = m_rotate_pid.calculate_output(setpoints.rotate, feedback.rotation_angle, time_dt);

    static Saturation const target_wheel_speed_saturation {-max_speed_rad_s, max_speed_rad_s};
    float const target_speed0 = target_wheel_speed_saturation.saturate(target_speed_balance - target_speed_rotate);
    float const target_speed1 = target_wheel_speed_saturation.saturate(target_speed_balance + target_speed_rotate);

    float const pwm0 = m_wheel0_speed_pid.calculate_output(target_speed0, feedback.angular_velocity0_rad_s, time_dt);
    float const pwm1 = m_wheel1_speed_pid.calculate_output(target_speed1, feedback.angular_velocity1_rad_s, time_dt);

    return Output {target_speed0, target_speed1, pwm0, pwm1};
}

void
Control_Loop::send_PID_controllers_parameters()
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
    uint8_t payload[ARRAY_SIZE(parameters) * 3u * BLE_Protocol::ENCODED_FLOAT_SIZE] {};
    BLE_Protocol::Payload_Writer writer(payload, sizeof(payload));
    for(PID::Parameters const& parameter: parameters)
    {
        writer.put_float(parameter.Kp);
        writer.put_float(parameter.Ki);
        writer.put_float(parameter.Kd);
    }
    ble_send_packet(BLE_Protocol::Message_Type::PID_STATE, writer);
#ifndef CONFIG_PID_ENABLED
    LQR::Parameters const lqr_parameters = m_balance_lqr.get_parameters();
    uint8_t lqr_payload[2u * BLE_Protocol::ENCODED_FLOAT_SIZE] {};
    BLE_Protocol::Payload_Writer lqr_writer(lqr_payload, sizeof(lqr_payload));
    lqr_writer.put_float(lqr_parameters.Kx);
    lqr_writer.put_float(lqr_parameters.Ky);
    ble_send_packet(BLE_Protocol::Message_Type::LQR_STATE, lqr_writer);
#endif
}

BLE_Protocol::Command_Status
Control_Loop::set_PID_parameters(BLE_Protocol::Payload_Reader& reader)
{
    uint8_t controller_value {};
    PID::Parameters parameters {};
    if(!reader.get_u8(controller_value) || !reader.get_float(parameters.Kp) || !reader.get_float(parameters.Ki) ||
       !reader.get_float(parameters.Kd) || !reader.done())
    {
        return BLE_Protocol::Command_Status::INVALID_LENGTH;
    }

    auto const controller = static_cast<BLE_Protocol::Controller_Id>(controller_value);
    if(!isfinite(parameters.Kp) || !isfinite(parameters.Ki) || !isfinite(parameters.Kd))
    {
        return BLE_Protocol::Command_Status::INVALID_VALUE;
    }

    auto status = BLE_Protocol::Command_Status::OK;
    switch(controller)
    {
        case BLE_Protocol::Controller_Id::DISTANCE:
        {
            m_distance_pid.set_parameters(parameters);
            break;
        }
        case BLE_Protocol::Controller_Id::LINEAR_SPEED:
        {
            m_linear_speed_pid.set_parameters(parameters);
            break;
        }
        case BLE_Protocol::Controller_Id::BALANCE:
        {
#ifdef CONFIG_PID_ENABLED
            m_balance_pid.set_parameters(parameters);
#else
            status = BLE_Protocol::Command_Status::UNSUPPORTED_MESSAGE;
#endif  // CONFIG_PID_ENABLED
            break;
        }
        case BLE_Protocol::Controller_Id::ROTATE:
        {
            m_rotate_pid.set_parameters(parameters);
            break;
        }
        case BLE_Protocol::Controller_Id::WHEEL_SPEED:
        {
            m_wheel0_speed_pid.set_parameters(parameters);
            m_wheel1_speed_pid.set_parameters(parameters);
            break;
        }
        default:
        {
            status = BLE_Protocol::Command_Status::INVALID_VALUE;
            break;
        }
    }
    return status;
}

#ifndef CONFIG_PID_ENABLED
BLE_Protocol::Command_Status
Control_Loop::set_LQR_parameters(BLE_Protocol::Payload_Reader& reader)
{
    LQR::Parameters parameters {};
    if(!reader.get_float(parameters.Kx) || !reader.get_float(parameters.Ky) || !reader.done())
    {
        return BLE_Protocol::Command_Status::INVALID_LENGTH;
    }
    if(!isfinite(parameters.Kx) || !isfinite(parameters.Ky))
    {
        return BLE_Protocol::Command_Status::INVALID_VALUE;
    }

    m_balance_lqr.set_parameters(parameters);
    return BLE_Protocol::Command_Status::OK;
}
#endif  // not CONFIG_PID_ENABLED

void
Control_Loop::reset()
{
    m_wheel0_speed_pid.reset();
    m_wheel1_speed_pid.reset();
    m_rotate_pid.reset();

#ifdef CONFIG_PID_ENABLED
    m_balance_pid.reset();
#endif  // CONFIG_PID_ENABLED
}
}  // namespace Robot_Control