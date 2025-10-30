#include "robot_controller.h"
#include "data_manager.h"
#include "motor_controller.h"
#include "saturation.h"
#include "zephyr/sys/util.h"

#ifdef CONFIG_ROBOT_CONTROL_LOG
#include "logger.h"
#endif  // CONFIG_ROBOT_CONTROL_LOG

namespace Robot_Control
{

Robot_Controller::Robot_Controller()
    : m_distance_setpoint(0.0f),
      m_balance_setpoint(balance_setpoint),
      m_rotate_setpoint(rotate_setpoint_rate),
      m_distance_pid(
          distance_pid_parameters, Saturation(angle_backward_max_deviation, angle_forward_max_deviation),
          distance_pid_filter_alpha, distance_pid_hysteresis),
#ifdef CONFIG_PID_ENABLED
      m_balance_pid(balance_pid_parameters, Saturation(-max_speed_rad_s, max_speed_rad_s), balance_pid_filter_alpha),
#else
      m_balance_lqr(balance_lqr_parameters, max_speed_rad_s),
#endif  // CONFIG_PID_ENABLED
      m_rotate_pid(
          rotate_pid_parameters, Saturation(-max_speed_rad_s, max_speed_rad_s), rotate_pid_filter_alpha,
          rotate_pid_hysteresis),
      m_wheel0_speed_pid(
          wheel_speed_pid_parameters,
          Saturation(-static_cast<float>(CONFIG_PWM_LIMIT), static_cast<float>(CONFIG_PWM_LIMIT)),
          speed_pid_filter_alpha),
      m_wheel1_speed_pid(
          wheel_speed_pid_parameters,
          Saturation(-static_cast<float>(CONFIG_PWM_LIMIT), static_cast<float>(CONFIG_PWM_LIMIT)),
          speed_pid_filter_alpha)
{
}

bool
Robot_Controller::normal_motors_control()
{
    DataManager::instance().update();
    imu_data const imu_data           = DataManager::instance().get_imu_data();
    encoders_data const encoders_data = DataManager::instance().get_encoders_data();
    float const rotation_angle        = DataManager::instance().get_rotation_angle();

#ifdef CONFIG_VALIDATE_ROBOT_ANGLE
    bool const disable_motors_command = validate_robot_angle(imu_data.angle_balance);
#else
    bool const disable_motors_command = false;
#endif  // CONFIG_VALIDATE_ROBOT_ANGLE

    if(!disable_motors_command)
    {
        float const balance_angle_deviation =
            m_distance_pid.calculate_output(m_distance_setpoint, encoders_data.robot_distance_m, imu_data.time_dt);

#ifdef CONFIG_PID_ENABLED
        float const target_speed_balance = m_balance_pid.calculate_output(
            m_balance_setpoint - balance_angle_deviation, imu_data.angle_balance, imu_data.time_dt);
#else
        float const target_speed_balance =
            m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif  // CONFIG_PID_ENABLED

        m_rotate_setpoint.update(imu_data.time_dt);
        float const target_speed_rotate =
            m_rotate_pid.calculate_output(m_rotate_setpoint.get_current_value(), rotation_angle, imu_data.time_dt);

        float const target_speed0 =
            MAX(MIN(target_speed_balance - target_speed_rotate, max_speed_rad_s), -max_speed_rad_s);
        float const target_speed1 =
            MAX(MIN(target_speed_balance + target_speed_rotate, max_speed_rad_s), -max_speed_rad_s);

        m_pwm0 = m_wheel0_speed_pid.calculate_output(
            target_speed0, encoders_data.encoder_0.angular_velocity_rad_s, imu_data.time_dt);
        m_pwm1 = m_wheel1_speed_pid.calculate_output(
            target_speed1, encoders_data.encoder_1.angular_velocity_rad_s, imu_data.time_dt);

#ifdef CONFIG_ROBOT_CONTROL_LOG
        platform_log(
            "APP", LOG_LEVEL_INF,
            "ds: %f, d: %f, ad: %f, bs: %f, ab: %f, rs: %f, ar: %f, ts0: %f, ts1: %f, s0: %f, s1: %f, pwm0: %f, pwm1: "
            "%f",
            (double)m_distance_setpoint, (double)encoders_data.robot_distance_m,
            (double)(balance_angle_deviation * radian_degrees / pi), (double)(m_balance_setpoint * radian_degrees / pi),
            (double)(imu_data.angle_balance * radian_degrees / pi),
            (double)(m_rotate_setpoint.get_current_value() * radian_degrees / pi),
            (double)(rotation_angle * radian_degrees / pi), (double)target_speed0, (double)target_speed1,
            (double)encoders_data.encoder_0.angular_velocity_rad_s,
            (double)encoders_data.encoder_1.angular_velocity_rad_s, (double)m_pwm0, (double)m_pwm1);
#endif  // CONFIG_ROBOT_CONTROL_LOG
#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
        m_identification_data = {
            .dt       = static_cast<float>(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME) / 1000.0f,
            .pwm      = (pwm0 + pwm1) / 2.0f,
            .angle    = imu_data.angle_balance,
            .angle_dt = imu_data.angle_balance_dt};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

        send_motors_data(m_pwm0, m_pwm1);
        trigger_motors_update();
    }

    return disable_motors_command;
}

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
    m_rotate_setpoint.reset();
    DataManager::instance().reset();

    m_wheel0_speed_pid.reset();
    m_wheel1_speed_pid.reset();
    m_rotate_pid.reset();
    m_balance_pid.reset();
}

#ifdef CONFIG_BLUETOOTH_DRV
void
Robot_Controller::parse_nus_data(char const* data)
{
    if(data == nullptr || *data == '\0')
    {
        return;
    }

    char const key      = data[0];
    char const* payload = data + 1;

    switch(key)
    {
        case 'f':
            if(*payload == 's')
            {
                payload++;

                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%s", payload);

                char* endptr = nullptr;
                float val    = strtof(buffer, &endptr);
                m_distance_setpoint += val;
            }
            else
            {
                m_distance_pid.parse_nus_parameters(payload);
            }
            break;
        case 'b':
            if(*payload == 's')
            {
                payload++;

                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%s", payload);

                char* endptr       = nullptr;
                float val          = strtof(buffer, &endptr);
                m_balance_setpoint = val * (pi / radian_degrees);
            }
            else
            {
#ifdef CONFIG_PID_ENABLED
                m_balance_pid.parse_nus_parameters(payload);
#else
                m_balance_lqr.parse_nus_parameters(payload);
#endif  // CONFIG_PID_ENABLED
            }
            break;
        case 's':
            m_wheel0_speed_pid.parse_nus_parameters(payload);
            m_wheel1_speed_pid.set_parameters(m_wheel0_speed_pid.get_parameters());
            break;
        case 'r':
            if(*payload == 's')
            {
                payload++;

                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%s", payload);

                char* endptr = nullptr;
                float val    = strtof(buffer, &endptr);
                m_rotate_setpoint.set_target(m_rotate_setpoint.get_target() + (val * (pi / radian_degrees)));
            }
            else
            {
                m_rotate_pid.parse_nus_parameters(payload);
            }
            break;
        default:
            break;
    }
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

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
identification_data
Robot_Controller::get_identification_data() const
{
    return m_identification_data;
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

}  // namespace Robot_Control
