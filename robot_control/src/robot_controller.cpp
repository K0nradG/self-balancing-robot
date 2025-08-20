#include "robot_controller.h"
#include "data_manager.h"
#include "motor_controller.h"

#ifdef CONFIG_ROBOT_CONTROL_LOG
#include "logger.h"
#endif  // CONFIG_ROBOT_CONTROL_LOG

namespace Robot_Control
{

Robot_Controller::Robot_Controller()
    : m_rotate_setpoint(rotate_setpoint),
      m_balance_setpoint(balance_setpoint),
      m_wheel0_speed_pid(wheel_speed_pid_parameters, max_speed_rad_s, speed_pid_filter_alpha),
      m_wheel1_speed_pid(wheel_speed_pid_parameters, max_speed_rad_s, speed_pid_filter_alpha),
      m_rotate_pid(rotate_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), rotate_pid_filter_alpha),
#ifdef CONFIG_PID_ENABLED
      m_balance_pid(balance_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), balance_pid_filter_alpha)
#else
      m_balance_lqr(balance_lqr_parameters, static_cast<float>(CONFIG_PWM_LIMIT))
#endif
{
}

void
Robot_Controller::control_motors()
{
    DataManager::instance().update();
    imu_data const imu_data           = DataManager::instance().get_imu_data();
    encoders_data const encoders_data = DataManager::instance().get_encoders_data();

    bool const disable_motors = validate_robot_angle(imu_data.angle_balance);
    if(disable_motors)
    {
        ramp_pwm_to_target(m_pwm0, 0.0f);
        ramp_pwm_to_target(m_pwm1, 0.0f);
    }
    else
    {
        // float const pwm0 = m_wheel0_speed_pid.calculate_output(
        //     target_speed - target_rotate_speed, encoders_data.encoder_0.angular_velocity_rad_s);
        // float const pwm1 = m_wheel1_speed_pid.calculate_output(
        //     target_speed + target_rotate_speed, encoders_data.encoder_1.angular_velocity_rad_s);

        float const pwm_rotate = 0.0f;  // m_rotate_pid.calculate_output(m_rotate_setpoint, imu_data.angle_rotation);

#ifdef CONFIG_PID_ENABLED
        float const pwm_balance = m_balance_pid.calculate_output(m_balance_setpoint, imu_data.angle_balance);
#else
        float const target_speed = m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif

        float const pwm0_target = pwm_balance - pwm_rotate;
        float const pwm1_target = pwm_balance + pwm_rotate;

        ramp_pwm_to_target(m_pwm0, pwm0_target);
        ramp_pwm_to_target(m_pwm1, pwm1_target);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
        m_identification_data = {
            .dt       = static_cast<float>(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME) / 1000.0f,
            .pwm      = (pwm0 + pwm1) / 2.0f,
            .angle    = imu_data.angle_balance,
            .angle_dt = imu_data.angle_balance_dt};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
    }

#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log(
        "APP", LOG_LEVEL_INF, "bs: %f, ab: %f, ar: %f, ea0: %f, ea1: %f, ev0: %f, ev1: %f, pwm0: %f, pwm1: %f",
        (double)(m_balance_setpoint * radian_degrees / pi), (double)(imu_data.angle_balance * radian_degrees / pi),
        (double)(imu_data.angle_rotation * radian_degrees / pi), (double)encoders_data.encoder_0.shaft_angle_rad,
        (double)encoders_data.encoder_1.shaft_angle_rad, (double)encoders_data.encoder_0.angular_velocity_rad_s,
        (double)encoders_data.encoder_1.angular_velocity_rad_s, (double)m_pwm0, (double)m_pwm1);
#endif  // CONFIG_ROBOT_CONTROL_LOG

    send_motors_data(m_pwm0, m_pwm1);
    trigger_motors_update();
}

#ifdef CONFIG_LOG_OVER_BLE
void
Robot_Controller::parse_nus_data(char const* data)
{
    if(data == nullptr || *data == '\0')
    {
        return;
    }

    char const key      = data[0];
    char const* payload = data++;

    switch(key)
    {
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
#endif
            }
            break;

        case 's':
            m_wheel0_speed_pid.parse_nus_parameters(payload);
            m_wheel1_speed_pid.set_parameters(m_wheel0_speed_pid.get_parameters());

        case 'r':
            if(*payload == 's')
            {
                payload++;

                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%s", payload);

                char* endptr      = nullptr;
                float val         = strtof(buffer, &endptr);
                m_rotate_setpoint = val * (pi / radian_degrees);
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
#endif

void
Robot_Controller::send_motors_data(float pwm_motor0, float pwm_motor1)
{
    set_start_motors(true);
    set_duty_cycle_value(static_cast<int>(pwm_motor0), static_cast<int>(pwm_motor1));
}

bool
Robot_Controller::validate_robot_angle(float balance_angle)
{
    static bool disable_motors                   = false;
    static constexpr float safe_angle_margin     = 30.0f * (pi / radian_degrees);
    static constexpr float safe_angle_hysteresis = 0.5f * (pi / radian_degrees);

    float const upper_limit = m_balance_setpoint + safe_angle_margin;
    float const lower_limit = m_balance_setpoint - safe_angle_margin;

    if(!disable_motors && (balance_angle > upper_limit || balance_angle < lower_limit))
    {
        disable_motors = true;
        return disable_motors;
    }

    if(disable_motors && (balance_angle < (upper_limit - safe_angle_hysteresis)) &&
       (balance_angle > (lower_limit + safe_angle_hysteresis)))
    {
        disable_motors = false;
    }
    return disable_motors;
}

void
Robot_Controller::ramp_pwm_to_target(float& pwm, float target_pwm)
{
    static constexpr float pwm_ramp_step = 1.0f;
    float const pwm_diff                 = target_pwm - pwm;

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
        pwm = target_pwm;
    }
}

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
identification_data
Robot_Controller::get_identification_data() const
{
    return m_identification_data;
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

}  // namespace Robot_Control
