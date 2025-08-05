#include "robot_controller.h"
#include "data_manager.h"
#include "motor_controller.h"
#include "supervisor.h"

#ifdef CONFIG_ROBOT_CONTROL_LOG
#include "logger.h"
#endif  // CONFIG_ROBOT_CONTROL_LOG

namespace Robot_Control
{

Robot_Controller::Robot_Controller()
    : m_balance_setpoint(balance_setpoint),
      m_rotate_setpoint(rotate_setpoint),
#ifdef CONFIG_PID_ENABLED
      m_balance_pid(
          balance_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT),
          balance_pid_filter_alpha),  // Some other limit if it doesn't produce PWM directly?
#else
      m_balance_lqr(balance_lqr_parameters, static_cast<float>(CONFIG_PWM_LIMIT)),
#endif
      m_wheel0_speed_pid(wheel_speed_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), speed_pid_filter_alpha),
      m_wheel1_speed_pid(wheel_speed_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), speed_pid_filter_alpha),
      m_rotate_pid(rotate_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), rotate_pid_filter_alpha)
{
}

void
Robot_Controller::control_motors()
{
    DataManager::instance().update();
    imu_data const imu_data           = DataManager::instance().get_imu_data();
    encoders_data const encoders_data = DataManager::instance().get_encoders_data();

    safety_supervisor((imu_data.angle_balance + m_balance_setpoint));

#ifdef CONFIG_PID_ENABLED
    float const target_speed = m_balance_pid.calculate_output(m_balance_setpoint, imu_data.angle_balance);
#else
    float const target_speed = m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif

    float const target_rotate_speed = m_rotate_pid.calculate_output(m_rotate_setpoint, imu_data.angle_rotation);

    float const pwm0 = m_wheel0_speed_pid.calculate_output(
        target_speed - target_rotate_speed, encoders_data.encoder_0.angular_velocity_rad_s);
    float const pwm1 = m_wheel1_speed_pid.calculate_output(
        target_speed + target_rotate_speed, encoders_data.encoder_1.angular_velocity_rad_s);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    m_identification_data = {
        .dt       = static_cast<float>(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME) / 1000.0f,
        .pwm      = (pwm0 + pwm1) / 2.0f,
        .angle    = imu_data.angle_balance,
        .angle_dt = imu_data.angle_balance_dt};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log(
        "APP", LOG_LEVEL_INF, "ab: %f, ev0: %f, ev1: %f, pwm0: %f, pwm1: %f",
        (double)(imu_data.angle_balance * radian_degrees / pi), (double)encoders_data.encoder_0.shaft_angle_rad,
        (double)encoders_data.encoder_1.shaft_angle_rad, (double)pwm0, (double)pwm1);
#endif  // CONFIG_ROBOT_CONTROL_LOG
    send_motors_data(static_cast<int>(pwm0), static_cast<int>(pwm1));
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

    char key            = data[0];
    const char* payload = data + 1;

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
Robot_Controller::send_motors_data(int pwm_motor0, int pwm_motor1)
{
    set_start_motors(true);
    set_duty_cycle_value(pwm_motor0, pwm_motor1);
}

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
identification_data
Robot_Controller::get_identification_data() const
{
    return m_identification_data;
}
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

}  // namespace Robot_Control
