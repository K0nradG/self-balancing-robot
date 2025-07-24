#include "robot_controller.h"
#include "encoder.h"
#include "imu.h"
#include "motor_controller.h"

namespace Robot_Control
{

Robot_Controller::Robot_Controller()
    : m_balance_setpoint(balance_setpoint),
#ifdef CONFIG_PID_ENABLED
      m_balance_pid(
          balance_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT),
          balance_pid_filter_alpha),  // Some other limit if it doesn't produce PWM directly?
#else
      m_balance_lqr(balance_lqr_parameters, static_cast<float>(CONFIG_PWM_LIMIT)),
#endif
      m_wheel0_speed_pid(wheel_speed_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), speed_pid_filter_alpha),
      m_wheel1_speed_pid(wheel_speed_pid_parameters, static_cast<float>(CONFIG_PWM_LIMIT), speed_pid_filter_alpha)
{
}

void
Robot_Controller::control_motors()
{
    imu_data const imu_data           = get_imu_data();
    encoders_data const encoders_data = get_encoders_data();

#ifdef CONFIG_PID_ENABLED
    float const target_speed = m_balance_pid.calculate_output(m_balance_setpoint, imu_data.angle_balance);
#else
    float const target_speed = m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif

    float const pwm0 =
        m_wheel0_speed_pid.calculate_output(target_speed, encoders_data.encoder_0.angular_velocity_rad_s);
    float const pwm1 =
        m_wheel1_speed_pid.calculate_output(target_speed, encoders_data.encoder_1.angular_velocity_rad_s);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    m_identification_data = {
        .dt       = static_cast<float>(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME) / 1000.0f,
        .pwm      = (pwm0 + pwm1) / 2.0f,
        .angle    = imu_data.angle_balance,
        .angle_dt = imu_data.angle_balance_dt};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
    send_motors_data(pwm0, pwm1);
    trigger_motors_update();
}

#ifdef CONFIG_LOG_OVER_BLE
void
Robot_Controller::parse_nus_data(char const* data)
{
    if(data == nullptr)
    {
        return;
    }

    char const key = *data;
    switch(key)
    {
        case 'b':
            data++;
            if(*data == 's')
            {
                data++;
                m_balance_setpoint = strtof(data, nullptr);
            }
            else
            {
#ifdef CONFIG_PID_ENABLED
                m_balance_pid.parse_nus_parameters(data);
#else
                m_balance_lqr.parse_nus_parameters(data);
#endif  // CONFIG_PID_ENABLED
            }
            break;
        case 's':
            data++;
            m_wheel0_speed_pid.parse_nus_parameters(data);
            m_wheel1_speed_pid.set_parameters(m_wheel0_speed_pid.get_parameters());
            break;
        // case 'r':
        //     break;
        default:
            break;
    }
}
#endif  // CONFIG_LOG_OVER_BLE

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