#include "robot_controller.h"
#include "imu.h"
#include "motor_controller.h"

namespace Robot_Control
{

Robot_Controller::Robot_Controller()
    : m_balance_setpoint(balance_setpoint),
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
    imu_data const imu_data = get_imu_data();

#ifdef CONFIG_PID_ENABLED
    float const pwm = m_balance_pid.calculate_output(m_balance_setpoint, imu_data.angle_balance);
#else
    float const pwm = m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    m_identification_data = {
        .dt       = static_cast<float>(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME) / 1000.0f,
        .pwm      = pwm,
        .angle    = imu_data.angle_balance,
        .angle_dt = imu_data.angle_balance_dt};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
    send_motors_data(static_cast<int>(pwm), static_cast<int>(pwm));
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

    if(*data == 'b')
    {
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
    }
    // else if(*data == 'r')
    // {
    //     parse_pid_params(&g_rotation_regulator, ptr);
    // }
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

} // namespace Robot_Control