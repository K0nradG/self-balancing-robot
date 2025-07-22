#include "robot_controller.h"
#include "motor_controller.h"

void
Robot_Controller::update_imu_data()
{
    m_imu_data = get_imu_data();
}

void
Robot_Controller::control_motors()
{
    update_imu_data();

    float const balance_error = m_balance_setpoint - m_imu_data.angle_balance;

    float const pwm = m_balance_pid.calculate_output(balance_error);

    send_motors_data(static_cast<int>(pwm), static_cast<int>(pwm));
}

void
Robot_Controller::send_motors_data(int pwm_motor0, int pwm_motor1)
{
    set_start_motors(true);
    set_duty_cycle_value(pwm_motor0, pwm_motor1);
}

void
Robot_Controller::parse_pid_params(char const* data)
{
    if(*data == 'b')
    {
        m_balance_pid.parse_nus_parameters(data);
    }
    // else if(*data == 'r')
    // {
    //     parse_pid_params(&g_rotation_regulator, ptr);
    // }
}