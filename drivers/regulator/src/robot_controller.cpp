#include "robot_controller.h"
#include "imu.h"
#include "motor_controller.h"

void
Robot_Controller::control_motors()
{
    imu_data const imu_data = get_imu_data();

    float const balance_error = m_balance_setpoint - imu_data.angle_balance;

    float const pwm = m_balance_pid.calculate_output(balance_error);

    send_motors_data(static_cast<int>(pwm), static_cast<int>(pwm));
    trigger_motors_update();
}

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
            m_balance_pid.parse_nus_parameters(data);
        }
    }
    // else if(*data == 'r')
    // {
    //     parse_pid_params(&g_rotation_regulator, ptr);
    // }
}

void
Robot_Controller::send_motors_data(int pwm_motor0, int pwm_motor1)
{
    set_start_motors(true);
    set_duty_cycle_value(pwm_motor0, pwm_motor1);
}