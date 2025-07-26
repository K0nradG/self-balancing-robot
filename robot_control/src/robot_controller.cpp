#include "robot_controller.h"
#include <cmath>
#include "encoder.h"
#include "imu.h"
#include "motor_controller.h"

encoders_data g__encoders_data = {0};

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
    imu_data const imu_data = get_imu_data();
    get_encoders_data();

#ifdef CONFIG_PID_ENABLED
    // float target_speed = m_balance_pid.calculate_output(m_balance_setpoint, imu_data.angle_balance);

    // float non_linear_gain = fminf(1.0f + fabs(imu_data.angle_balance) * 0.82f, 5.0f);
    // target_speed *= non_linear_gain;

    float angle_rel = imu_data.angle_balance - m_balance_setpoint;

    float target_speed = m_balance_pid.calculate_output(0.0f, angle_rel);
    target_speed += imu_data.angle_balance_dt * 1.5f;

    float non_linear_gain = fminf(1.0f + fabs(angle_rel) * 0.82f, 5.0f);
    target_speed *= non_linear_gain;

    // Ratunkowe minimum przy dużym przechyle
    if(fabs(angle_rel) > 8.0f && fabs(target_speed) < 1.5f)
    {
        target_speed = copysignf(1.5f, angle_rel);
    }

#else
    float const target_speed = m_balance_lqr.calculate_output(imu_data.angle_balance, imu_data.angle_balance_dt);
#endif

    float const pwm0 =
        m_wheel0_speed_pid.calculate_output(target_speed, g__encoders_data.encoder_0.angular_velocity_rad_s);
    float const pwm1 =
        m_wheel1_speed_pid.calculate_output(target_speed, g__encoders_data.encoder_1.angular_velocity_rad_s);

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
    m_identification_data = {
        .dt       = static_cast<float>(CONFIG_BALANCE_REGULATOR_SAMPLE_TIME) / 1000.0f,
        .pwm      = (pwm0 + pwm1) / 2.0f,
        .angle    = imu_data.angle_balance,
        .angle_dt = imu_data.angle_balance_dt};
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
    send_motors_data(0, 0);
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
                m_balance_setpoint = val;
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

void
new_encoder_data_callback(encoders_data encoders_data)
{
    g__encoders_data = encoders_data;
}

static int
init(void)
{
    new_encoder_data_updated_cb_register(new_encoder_data_callback);

#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log("REGULATOR", LOG_LEVEL_INF, "controller finished");
#endif  // CONFIG_ROBOT_CONTROL_LOG
    return 0;
}

SYS_INIT(init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);