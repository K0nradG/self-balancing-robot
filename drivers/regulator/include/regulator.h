#ifndef REGULATOR_H_
#define REGULATOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "imu.h"

struct identification_regulator_data
{
    float dt;
    float pwm;
    float angle;
    float angle_dt;
};

typedef void (*regulator_data_updated_cb_t)(struct identification_regulator_data data);

void
new_pwm_cb_register(regulator_data_updated_cb_t _new_pwm_cb);
#endif

struct pid_regulator_parameters
{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
};

typedef void (*regulator_params_updated_cb_t)(struct pid_regulator_parameters _pid_regulator_parameters);

#ifdef CONFIG_LOG_OVER_BLE
void
new_nus_parameters_received_for_regulator(const uint8_t* data, uint16_t len);
#endif  // CONFIG_LOG_OVER_BLE

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
void
new_imu_angle_for_regulator(struct identification_data data);

#else
void
new_imu_angle_for_regulator(float _angle);
#endif

void
new_pid_regulator_parameters_cb_register(regulator_params_updated_cb_t _new_pid_regulator_parameters_cb);

void
regulator_start_automatic_control(void);

void
regulator_stop_automatic_control(void);

#ifdef __cplusplus
}
#endif

#endif /* REGULATOR_H_ */
