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
#endif // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
void
new_imu_angle_for_regulator(struct identification_data data);

#else
void
new_imu_angle_for_regulator(float _angle);
#endif // CONFIG_MODEL_IDENTIFICATION_DRV

#ifdef CONFIG_PID_ENABLED
typedef float (*calculate_regulator_output_cb_t)(float angle);
#else
typedef float (*calculate_regulator_output_cb_t)(float angle, float angle_dt);
#endif // CONFIG_PID_ENABLED

void
new_calculate_regulator_output_cb_register(calculate_regulator_output_cb_t _new_calculate_regulator_output_cb);

typedef float (*get_setpoint_cb_t)(void);

void
new_get_setpoint_cb_register(get_setpoint_cb_t _new_get_setpoint_cb);

void
regulator_start_automatic_control(void);

void
regulator_stop_automatic_control(void);

#ifdef __cplusplus
}
#endif

#endif /* REGULATOR_H_ */
