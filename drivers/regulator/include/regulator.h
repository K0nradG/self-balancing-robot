#ifndef REGULATOR_H_
#define REGULATOR_H_

#include <stdint.h>
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_MODEL_IDENTIFICATION_DRV)
struct identification_data
{
    float dt;
    float pwm;
    float angle;
    float angle_dt;
};

typedef void (*send_identification_data_cb_t)(struct identification_data data);

void
new_send_identification_data_cb_register(send_identification_data_cb_t new_send_identification_data_cb);
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

void
new_imu_data_for_regulator(imu_data imu_data);

#ifdef CONFIG_PID_ENABLED
typedef float (*calculate_regulator_output_cb_t)(float error);
#else
typedef float (*calculate_regulator_output_cb_t)(float angle, float angle_dt);
#endif  // CONFIG_PID_ENABLED

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
