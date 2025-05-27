#ifndef PID_H_
#define PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pid_regulator_parameters
{
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
};

typedef void (*pid_params_updated_cb_t)(struct pid_regulator_parameters _pid_regulator_parameters);

void
new_pid_parameters_cb_register(pid_params_updated_cb_t _new_pid_parameters_cb);

float 
calculate_balance_regulator_output(float error);

float
calculate_rotation_regulator_output(float error);

float
get_balance_setpoint(void);

float
get_rotation_setpoint(void);

#ifdef CONFIG_LOG_OVER_BLE
void
parse_regulator_data(const char* data);
#endif  // CONFIG_LOG_OVER_BLE

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
