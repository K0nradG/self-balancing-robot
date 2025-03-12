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

float
calculate_pid_output(float error);

typedef void (*pid_params_updated_cb_t)(struct pid_regulator_parameters _pid_regulator_parameters);

#ifdef CONFIG_LOG_OVER_BLE
void
new_nus_parameters_received_for_pid(const uint8_t* data, uint16_t len);
#endif  // CONFIG_LOG_OVER_BLE

void
new_pid_parameters_cb_register(pid_params_updated_cb_t _new_pid_parameters_cb);

float
get_setpoint_pid(void);

#ifdef __cplusplus
}
#endif

#endif /* PID_H_ */
