#ifndef REGULATOR_H_
#define REGULATOR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pid_regulator_parameters
{
    float K;
    float I;
    float D;
    float setpoint;
};

typedef void (*regulator_params_updated_cb_t)(struct pid_regulator_parameters _pid_regulator_parameters);

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
