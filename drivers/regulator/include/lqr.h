#ifndef LQR_H_
#define LQR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct lqr_parameters
{
    float Kx;
    float Ky;
    float setpoint;
};

typedef void (*lqr_params_updated_cb_t)(struct lqr_parameters _lqr_regulator_parameters);

void
new_lqr_parameters_cb_register(lqr_params_updated_cb_t _new_lqr_parameters_cb);

float
calculate_regulator_output(float angle, float angle_dt);

float
get_setpoint(void);

#ifdef CONFIG_LOG_OVER_BLE
void
parse_regulator_data(const char* data);
#endif  // CONFIG_LOG_OVER_BLE

#ifdef __cplusplus
}
#endif

#endif /* LQR_H_ */
