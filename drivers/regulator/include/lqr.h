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

float
calculate_regulator_output(float angle, float angle_dt);

typedef void (*lqr_params_updated_cb_t)(struct lqr_parameters _lqr_regulator_parameters);

#ifdef CONFIG_LOG_OVER_BLE
void
new_nus_parameters_received_for_regulator(const uint8_t* data, uint16_t len);
#endif  // CONFIG_LOG_OVER_BLE

void
new_lqr_parameters_cb_register(lqr_params_updated_cb_t _new_lqr_parameters_cb);

float
get_setpoint(void);

#ifdef __cplusplus
}
#endif

#endif /* LQR_H_ */
