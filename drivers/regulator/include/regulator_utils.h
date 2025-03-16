#ifndef REGULATOR_UTILS_H_
#define REGULATOR_UTILS_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MS_TO_SECONDS 0.001f
#define M_PI 3.14159265358979323846f
#define N_coeff (2.0f * M_PI * (float)CONFIG_FILTER_CUTOFF_FREQUENCY)         // Filter coefficient in [rad/s]
#define N_dt (N_coeff * (float)CONFIG_REGULATOR_SAMPLE_TIME * MS_TO_SECONDS)  // [rad]
#define ALPHA      \
    N_dt / (1.0f + \
            N_dt)  // Alpha coefficients to be used directly by the low-pass filter: alpha = (N * dt) / (1 + N * dt)
#define DEG_TO_RAD (M_PI / 180.0f)
#define ANGLE_OFFSET 95.0f

float
limit(float input, float lower_bound, float upper_bound);

float
low_pass_filter(float input);

#ifdef __cplusplus
}
#endif

#endif /* REGULATOR_UTILS_H_ */
