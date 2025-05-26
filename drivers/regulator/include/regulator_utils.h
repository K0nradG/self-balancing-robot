#ifndef REGULATOR_UTILS_H_
#define REGULATOR_UTILS_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define M_PI 3.14159265358979323846f
#define DEG_TO_RAD (M_PI / 180.0f)

float
limit(float input, float lower_bound, float upper_bound);

float
input_low_pass_filter(float input);

#ifdef __cplusplus
}
#endif

#endif /* REGULATOR_UTILS_H_ */
