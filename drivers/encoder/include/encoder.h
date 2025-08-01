#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /*800 impulses per motor shaft rotate*/
    int32_t impulse_count;
    float shaft_rotate_count;
    float shaft_angle_rad;
    float distance_m;
    float angular_velocity_rad_s;
    float linear_velocity_m_s;
} encoder_data;

typedef struct
{
    encoder_data encoder_0;
    encoder_data encoder_1;
    float robot_angle_rad;
} encoders_data;

// _ added since the same naming was used in data manager - causes wrong calls.
encoders_data
_get_encoders_data(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
