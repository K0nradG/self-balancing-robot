#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _encoder
{
    /*800 impulses per motor shaft rotate*/
    int32_t impulse_count;
    float shaft_rotate_count;
    float shaft_angle_rad;
    float distance_m;
};

struct encoder_data
{
    struct _encoder encoder_0;
    struct _encoder encoder_1;
};

typedef void (*encoder_data_updated_cb_t)(struct encoder_data encoder_data);

void
new_encoder_data_updated_cb_register(encoder_data_updated_cb_t _new_encoder_data_cb);

void
encoder_start_periodic_data_update();

void
encoder_stop_periodic_data_update();

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
