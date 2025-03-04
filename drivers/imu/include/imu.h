#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV

struct identification_data
{
    float angle;
    float angle_dt;
};

typedef void (*imu_updated_cb_t)(struct identification_data data);

#else

typedef void (*imu_updated_cb_t)(float angle);

#endif /*CONFIG_MODEL_IDENTIFICATION_DRV*/

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
