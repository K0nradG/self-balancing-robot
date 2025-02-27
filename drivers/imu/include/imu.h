#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct imu_data
{
    struct angle
    {
        int angle_int;    // integral part of an angle
        int angle_fract;  // fract part of an angle
    } angle_data;
};

typedef void (*imu_updated_cb_t)(struct imu_data _imu_data);

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
