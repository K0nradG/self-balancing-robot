#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*imu_updated_cb_t)(float angle);

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
