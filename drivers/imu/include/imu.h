#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct imu_data
{
    float angle_balance;
    float angle_balance_dt;
    float angle_rotation;
} imu_data;

typedef void (*imu_updated_cb_t)(imu_data data);

void
new_imu_cb_register(imu_updated_cb_t _new_imu_cb);

void
mpu_reset(uint8_t conf);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
