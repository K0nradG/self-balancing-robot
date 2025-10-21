#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float angle_balance;
    float angle_balance_dt;
    float angle_rotation_dt;
    float time_dt;
} imu_data;

int
imu_init(void);

void
mpu_reset(uint8_t conf);

// _ added since the same naming was used in data manager - causes wrong calls.
imu_data
_get_imu_data(void);

//! @brief Resets the IMU static balance angle on next update call
void
reset_imu_balance_angle(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
