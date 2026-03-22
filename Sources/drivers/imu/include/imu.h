// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include <stdint.h>
#include <zephyr/drivers/sensor.h>

struct imu_data
{
    float angle_balance;
    float angle_balance_dt;
    float angle_rotation_dt;
    float time_dt;
};

int
imu_init();

void
mpu_reset(uint8_t conf);

// _ added since the same naming was used in data manager - causes wrong calls.
imu_data
_get_imu_data();

//! @brief Resets the IMU static balance angle on next update call
void
reset_imu_balance_angle();