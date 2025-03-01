#ifndef IMU_CONFIG_H_
#define IMU_CONFIG_H_

#include <zephyr/drivers/i2c.h>

void
set_dlpf(const struct device* i2c_dev);

void
set_measurement_interval(const struct device* i2c_dev);

#endif /* IMU_CONFIG_H_ */
