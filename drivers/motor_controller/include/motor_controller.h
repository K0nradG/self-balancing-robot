#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum DIRECTION
{
    POSITIVE = 1,
    NEGATIVE = 0,
} DIRECTION;

typedef struct MOTORS_DATA
{
    DIRECTION direction;
    uint8_t duty_cycle_percent;
    bool start;
} MOTORS_DATA;

void
set_enable_controller(bool enable);

void
set_start_motors(bool start);

void
set_direction(DIRECTION direction);

void
set_duty_cycle_value(uint8_t duty_cycle_percent);

void
motor_controller_start(void);

void
motor_controller_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H_ */
