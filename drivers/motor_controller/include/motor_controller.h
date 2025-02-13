#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum DIRECTION
{
    POSITIVE = 0,
    NEGATIVE
} DIRECTION;

typedef struct MOTORS_DATA
{
    DIRECTION direction;
    float duty_cycle_f;
    bool start;
} MOTORS_DATA;

void
set_enable_controller(bool enable);

void
set_start_motors(bool start);

void
set_direction(DIRECTION direction);

void
set_duty_cycle_value(float duty_cycle_f);

void
motor_controller_start(void);

void
motor_controller_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H_ */
