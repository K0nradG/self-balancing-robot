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

typedef enum
{
    BALANCE_REGULATOR  = 0,
    ROTATION_REGULATOR = 1,
} SOURCE;

typedef struct MOTORS_DATA
{
    // DIRECTION direction;
    // SOURCE source;
    int8_t duty_cycle_percent_motor0;
    int8_t duty_cycle_percent_motor1;
    bool start;
} MOTORS_DATA;

void
set_enable_controller(bool enable);

void
set_start_motors(bool start);

void
set_direction(DIRECTION direction, SOURCE source);

void
set_duty_cycle_value(int8_t duty_cycle_percent_motor0, int8_t duty_cycle_percent_motor1);

void
trigger_motors_update(void);

void
stop_motors(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H_ */
