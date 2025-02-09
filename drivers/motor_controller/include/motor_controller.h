#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum DIRECTION
{
    PLUS = 0,
    MINUS
} DIRECTION;

typedef struct MOTORS_DATA
{
    DIRECTION direction;
    int pwm_value;
    bool start;
} MOTORS_DATA;

static void
enable_controller(bool enable);

static void
start_stop_motor(bool start);

static void
set_direction(DIRECTION direction);

static void
set_pwm_value(int pwm_value);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H_ */
