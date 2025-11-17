#pragma once

/*These are the commands which comes from the ble central (control device - phone or 7002DK) to interact with robot*/

// DFU commands
#define DFU_PREFIX 'd'

#define DFU_START_CMD 'b'
#define DFU_SKIP_CMD  's'

// Regulator commands
#define REG_DISTANCE_PID_PREFIX     'f'
#define REG_LINEAR_SPEED_PID_PREFIX 's'
#define REG_BALANCE_PID_PREFIX      'b'
#define REG_ROTATE_PID_PREFIX       'r'
#define REG_WHEEL_PID_PREFIX        'w'

#define REG_SETPOINT_CMD         's'
#define REG_PID_K_GAIN_CMD       'k'
#define REG_PID_I_GAIN_CMD       'i'
#define REG_PID_D_GAIN_CMD       'd'
#define REG_PID_FILTER_ALPHA_CMD 'f'

// State machine commands
#define STATE_MACHINE_PREFIX 'm'

#define STATE_MACHINE_START 'b'
#define STATE_MACHINE_STOP  's'

// Trajectory manager commands
#define TRAJECTORY_MANAGER_PREFIX 't'

#define TRAJECTORY_MANAGER_ROTATION 'r'
#define TRAJECTORY_MANAGER_DISTANCE 'f'