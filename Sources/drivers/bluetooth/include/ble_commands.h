#ifndef BLE_CMD_H
#define BLE_CMD_H

/*These are the commands whihc comes from the ble central (control device - phone or 7002DK) to interact with robot*/

// DFU commands
#define DFU_PREFIX 'd'

#define DFU_START_CMD 'b'
#define DFU_SKIP_CMD  's'

// Regulator commands
#define REG_WHEEL_PID_PREFIX   's'
#define REG_BALANCE_PID_PREFIX 'b'
#define REG_ROTATE_PID_PREFIX  'r'

#define REG_SETPOINT_CMD   's'
#define REG_PID_K_GAIN_CMD 'k'
#define REG_PID_I_GAIN_CMD 'i'
#define REG_PID_D_GAIN_CMD 'd'

#endif /*BLE_CMD_H*/