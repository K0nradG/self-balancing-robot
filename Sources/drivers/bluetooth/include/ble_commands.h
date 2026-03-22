// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

/*These are the commands which come from the ble central (control device - phone or 7002DK) to interact with robot*/

namespace BLE_Commands
{
enum General
{
    GET_REGULATOR_PARAMS = 'g'
};

enum Prefix
{
    DFU                = 'd',
    STATE_MACHINE      = 'm',
    TRAJECTORY_MANAGER = 't',
    DISTANCE_PID       = 'f',
    LINEAR_SPEED_PID   = 's',
    BALANCE_PID        = 'b',
    ROTATE_PID         = 'r',
    WHEEL_PID          = 'w',
    IDENTIFICATION     = 'i'
};

enum DFU
{
    DFU_START = 'b',
    DFU_SKIP  = 's'
};

enum Regulator
{
    SETPOINT         = 's',
    PID_K_GAIN       = 'k',
    PID_I_GAIN       = 'i',
    PID_D_GAIN       = 'd',
    PID_FILTER_ALPHA = 'f'
};

enum State_Machine
{
    START = 'b',
    STOP  = 's'
};

enum Trajectory_Manager
{
    ROTATION             = 'r',
    DISTANCE             = 'f',
    TRAJECTORY_COMPLETED = 'c'
};

}  // namespace BLE_Commands