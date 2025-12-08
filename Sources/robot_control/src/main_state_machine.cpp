#include "main_state_machine.h"
#include "ble_commands.h"

namespace Robot_Control
{

void
Main_State_Machine::update()
{
    if(m_state == IDENTIFICATION)
    {
        return;
    }

    if(m_flags.disable_motors_command)  // Always handle emergency disable.
    {
        m_state = SOFT_STOP;
    }
    else
    {
        main_state_machine_update();
    }

    m_flags.reset();
}

void
Main_State_Machine::main_state_machine_update()
{
    switch(m_state)
    {
        case READY_TO_START:
            if(m_flags.swing_up_start)
            {
                m_state = SWING_UP;
            }
            else if(m_flags.normal_start)
            {
                m_state = NORMAL_OPERATION;
            }
            break;
        case SWING_UP:
            if(m_flags.swing_up_finished)
            {
                m_state = NORMAL_OPERATION;
            }
            break;
        case NORMAL_OPERATION:
            if(m_flags.stop)
            {
                m_state = SOFT_STOP;
            }
            break;
        case SOFT_STOP:
            if(m_flags.motors_stopped)
            {
                m_state = RESET_AFTER_STOP;
            }
            break;
        case RESET_AFTER_STOP:
            m_state = READY_TO_START;
            break;
        default:
            break;
    }
}

Main_State_Machine::State
Main_State_Machine::get_state() const
{
    return m_state;
}

void
Main_State_Machine::set_swing_up_finished(bool swing_up_finished)
{
    m_flags.swing_up_finished = swing_up_finished;
}

void
Main_State_Machine::set_disable_motors_command(bool disable_motors_command)
{
    m_flags.disable_motors_command = disable_motors_command;
}

void
Main_State_Machine::set_motors_stopped(bool motors_stopped)
{
    m_flags.motors_stopped = motors_stopped;
}

void
Main_State_Machine::set_identification_state()
{
    m_state = IDENTIFICATION;
}

void
Main_State_Machine::parse_nus_commands(char const* data)
{
    if((data == nullptr) || (*data == '\0'))
    {
        return;
    }

    char const command = data[0];
    switch(command)
    {
        case BLE_Commands::State_Machine::NORMAL_START:
            if(m_state == READY_TO_START)
            {
                m_flags.normal_start = true;
            }
            break;
        case BLE_Commands::State_Machine::SWING_UP_START:
            if(m_state == READY_TO_START)
            {
                m_flags.swing_up_start = true;
            }
            break;
        case BLE_Commands::State_Machine::STOP:
            if(m_state == NORMAL_OPERATION)
            {
                m_flags.stop = true;
            }
            break;
        default:
            break;
    }
}

}  // namespace Robot_Control