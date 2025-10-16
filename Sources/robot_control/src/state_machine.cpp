#include "state_machine.h"
#include "ble_commands.h"

namespace Robot_Control
{

State_Machine::Transition_Flags State_Machine::m_flags {};

void
State_Machine::update()
{
    if(m_state == IDENTIFICATION)
    {
        return;
    }

    switch(m_state)
    {
        case READY_TO_START:
            if(m_flags.start)
            {
                m_state       = NORMAL_OPERATION;
                m_flags.start = false;
            }
            break;
        case NORMAL_OPERATION:
            if(m_flags.disable_motors_command || m_flags.stop)
            {
                m_state                        = SOFT_STOP;
                m_flags.stop                   = false;
                m_flags.disable_motors_command = false;
            }
            break;
        case SOFT_STOP:
            if(m_flags.motors_stopped)
            {
                m_state                = RESET_AFTER_STOP;
                m_flags.motors_stopped = false;
            }
            break;
        case RESET_AFTER_STOP:
            m_state = READY_TO_START;
            break;
        default:
            break;
    }
}

State_Machine::State
State_Machine::get_state() const
{
    return m_state;
}

void
State_Machine::set_disable_motors_command(bool disable_motors_command)
{
    m_flags.disable_motors_command = disable_motors_command;
}

void
State_Machine::set_motors_stopped(bool motors_stopped)
{
    m_flags.motors_stopped = motors_stopped;
}

void
State_Machine::set_identification_state()
{
    m_state = IDENTIFICATION;
}

void
State_Machine::parse_nus_commands(char const* data)
{
    if(data == nullptr || *data == '\0')
    {
        return;
    }

    char const command = data[0];
    switch(command)
    {
        case STATE_MACHINE_START:
            m_flags.start = true;
            break;
        case STATE_MACHINE_STOP:
            m_flags.stop = true;
            break;
        default:
            break;
    }
}

}  // namespace Robot_Control