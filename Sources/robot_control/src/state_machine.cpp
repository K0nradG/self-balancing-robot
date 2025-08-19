#include "state_machine.h"

namespace Robot_Control
{

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
            if(m_flags.button_pressed)
            {
                m_state = NORMAL_OPERATION;
            }
            break;
        case NORMAL_OPERATION:
            if(m_flags.disable_motors_command || m_flags.button_pressed)
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

    // Explicitly reset the flag.
    m_flags.button_pressed = false;
}

State_Machine::State
State_Machine::get_state() const
{
    return m_state;
}

void
State_Machine::set_button_pressed(bool button_pressed)
{
    m_flags.button_pressed = button_pressed;
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

}  // namespace Robot_Control