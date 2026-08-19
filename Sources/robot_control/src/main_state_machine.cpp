// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "main_state_machine.h"

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
#include "model_identification.h"
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV

namespace Robot_Control
{

void
Main_State_Machine::update()
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
                m_state = OPERATION;
            }
            break;
        case OPERATION:
            if(m_flags.disable_motors_command || m_flags.stop)
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

    m_flags.reset();
}

Main_State_Machine::State
Main_State_Machine::get_state() const
{
    return m_state;
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
Main_State_Machine::set_stop_command()
{
    m_flags.stop = true;
}

bool
Main_State_Machine::apply_command(BLE_Protocol::State_Action action)
{
    switch(action)
    {
        case BLE_Protocol::State_Action::START:
            if(m_state == READY_TO_START)
            {
                m_flags.start = true;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
                Model_Identification::instance().activate_identification();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
                return true;
            }
            break;
        case BLE_Protocol::State_Action::STOP:
            if(m_state == OPERATION)
            {
                m_flags.stop = true;
                return true;
            }
            break;
        default:
            break;
    }
    return false;
}

}  // namespace Robot_Control