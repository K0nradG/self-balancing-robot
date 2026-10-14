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

BLE_Protocol::Command_Status
Main_State_Machine::receive_command(BLE_Protocol::Payload_Reader& reader)
{
    uint8_t action {};
    if(!reader.get_u8(action) || !reader.done())
    {
        return BLE_Protocol::Command_Status::INVALID_LENGTH;
    }

    return Main_State_Machine::instance().apply_command(static_cast<BLE_Protocol::State_Action>(action));
}

BLE_Protocol::Command_Status
Main_State_Machine::apply_command(BLE_Protocol::State_Action action)
{
    auto status = BLE_Protocol::Command_Status::OK;
    switch(action)
    {
        case BLE_Protocol::State_Action::START:
            if(m_state == READY_TO_START)
            {
                m_flags.start = true;

#ifdef CONFIG_MODEL_IDENTIFICATION_DRV
                Model_Identification::instance().activate_identification();
#endif  // CONFIG_MODEL_IDENTIFICATION_DRV
            }
            break;
        case BLE_Protocol::State_Action::STOP:
            if(m_state == OPERATION)
            {
                m_flags.stop = true;
            }
            break;
        default:
        {
            status = BLE_Protocol::Command_Status::INVALID_STATE;
            break;
        }
    }
    return status;
}

}  // namespace Robot_Control