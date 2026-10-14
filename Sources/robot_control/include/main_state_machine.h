// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

#include "ble_payload_reader.h"
#include "ble_protocol_types.h"

namespace Robot_Control
{

class Main_State_Machine
{
    struct Transition_Flags
    {
        bool start {};
        bool stop {};
        bool disable_motors_command {};
        bool motors_stopped {};

        void
        reset()
        {
            start                  = false;
            stop                   = false;
            disable_motors_command = false;
            motors_stopped         = false;
        }
    };

public:
    static Main_State_Machine&
    instance()
    {
        static Main_State_Machine inst;
        return inst;
    }

    enum State
    {
        READY_TO_START = 0,
        OPERATION,
        IDENTIFICATION,
        SOFT_STOP,
        RESET_AFTER_STOP
    };

    void
    update();

    State
    get_state() const;

    void
    set_disable_motors_command(bool disable_motors_command);

    void
    set_motors_stopped(bool motors_stopped);

    void
    set_stop_command();

    BLE_Protocol::Command_Status
    receive_command(BLE_Protocol::Payload_Reader& reader);

private:
    Main_State_Machine()                          = default;
    Main_State_Machine(Main_State_Machine const&) = delete;
    Main_State_Machine(Main_State_Machine&&)      = delete;
    Main_State_Machine&
    operator=(Main_State_Machine const&) = delete;
    Main_State_Machine&
    operator=(Main_State_Machine&&) = delete;

    State m_state = State::READY_TO_START;
    Transition_Flags m_flags {};

    BLE_Protocol::Command_Status
    apply_command(BLE_Protocol::State_Action action);
};

}  // namespace Robot_Control