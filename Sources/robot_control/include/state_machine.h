#pragma once

namespace Robot_Control
{

class State_Machine
{
public:
    enum State
    {
        READY_TO_START = 0,
        NORMAL_OPERATION,
        IDENTIFICATION,
        SOFT_STOP,
        RESET_AFTER_STOP
    };

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

    void
    update();

    State
    get_state() const;

    void
    set_button_pressed(bool button_pressed);

    void
    set_disable_motors_command(bool disable_motors_command);

    void
    set_motors_stopped(bool motors_stopped);

    // This is basically a hack so that when identification is active, the state machine won't have any effect.
    // TODO: Remove the state machine from identification driver and couple it with this one.
    void
    set_identification_state();

    void
    parse_nus_commands(char const* data);

private:
    State m_state = State::READY_TO_START;
    Transition_Flags m_flags {};
};

}  // namespace Robot_Control