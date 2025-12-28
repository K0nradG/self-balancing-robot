#pragma once

namespace Robot_Control
{

class Main_State_Machine
{
    struct Transition_Flags
    {
        bool swing_up_start {};
        bool swing_up_finished {};
        bool enable_driving_controllers {};
        bool normal_start {};
        bool stop {};
        bool disable_motors_command {};
        bool motors_stopped {};

        void
        reset()
        {
            swing_up_start             = false;
            swing_up_finished          = false;
            enable_driving_controllers = false;
            normal_start               = false;
            stop                       = false;
            disable_motors_command     = false;
            motors_stopped             = false;
        }
    };

public:
    enum State
    {
        READY_TO_START = 0,
        SWING_UP,
        BALANCING_WITH_DRIVING_DISABLED,
        NORMAL_OPERATION,
        IDENTIFICATION,
        SOFT_STOP,
        RESET_AFTER_STOP
    };

    void
    update();

    State
    get_state() const;

    void
    set_swing_up_finished(bool swing_up_finished);

    void
    set_enable_driving_controllers(bool enable_driving_controllers);

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

    void
    main_state_machine_update();
};

}  // namespace Robot_Control