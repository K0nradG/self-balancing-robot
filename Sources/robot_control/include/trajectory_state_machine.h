#pragma once

class Trajectory_State_Machine
{
    struct Transition_Flags
    {
        bool start {};
        bool rotation_angle_setpoint_updated {};
        bool rotation_angle_target_reached {};
        bool distance_setpoint_updated {};
        bool distance_target_reached {};
        bool trajectory_acknowledged {};

        void
        reset()
        {
            start                           = false;
            rotation_angle_setpoint_updated = false;
            rotation_angle_target_reached   = false;
            distance_setpoint_updated       = false;
            distance_target_reached         = false;
            trajectory_acknowledged         = false;
        }
    };

public:
    enum State
    {
        IDLE = 0,
        UPDATE_ROTATION_ANGLE_SETPOINT,
        CONTROL_ROTATION_ANGLE,
        UPDATE_DISTANCE_SETPOINT,
        CONTROL_DISTANCE,
        TRAJECTORY_COMPLETED
    };

    void
    update();

    State
    get_state() const;

    void
    set_start();

    void
    set_rotation_angle_setpoint_updated();

    void
    set_rotation_angle_target_reached();

    void
    set_distance_setpoint_updated();

    void
    set_distance_target_reached();

    void
    set_trajectory_acknowledged();

    void
    reset();

private:
    State m_state {IDLE};
    Transition_Flags m_flags {};
};