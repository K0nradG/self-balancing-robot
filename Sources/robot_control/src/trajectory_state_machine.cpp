#include "trajectory_state_machine.h"

namespace Robot_Control
{

void
Trajectory_State_Machine::update()
{
    switch(m_state)
    {
        case IDLE:
            if(m_flags.start)
            {
                m_state = UPDATE_ROTATION_ANGLE_SETPOINT;
            }
            break;
        case UPDATE_ROTATION_ANGLE_SETPOINT:
            if(m_flags.rotation_angle_setpoint_updated)
            {
                m_state = CONTROL_ROTATION_ANGLE;
            }
            break;
        case CONTROL_ROTATION_ANGLE:
            if(m_flags.rotation_angle_target_reached)
            {
                m_state = UPDATE_DISTANCE_SETPOINT;
            }
            break;
        case UPDATE_DISTANCE_SETPOINT:
            if(m_flags.distance_setpoint_updated)
            {
                m_state = CONTROL_DISTANCE;
            }
            break;
        case CONTROL_DISTANCE:
            if(m_flags.distance_target_reached)
            {
                m_state = TRAJECTORY_COMPLETED;
            }
            break;
        case TRAJECTORY_COMPLETED:
            if(m_flags.trajectory_acknowledged)
            {
                m_state = IDLE;
            }
            break;
        default:
            break;
    }

    m_flags.reset();
}

Trajectory_State_Machine::State
Trajectory_State_Machine::get_state() const
{
    return m_state;
}

void
Trajectory_State_Machine::set_start()
{
    m_flags.start = true;
}

void
Trajectory_State_Machine::set_rotation_angle_setpoint_updated()
{
    m_flags.rotation_angle_setpoint_updated = true;
}

void
Trajectory_State_Machine::set_rotation_angle_target_reached()
{
    m_flags.rotation_angle_target_reached = true;
}

void
Trajectory_State_Machine::set_distance_setpoint_updated()
{
    m_flags.distance_setpoint_updated = true;
}

void
Trajectory_State_Machine::set_distance_target_reached()
{
    m_flags.distance_target_reached = true;
}

void
Trajectory_State_Machine::set_trajectory_acknowledged()
{
    m_flags.trajectory_acknowledged = true;
}

void
Trajectory_State_Machine::reset()
{
    m_state = IDLE;
    m_flags.reset();
}

}  // namespace Robot_Control