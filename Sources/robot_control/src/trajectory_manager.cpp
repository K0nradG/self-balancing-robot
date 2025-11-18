#include "trajectory_manager.h"
#include <math.h>
#include <stdlib.h>
#include "ble_commands.h"
#include "logger.h"
#include "trajectory_state_machine.h"

static Logging::Logger<IS_ENABLED(CONFIG_ROBOT_CONTROL_LOG)> trajectory_manager_logger("TRAJECTORY");

namespace Robot_Control
{

Trajectory_Manager::Trajectory_Manager(float& distance_setpoint, Ramp& rotation_setpoint_ramp)
    : m_state_machine(),
      m_distance_setpoint(distance_setpoint),
      m_rotation_setpoint_ramp(rotation_setpoint_ramp),
      m_stage_completion_cycles_counter(0u),
      m_stage_skip_cycles_counter(0u),
      m_new_rotation_angle_setpoint(0.0f),
      m_new_distance_setpoint(0.0f),
      m_stop_logs(false)
{
}

void
Trajectory_Manager::parse_trajectory_point(char const* data)
{
    if(data == nullptr)
    {
        return;
    }

    static constexpr float abs_diff   = 1e-3f;
    float new_rotation_angle_setpoint = 0.0f;
    float new_distance_setpoint       = 0.0f;

    while(*data)
    {
        if((*data == BLE_Commands::Trajectory_Manager::ROTATION) ||
           (*data == BLE_Commands::Trajectory_Manager::DISTANCE))
        {
            char key = *data;
            data++;
            char* next_data = nullptr;
            float value     = strtof(data, &next_data);

            if(data == next_data)
            {
                break;
            }
            data = next_data;

            switch(key)
            {
                case BLE_Commands::Trajectory_Manager::ROTATION:
                    new_rotation_angle_setpoint = value * deg_to_rad;
                    break;
                case BLE_Commands::Trajectory_Manager::DISTANCE:
                    new_distance_setpoint = value;
                    break;
                default:
                    break;
            }
        }
        else
        {
            data++;
        }
    }

    if((fabsf(m_new_rotation_angle_setpoint - new_rotation_angle_setpoint) > abs_diff) ||
       (fabsf(m_new_distance_setpoint - new_distance_setpoint) > abs_diff))
    {
        m_new_rotation_angle_setpoint = new_rotation_angle_setpoint;
        m_new_distance_setpoint       = new_distance_setpoint;
        m_state_machine.set_start();
    }
}

bool
Trajectory_Manager::trajectory_started() const
{
    return m_state_machine.get_state() > Trajectory_State_Machine::State::IDLE;
}

void
Trajectory_Manager::update(float current_rotation_angle, float current_distance)
{
    using State = Trajectory_State_Machine::State;

    m_state_machine.update();
    State const state = m_state_machine.get_state();
    switch(state)
    {
        case State::UPDATE_ROTATION_ANGLE_SETPOINT:
            m_rotation_setpoint_ramp.set_target(m_new_rotation_angle_setpoint);
            m_state_machine.set_rotation_angle_setpoint_updated();
            break;
        case State::CONTROL_ROTATION_ANGLE:
            control_rotation_angle(current_rotation_angle);
            break;
        case State::UPDATE_DISTANCE_SETPOINT:
            m_distance_setpoint = m_new_distance_setpoint;
            m_state_machine.set_distance_setpoint_updated();
            break;
        case State::CONTROL_DISTANCE:
            control_distance(current_distance);
            break;
        case State::TRAJECTORY_COMPLETED:
            acknowledge_trajectory_completed();
            break;
        case State::IDLE:
        default:
            break;
    }
}

void
Trajectory_Manager::acknowledge_trajectory_completed()
{
    trajectory_manager_logger.platform_log(
        Logging::LOG_LEVEL::INF, "%c%c", BLE_Commands::Prefix::TRAJECTORY_MANAGER,
        BLE_Commands::Trajectory_Manager::TRAJECTORY_COMPLETED);
    reset();
    m_state_machine.set_trajectory_acknowledged();
}

bool
Trajectory_Manager::stop_logs() const
{
    return m_stop_logs;
}

void
Trajectory_Manager::reset()
{
    m_state_machine.reset();
    m_stage_completion_cycles_counter = 0u;
    m_stage_skip_cycles_counter       = 0u;
    m_new_rotation_angle_setpoint     = m_rotation_setpoint_ramp.get_target();
    m_new_distance_setpoint           = m_distance_setpoint;
    m_stop_logs                       = false;
}

void
Trajectory_Manager::control_rotation_angle(float current_rotation_angle)
{
    // Skip the state through direct counter setting
    if(m_stage_skip_cycles_counter >= max_allowed_cycles)
    {
        m_stage_skip_cycles_counter       = 0u;
        m_stage_completion_cycles_counter = min_cycles_to_complete_stage;
    }

    if(m_stage_completion_cycles_counter >= min_cycles_to_complete_stage)
    {
        m_state_machine.set_rotation_angle_target_reached();
        m_stage_completion_cycles_counter = 0u;
        return;
    }

    if(fabsf(m_rotation_setpoint_ramp.get_target() - current_rotation_angle) <= rotation_angle_acceptable_range)
    {
        m_stage_completion_cycles_counter++;
    }
    else
    {
        m_stage_completion_cycles_counter = 0u;
    }

    m_stage_skip_cycles_counter++;
}

void
Trajectory_Manager::control_distance(float current_distance)
{
    // Skip the state through direct counter setting
    if(m_stage_skip_cycles_counter >= max_allowed_cycles)
    {
        m_stage_skip_cycles_counter       = 0u;
        m_stage_completion_cycles_counter = min_cycles_to_complete_stage;
    }

    if(m_stage_completion_cycles_counter >= min_cycles_to_complete_stage)
    {
        m_state_machine.set_distance_target_reached();
        m_stage_completion_cycles_counter = 0u;
        m_stop_logs                       = true;
        return;
    }

    if(fabsf(m_distance_setpoint - current_distance) <= distance_acceptable_range)
    {
        m_stage_completion_cycles_counter++;
    }
    else
    {
        m_stage_completion_cycles_counter = 0u;
    }

    m_stage_skip_cycles_counter++;
}

}  // namespace Robot_Control