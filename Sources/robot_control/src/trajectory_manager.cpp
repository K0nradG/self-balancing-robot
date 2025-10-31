#include "trajectory_manager.h"
#include <math.h>
#include <stdlib.h>
#include "ble_commands.h"

#ifdef CONFIG_ROBOT_CONTROL_LOG
#include "logger.h"
#endif  // CONFIG_ROBOT_CONTROL_LOG

void
Trajectory_Manager::parse_trajectory_point(char const* data)
{
    if(data == nullptr)
    {
        return;
    }

    static constexpr float abs_diff = 1e-3f;
    float new_rotation_target       = 0.0f;
    float new_distance_target       = 0.0f;

    while(*data)
    {
        if((*data == TRAJECTORY_MANAGER_ROTATION) || (*data == TRAJECTORY_MANAGER_DISTANCE))
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
                case TRAJECTORY_MANAGER_ROTATION:
                    new_rotation_target = value * deg_to_rad;
                    break;
                case TRAJECTORY_MANAGER_DISTANCE:
                    new_distance_target = value;
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

    if((fabsf(m_rotation_angle_target - new_rotation_target) > abs_diff) ||
       (fabsf(m_distance_target - new_distance_target) > abs_diff))
    {
        m_rotation_angle_target = new_rotation_target;
        m_distance_target       = new_distance_target;
        m_trajectory_started    = true;
    }
}

void
Trajectory_Manager::control_rotation_angle_target(float current_rotation_angle)
{
    if(m_rotation_angle_target_reached)
    {
        return;
    }

    if(m_stage_completion_cycles_counter >= min_cycles_to_complete_stage)
    {
        m_rotation_angle_target_reached   = true;
        m_stage_completion_cycles_counter = 0u;
        return;
    }

    if(fabsf((m_rotation_angle_target + m_initial_rotation_angle) - current_rotation_angle) <=
       rotation_angle_acceptable_range)
    {
        m_stage_completion_cycles_counter++;
    }
    else
    {
        m_stage_completion_cycles_counter = 0u;
    }
}

void
Trajectory_Manager::control_distance_target(float current_distance)
{
    if(m_distance_target_reached)
    {
        return;
    }

    if(m_stage_completion_cycles_counter >= min_cycles_to_complete_stage)
    {
        m_distance_target_reached         = true;
        m_stage_completion_cycles_counter = 0u;
        return;
    }

    if(fabsf((m_distance_target + m_initial_distance) - current_distance) <= distance_acceptable_range)
    {
        m_stage_completion_cycles_counter++;
    }
    else
    {
        m_stage_completion_cycles_counter = 0u;
    }
}

void
Trajectory_Manager::acknowledge_trajectory_completed()
{
#ifdef CONFIG_ROBOT_CONTROL_LOG
    platform_log("TRAJECTORY", LOG_LEVEL_INF, "finished");
#endif  // CONFIG_ROBOT_CONTROL_LOG
    reset();
}

bool
Trajectory_Manager::trajectory_started() const
{
    return m_trajectory_started;
}

bool
Trajectory_Manager::rotation_target_given() const
{
    return m_rotation_angle_target_given;
}

bool
Trajectory_Manager::rotation_angle_target_reached() const
{
    return m_rotation_angle_target_reached;
}

bool
Trajectory_Manager::distance_target_given() const
{
    return m_distance_target_given;
}

bool
Trajectory_Manager::trajectory_completed() const
{
    return (m_rotation_angle_target_reached && m_distance_target_reached);
}

void
Trajectory_Manager::set_initial_rotation_angle(float initial_rotation_angle)
{
    m_initial_rotation_angle = initial_rotation_angle;
}

float
Trajectory_Manager::get_rotation_angle_target()
{
    return m_rotation_angle_target;
}

void
Trajectory_Manager::set_rotation_angle_target_given()
{
    m_rotation_angle_target_given = true;
}

void
Trajectory_Manager::set_initial_distance(float initial_distance)
{
    m_initial_distance = initial_distance;
}

float
Trajectory_Manager::get_distance_target()
{
    return m_distance_target;
}

void
Trajectory_Manager::set_distance_target_given()
{
    m_distance_target_given = true;
}

void
Trajectory_Manager::reset()
{
    m_trajectory_started              = false;
    m_rotation_angle_target_given     = false;
    m_distance_target_given           = false;
    m_rotation_angle_target_reached   = false;
    m_distance_target_reached         = false;
    m_stage_completion_cycles_counter = 0u;
    m_initial_rotation_angle          = 0.0f;
    m_rotation_angle_target           = 0.0f;
    m_distance_target                 = 0.0f;
    m_initial_distance                = 0.0f;
}