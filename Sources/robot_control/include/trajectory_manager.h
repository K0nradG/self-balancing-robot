// Performs trajectory tracking by controlling distance and rotation angle setpoints directly - passed by reference, to
// allow for free modification of their value

#pragma once
#include <stdint.h>
#include "ramp.h"
#include "trajectory_state_machine.h"

namespace Robot_Control
{

class Trajectory_Manager
{
    static constexpr uint8_t min_cycles_to_complete_stage  = 20u;
    static constexpr uint16_t max_allowed_cycles           = 1000u;
    static constexpr float pi                              = 3.14159265358979323846f;
    static constexpr float deg_to_rad                      = static_cast<float>(pi) / 180.f;
    static constexpr float rotation_angle_acceptable_range = 0.6f * deg_to_rad;
    static constexpr float distance_acceptable_range       = 0.02f;

public:
    Trajectory_Manager(float& distance_setpoint, Ramp& rotation_setpoint_ramp);

    void
    parse_trajectory_point(char const* data);

    bool
    trajectory_started() const;

    void
    update(float current_rotation_angle, float current_distance);

    bool
    stop_logs() const;

    void
    reset();

private:
    Trajectory_State_Machine m_state_machine;
    float& m_distance_setpoint;
    Ramp& m_rotation_setpoint_ramp;

    uint8_t m_stage_completion_cycles_counter;
    uint16_t m_stage_skip_cycles_counter;
    float m_rotation_angle_setpoint_increment;
    float m_distance_setpoint_increment;
    bool m_stop_logs;

    void
    control_rotation_angle(float current_rotation_angle);

    void
    control_distance(float current_distance);

    void
    acknowledge_trajectory_completed();
};

}  // namespace Robot_Control