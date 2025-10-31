#pragma once
#include <stdint.h>

class Trajectory_Manager
{
    static constexpr uint8_t min_cycles_to_complete_stage  = 20u;
    static constexpr float pi                              = 3.14159265358979323846f;
    static constexpr float deg_to_rad                      = static_cast<float>(pi) / 180.f;
    static constexpr float rotation_angle_acceptable_range = 0.6f * deg_to_rad;
    static constexpr float distance_acceptable_range       = 0.02f;

public:
    void
    parse_trajectory_point(char const* data);

    void
    control_rotation_angle_target(float current_rotation_angle);

    void
    control_distance_target(float current_distance);

    void
    acknowledge_trajectory_completed();

    bool
    trajectory_started() const;

    bool
    rotation_target_given() const;

    bool
    rotation_angle_target_reached() const;

    bool
    distance_target_given() const;

    bool
    trajectory_completed() const;

    void
    set_initial_rotation_angle(float initial_rotation_angle);

    float
    get_rotation_angle_target();

    void
    set_rotation_angle_target_given();

    void
    set_initial_distance(float initial_distance);

    float
    get_distance_target();

    void
    set_distance_target_given();

    void
    reset();

private:
    bool m_trajectory_started {false};
    bool m_rotation_angle_target_given {false};
    bool m_distance_target_given {false};
    bool m_rotation_angle_target_reached {false};
    bool m_distance_target_reached {false};
    uint8_t m_stage_completion_cycles_counter {0u};
    float m_initial_rotation_angle {};
    float m_rotation_angle_target {};
    float m_initial_distance {};
    float m_distance_target {};
};