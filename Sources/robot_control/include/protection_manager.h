// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

namespace Robot_Control
{

class Protection_Manager
{
public:
    static Protection_Manager&
    instance()
    {
        static Protection_Manager s_protection_manager {};
        return s_protection_manager;
    }

    bool
    validate_robot_angle(float balance_setpoint, float balance_angle) const;

private:
    Protection_Manager() = default;

    Protection_Manager(Protection_Manager const&) = delete;

    Protection_Manager const&
    operator=(Protection_Manager const&) = delete;
};
}  // namespace Robot_Control