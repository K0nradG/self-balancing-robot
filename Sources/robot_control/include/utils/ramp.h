// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

namespace Robot_Control
{

class Ramp
{
public:
    Ramp(float rate) : m_rate(rate) {}

    void
    update(float dt);

    float
    get_current_value() const;

    float
    get_target() const;

    void
    set_target(float target);

    void
    reset();

private:
    bool m_ramp_finished {true};
    float const m_rate;
    float m_target {};
    float m_current_value {};
};

}  // namespace Robot_Control