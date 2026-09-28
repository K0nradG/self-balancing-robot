// Copyright 2026 Filip Dymczyk and Konrad Grucel

#pragma once

namespace Robot_Control
{

class Low_Pass_Filter
{
public:
    Low_Pass_Filter(float alpha) : m_alpha(alpha) {}

    float
    filter(float input);

    void
    set_alpha(float alpha);

private:
    float m_alpha;
    float m_last_output {};
};

}  // namespace Robot_Control