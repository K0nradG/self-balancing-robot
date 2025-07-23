#pragma once

class Low_Pass_Filter
{
public:
    Low_Pass_Filter(float alpha) : m_alpha(alpha) {}

    float
    filter(float input);

private:
    float m_alpha;
    float m_last_output {};
};