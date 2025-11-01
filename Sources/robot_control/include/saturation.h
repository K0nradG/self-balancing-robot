#pragma once

class Saturation
{
public:
    Saturation(float lower_limit, float upper_limit);

    float
    saturate(float input) const;

private:
    float m_lower_limit;
    float m_upper_limit;
};