// Copyright 2026 Filip Dymczyk and Konrad Grucel

#include "saturation.h"
#include "zephyr/sys/util.h"

namespace Robot_Control
{

Saturation::Saturation(float lower_limit, float upper_limit) : m_lower_limit(lower_limit), m_upper_limit(upper_limit) {}

float
Saturation::saturate(float input) const
{
    input = MAX(input, m_lower_limit);
    input = MIN(input, m_upper_limit);
    return input;
}

}  // namespace Robot_Control