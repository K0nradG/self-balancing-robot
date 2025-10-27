#include "ramp.h"
#include <math.h>

void
Ramp::update(float dt)
{
    if(m_ramp_finished)
    {
        return;
    }

    float const max_step = m_rate * dt;
    float const current_diff = m_target - m_current_value;
    
    if (fabsf(current_diff) <= max_step)
    {
        m_current_value = m_target;
        m_ramp_finished = true;
    }
    else
    {
        float const direction = (current_diff > 0.0f) ? 1.0f : -1.0f;
        m_current_value += (max_step * direction);
    } 
}

float
Ramp::get_current_value() const
{
    return m_current_value;
}

void
Ramp::set_target(float target)
{
    m_target = target;
    m_ramp_finished = false;
}

void
Ramp::reset()
{
    m_ramp_finished = true;
    m_target = 0.0f;
    m_current_value = 0.0f;
}