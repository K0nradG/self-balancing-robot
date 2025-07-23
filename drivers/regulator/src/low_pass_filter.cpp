#include "low_pass_filter.h"

float
Low_Pass_Filter::filter(float input)
{
    float const output = m_alpha * input + (1.0f - m_alpha) * m_last_output;
    m_last_output      = output;

    return output;
}