#include "low_pass_filter.h"
#include "zephyr/sys/util.h"

namespace Robot_Control
{

float
Low_Pass_Filter::filter(float input)
{
    float const output = m_alpha * input + (1.0f - m_alpha) * m_last_output;
    m_last_output      = output;

    return output;
}

void
Low_Pass_Filter::set_alpha(float alpha)
{
    alpha   = MAX(0.0f, alpha);
    alpha   = MIN(1.0f, alpha);
    m_alpha = alpha;
}

}  // namespace Robot_Control