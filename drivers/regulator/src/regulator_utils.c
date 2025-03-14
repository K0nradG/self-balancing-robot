#include "regulator_utils.h"

float
limit(float input, float lower_bound, float upper_bound)
{
    if(input < lower_bound)
    {
        input = lower_bound;
    }
    if(input > upper_bound)
    {
        input = upper_bound;
    }
    return input;
}

float
low_pass_filter(float input)
{
    static float last_output = 0.0f;
    float const output       = ALPHA * input + (1 - ALPHA) * last_output;
    last_output              = output;
    return output;
}
