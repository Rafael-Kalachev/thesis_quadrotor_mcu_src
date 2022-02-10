
#include "m_control/inc/limit.h"

float32_t limit_max_min(float32_t val, float32_t max, float32_t min)
{
    if (val > max)
    {
        return max;
    }
    
    if (val < min)
    {
        return min;
    }

    return val;

}
