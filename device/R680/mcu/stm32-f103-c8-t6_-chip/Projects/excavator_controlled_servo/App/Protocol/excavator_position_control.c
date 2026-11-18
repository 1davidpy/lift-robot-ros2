#include "excavator_position_control.h"

#include <stddef.h>

bool ExcavatorPosition_TargetAllowed(uint16_t current, uint16_t target,
                                     uint16_t minimum, uint16_t maximum, uint8_t step)
{
    if (minimum >= maximum || maximum > 4095U || current > 4095U || target > 4095U || step == 0U) return false;
    uint16_t difference = current > target ? current - target : target - current;
    if (difference > step) return false;
    if (current < minimum) return target >= current && target <= maximum;
    if (current > maximum) return target <= current && target >= minimum;
    return target >= minimum && target <= maximum;
}

bool ExcavatorPosition_NextTarget(uint16_t current, uint16_t minimum, uint16_t maximum,
                                  int8_t direction, uint8_t step, uint16_t *target)
{
    if (target == NULL || step == 0U || step > 10U || direction < -1 || direction > 1 || direction == 0) return false;
    if (minimum >= maximum || maximum > 4095U || current > 4095U) return false;
    int32_t next = (int32_t)current + (int32_t)direction * step;
    if (current < minimum)
    {
        if (direction < 0) return false;
        if (next > minimum) next = minimum;
    }
    else if (current > maximum)
    {
        if (direction > 0) return false;
        if (next < maximum) next = maximum;
    }
    else
    {
        if (next < minimum) next = minimum;
        if (next > maximum) next = maximum;
    }
    if (!ExcavatorPosition_TargetAllowed(current, (uint16_t)next, minimum, maximum, step)) return false;
    *target = (uint16_t)next;
    return *target != current;
}
