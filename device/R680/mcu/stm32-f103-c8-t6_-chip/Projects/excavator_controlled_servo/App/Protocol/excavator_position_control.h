#ifndef EXCAVATOR_POSITION_CONTROL_H
#define EXCAVATOR_POSITION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Compute a bounded incremental target, allowing only inward recovery outside limits.
 */
bool ExcavatorPosition_NextTarget(uint16_t current, uint16_t minimum, uint16_t maximum,
                                  int8_t direction, uint8_t step, uint16_t *target);

/**
 * @brief Check a bounded target; outside limits only inward movement into the safe range is allowed.
 */
bool ExcavatorPosition_TargetAllowed(uint16_t current, uint16_t target,
                                     uint16_t minimum, uint16_t maximum, uint8_t step);

#endif
