#ifndef SWING_CENTER_STORAGE_H
#define SWING_CENTER_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Load the last operator-selected swing center from internal flash.
 * @param position_pulses Destination for the signed encoder position.
 * @return True when a valid stored record exists.
 */
bool SwingCenterStorage_Load(int32_t *position_pulses);

/**
 * @brief Replace the stored swing center in the reserved flash page.
 * @param position_pulses Signed encoder position to persist.
 * @return True when erase, programming, and readback verification succeed.
 */
bool SwingCenterStorage_Save(int32_t position_pulses);

#endif /* SWING_CENTER_STORAGE_H */
