#ifndef BSP_BRAKE_RELAY_H
#define BSP_BRAKE_RELAY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the brake relay output in the brake-engaged state.
 */
void BSP_BrakeRelay_Init(void);

/**
 * @brief Control the motor brake relay.
 * @param released True to energize the relay and release the brake.
 */
void BSP_BrakeRelay_SetReleased(bool released);

/**
 * @brief Return the last requested brake state.
 * @return True when the relay is energized to release the brake.
 */
bool BSP_BrakeRelay_IsReleased(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BRAKE_RELAY_H */
