#ifndef BSP_POSITION_SENSOR_H
#define BSP_POSITION_SENSOR_H

#include <stdint.h>
#include "bsp_position_sensor_config.h"

#if !defined(BSP_POSITION_SENSOR_COUNT) || BSP_POSITION_SENSOR_COUNT < 1U || BSP_POSITION_SENSOR_COUNT > 8U
#error "BSP_POSITION_SENSOR_COUNT must be configured between 1 and 8."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the configured active-high position inputs.
 */
void BSP_PositionSensor_Init(void);

/**
 * @brief Read all position inputs as a bit mask.
 * @return Bit i is set when configured active-high channel i is asserted.
 */
uint8_t BSP_PositionSensor_ReadMask(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POSITION_SENSOR_H */
