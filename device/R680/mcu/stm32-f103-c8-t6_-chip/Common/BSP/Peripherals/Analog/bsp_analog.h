#ifndef BSP_ANALOG_H
#define BSP_ANALOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_ANALOG_LEFT_Y = 0,
    BSP_ANALOG_LEFT_X,
    BSP_ANALOG_RIGHT_Y,
    BSP_ANALOG_RIGHT_X,
    BSP_ANALOG_WHEEL_LEFT,
    BSP_ANALOG_WHEEL_RIGHT,
    BSP_ANALOG_COUNT,
} BspAnalogChannel;

/**
 * @brief Initialize project analog input pins and ADC1.
 */
void BSP_Analog_Init(void);

/**
 * @brief Read one analog input channel.
 * @param channel Logical analog input channel.
 * @return Raw 12-bit ADC value.
 */
uint16_t BSP_Analog_Read(BspAnalogChannel channel);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ANALOG_H */
