#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

typedef enum
{
    BSP_LED_STATUS = 0,
    BSP_LED_AUX,
    BSP_LED_COUNT
} BspLed;

/** @brief Initialize the LEDs enabled by the project pin map, initially off. */
void BSP_LED_Init(void);
/** @brief Turn on an available LED; unavailable LEDs are ignored. */
void BSP_LED_On(BspLed led);
/** @brief Turn off an available LED. */
void BSP_LED_Off(BspLed led);
/** @brief Toggle an available LED. */
void BSP_LED_Toggle(BspLed led);
/** @brief Write a logical LED state (nonzero means on). */
void BSP_LED_Write(BspLed led, uint8_t state);

#endif /* BSP_LED_H */
