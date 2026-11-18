#ifndef BSP_PIN_TYPES_H
#define BSP_PIN_TYPES_H

#include <stdint.h>

#include "stm32f10x.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t clock;
    GPIOMode_TypeDef mode;
    GPIOSpeed_TypeDef speed;
    uint8_t active_level;
} BspPinConfig;

#define BSP_PIN_ACTIVE_LOW     0U
#define BSP_PIN_ACTIVE_HIGH    1U

#endif /* BSP_PIN_TYPES_H */
