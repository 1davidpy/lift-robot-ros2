#ifndef BSP_PIN_H
#define BSP_PIN_H

#include "bsp_pin_types.h"

#define BSP_LED_STATUS_PORT    GPIOC
#define BSP_LED_STATUS_PIN     GPIO_Pin_13
#define BSP_LED_STATUS_CLOCK   RCC_APB2Periph_GPIOC
#define BSP_LED_STATUS_MODE    GPIO_Mode_Out_PP
#define BSP_LED_STATUS_SPEED   GPIO_Speed_2MHz
#define BSP_LED_STATUS_ACTIVE  BSP_PIN_ACTIVE_LOW

#endif /* BSP_PIN_H */
