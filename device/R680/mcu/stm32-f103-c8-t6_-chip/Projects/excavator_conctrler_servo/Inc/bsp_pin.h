#ifndef BSP_PIN_H
#define BSP_PIN_H

#include "bsp_pin_types.h"

/* System indicator LED */
#define BSP_LED_STATUS_PORT        GPIOC
#define BSP_LED_STATUS_PIN         GPIO_Pin_13
#define BSP_LED_STATUS_CLOCK       RCC_APB2Periph_GPIOC
#define BSP_LED_STATUS_MODE        GPIO_Mode_Out_OD
#define BSP_LED_STATUS_SPEED       GPIO_Speed_2MHz
#define BSP_LED_STATUS_ACTIVE      BSP_PIN_ACTIVE_LOW

/* NRF24L01+ enhanced antenna module, SPI1 */
#define BSP_NRF24_SPI              SPI1
#define BSP_NRF24_SPI_CLOCK        RCC_APB2Periph_SPI1
#define BSP_NRF24_SPI_BAUD_PRESCALER SPI_BaudRatePrescaler_16
#define BSP_NRF24_CSN_PORT         GPIOA
#define BSP_NRF24_CSN_PIN          GPIO_Pin_4
#define BSP_NRF24_CSN_CLOCK        RCC_APB2Periph_GPIOA
#define BSP_NRF24_SCK_PORT         GPIOA
#define BSP_NRF24_SCK_PIN          GPIO_Pin_5
#define BSP_NRF24_SCK_CLOCK        RCC_APB2Periph_GPIOA
#define BSP_NRF24_MISO_PORT        GPIOA
#define BSP_NRF24_MISO_PIN         GPIO_Pin_6
#define BSP_NRF24_MISO_CLOCK       RCC_APB2Periph_GPIOA
#define BSP_NRF24_MOSI_PORT        GPIOA
#define BSP_NRF24_MOSI_PIN         GPIO_Pin_7
#define BSP_NRF24_MOSI_CLOCK       RCC_APB2Periph_GPIOA
#define BSP_NRF24_CE_PORT          GPIOA
#define BSP_NRF24_CE_PIN           GPIO_Pin_11
#define BSP_NRF24_CE_CLOCK         RCC_APB2Periph_GPIOA
#define BSP_NRF24_IRQ_PORT         GPIOA
#define BSP_NRF24_IRQ_PIN          GPIO_Pin_12
#define BSP_NRF24_IRQ_CLOCK        RCC_APB2Periph_GPIOA

/* Joysticks, ADC1 */
#define BSP_JOYSTICK_LEFT_Y_PORT   GPIOA
#define BSP_JOYSTICK_LEFT_Y_PIN    GPIO_Pin_0
#define BSP_JOYSTICK_LEFT_Y_CLOCK  RCC_APB2Periph_GPIOA
#define BSP_JOYSTICK_LEFT_Y_ADC_CHANNEL ADC_Channel_0
#define BSP_JOYSTICK_LEFT_X_PORT   GPIOA
#define BSP_JOYSTICK_LEFT_X_PIN    GPIO_Pin_1
#define BSP_JOYSTICK_LEFT_X_CLOCK  RCC_APB2Periph_GPIOA
#define BSP_JOYSTICK_LEFT_X_ADC_CHANNEL ADC_Channel_1
#define BSP_JOYSTICK_RIGHT_Y_PORT  GPIOA
#define BSP_JOYSTICK_RIGHT_Y_PIN   GPIO_Pin_2
#define BSP_JOYSTICK_RIGHT_Y_CLOCK RCC_APB2Periph_GPIOA
#define BSP_JOYSTICK_RIGHT_Y_ADC_CHANNEL ADC_Channel_2
#define BSP_JOYSTICK_RIGHT_X_PORT  GPIOA
#define BSP_JOYSTICK_RIGHT_X_PIN   GPIO_Pin_3
#define BSP_JOYSTICK_RIGHT_X_CLOCK RCC_APB2Periph_GPIOA
#define BSP_JOYSTICK_RIGHT_X_ADC_CHANNEL ADC_Channel_3

/* Left and right wheels, ADC1 */
#define BSP_WHEEL_LEFT_PORT        GPIOB
#define BSP_WHEEL_LEFT_PIN         GPIO_Pin_0
#define BSP_WHEEL_LEFT_CLOCK       RCC_APB2Periph_GPIOB
#define BSP_WHEEL_LEFT_ADC_CHANNEL ADC_Channel_8
#define BSP_WHEEL_RIGHT_PORT       GPIOB
#define BSP_WHEEL_RIGHT_PIN        GPIO_Pin_1
#define BSP_WHEEL_RIGHT_CLOCK      RCC_APB2Periph_GPIOB
#define BSP_WHEEL_RIGHT_ADC_CHANNEL ADC_Channel_9

/* SPI display */
#define BSP_DISPLAY_SCL_PORT       GPIOB
#define BSP_DISPLAY_SCL_PIN        GPIO_Pin_13
#define BSP_DISPLAY_SCL_CLOCK      RCC_APB2Periph_GPIOB
#define BSP_DISPLAY_SDA_PORT       GPIOB
#define BSP_DISPLAY_SDA_PIN        GPIO_Pin_15
#define BSP_DISPLAY_SDA_CLOCK      RCC_APB2Periph_GPIOB
#define BSP_DISPLAY_RES_PORT       GPIOB
#define BSP_DISPLAY_RES_PIN        GPIO_Pin_7
#define BSP_DISPLAY_RES_CLOCK      RCC_APB2Periph_GPIOB
#define BSP_DISPLAY_DC_PORT        GPIOB
#define BSP_DISPLAY_DC_PIN         GPIO_Pin_8
#define BSP_DISPLAY_DC_CLOCK       RCC_APB2Periph_GPIOB
#define BSP_DISPLAY_CS_PORT        GPIOB
#define BSP_DISPLAY_CS_PIN         GPIO_Pin_9
#define BSP_DISPLAY_CS_CLOCK       RCC_APB2Periph_GPIOB
#define BSP_DISPLAY_BLK_PORT       GPIOB
#define BSP_DISPLAY_BLK_PIN        GPIO_Pin_10
#define BSP_DISPLAY_BLK_CLOCK      RCC_APB2Periph_GPIOB

/* Keys */
#define BSP_KEY_TOP_L_PORT         GPIOA
#define BSP_KEY_TOP_L_PIN          GPIO_Pin_15
#define BSP_KEY_TOP_L_CLOCK        RCC_APB2Periph_GPIOA
#define BSP_KEY_TOP_R_PORT         GPIOB
#define BSP_KEY_TOP_R_PIN          GPIO_Pin_11
#define BSP_KEY_TOP_R_CLOCK        RCC_APB2Periph_GPIOB
#define BSP_KEY_MODE_PORT          GPIOB
#define BSP_KEY_MODE_PIN           GPIO_Pin_5
#define BSP_KEY_MODE_CLOCK         RCC_APB2Periph_GPIOB
#define BSP_KEY_SPEED_PORT         GPIOB
#define BSP_KEY_SPEED_PIN          GPIO_Pin_6
#define BSP_KEY_SPEED_CLOCK        RCC_APB2Periph_GPIOB

/* Debug UART, CH340N */
#define BSP_DEBUG_UART_TX_PORT     GPIOA
#define BSP_DEBUG_UART_TX_PIN      GPIO_Pin_9
#define BSP_DEBUG_UART_TX_CLOCK    RCC_APB2Periph_GPIOA
#define BSP_DEBUG_UART_RX_PORT     GPIOA
#define BSP_DEBUG_UART_RX_PIN      GPIO_Pin_10
#define BSP_DEBUG_UART_RX_CLOCK    RCC_APB2Periph_GPIOA

/* SWD */
#define BSP_SWDIO_PORT             GPIOA
#define BSP_SWDIO_PIN              GPIO_Pin_13
#define BSP_SWDIO_CLOCK            RCC_APB2Periph_GPIOA
#define BSP_SWCLK_PORT             GPIOA
#define BSP_SWCLK_PIN              GPIO_Pin_14
#define BSP_SWCLK_CLOCK            RCC_APB2Periph_GPIOA

#endif /* BSP_PIN_H */
