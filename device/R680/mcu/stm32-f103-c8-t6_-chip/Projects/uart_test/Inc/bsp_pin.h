#ifndef BSP_PIN_H
#define BSP_PIN_H

#include "bsp_pin_types.h"

/* System indicator LED */
#define BSP_LED_STATUS_PORT              GPIOC
#define BSP_LED_STATUS_PIN               GPIO_Pin_13
#define BSP_LED_STATUS_CLOCK             RCC_APB2Periph_GPIOC
#define BSP_LED_STATUS_MODE              GPIO_Mode_Out_PP
#define BSP_LED_STATUS_SPEED             GPIO_Speed_2MHz
#define BSP_LED_STATUS_ACTIVE            BSP_PIN_ACTIVE_LOW

/* USB-to-UART debug port, USART1 at 115200 bps */
#define BSP_DEBUG_UART                   USART1
#define BSP_DEBUG_UART_BAUD_RATE         115200U
#define BSP_DEBUG_UART_CLOCK_ENABLE()    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define BSP_DEBUG_UART_IRQN               USART1_IRQn
#define BSP_DEBUG_UART_IRQ_HANDLER        USART1_IRQHandler
#define BSP_DEBUG_UART_TX_PORT           GPIOA
#define BSP_DEBUG_UART_TX_PIN            GPIO_Pin_9
#define BSP_DEBUG_UART_TX_CLOCK          RCC_APB2Periph_GPIOA
#define BSP_DEBUG_UART_RX_PORT           GPIOA
#define BSP_DEBUG_UART_RX_PIN            GPIO_Pin_10
#define BSP_DEBUG_UART_RX_CLOCK          RCC_APB2Periph_GPIOA

#endif /* BSP_PIN_H */
