#ifndef BSP_PIN_H
#define BSP_PIN_H

#include "bsp_pin_types.h"
#include "excavator_servo_config.h"

/* System indicator LED */
#define BSP_LED_STATUS_PORT              GPIOC
#define BSP_LED_STATUS_PIN               GPIO_Pin_13
#define BSP_LED_STATUS_CLOCK             RCC_APB2Periph_GPIOC
#define BSP_LED_STATUS_MODE              GPIO_Mode_Out_PP
#define BSP_LED_STATUS_SPEED             GPIO_Speed_2MHz
#define BSP_LED_STATUS_ACTIVE            BSP_PIN_ACTIVE_LOW

/* Feetech STS3215 servo bus, USART2 at 1 Mbps */
#define BSP_STS3215_UART                 USART2
#define BSP_STS3215_UART_CLOCK           RCC_APB1Periph_USART2
#define BSP_STS3215_UART_BAUD_RATE       1000000U
#define BSP_STS3215_READ_ONLY            EXCAVATOR_READ_ONLY_DIAGNOSTICS
#define BSP_STS3215_SPEED_WRITE_ID_MASK  0x38U
#define BSP_STS3215_SPEED_WRITE_MAX      EXCAVATOR_CHASSIS_TEST_SPEED_MAX
#define BSP_STS3215_SPEED_REQUIRE_WHEEL  1
#define BSP_STS3215_MODE_WRITE_ID_MASK   0xBFU
#define BSP_STS3215_MODE_WRITE_VALUE     BSP_STS3215_MODE_WHEEL
#define BSP_STS3215_CALIBRATION_ID_MASK  0x87U
#define BSP_STS3215_CALIBRATION_SPEED_MAX EXCAVATOR_CALIBRATION_SPEED_MAX
#define BSP_STS3215_CALIBRATION_STEP_MAX EXCAVATOR_CALIBRATION_STEP_MAX
#define BSP_STS3215_REMOTE_STEP_MAX      EXCAVATOR_REMOTE_POSITION_WINDOW_MAX
#define BSP_STS3215_REMOTE_SPEED_MAX     EXCAVATOR_REMOTE_SPEED_MAX
#define BSP_STS3215_TORQUE_WRITE_ID_MASK 0xBFU
#define BSP_STS3215_POSITION_WRITES      0
#define BSP_STS3215_UART_IRQN            USART2_IRQn
#define BSP_STS3215_UART_IRQ_HANDLER     USART2_IRQHandler
#define BSP_STS3215_UART_IRQ_PRIORITY    4U
#define BSP_STS3215_UART_TX_PORT         GPIOA
#define BSP_STS3215_UART_TX_PIN          GPIO_Pin_2
#define BSP_STS3215_UART_TX_CLOCK        RCC_APB2Periph_GPIOA
#define BSP_STS3215_UART_RX_PORT         GPIOA
#define BSP_STS3215_UART_RX_PIN          GPIO_Pin_3
#define BSP_STS3215_UART_RX_CLOCK        RCC_APB2Periph_GPIOA

/* USB-to-UART control and debug port, USART1 at 115200 bps */
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

/* NRF24L01+ radio, SPI1. */
#define BSP_NRF24_SPI                    SPI1
#define BSP_NRF24_SPI_CLOCK              RCC_APB2Periph_SPI1
#define BSP_NRF24_SPI_BAUD_PRESCALER     SPI_BaudRatePrescaler_16
#define BSP_NRF24_CSN_PORT               GPIOB
#define BSP_NRF24_CSN_PIN                GPIO_Pin_12
#define BSP_NRF24_CSN_CLOCK              RCC_APB2Periph_GPIOB
#define BSP_NRF24_SCK_PORT               GPIOA
#define BSP_NRF24_SCK_PIN                GPIO_Pin_5
#define BSP_NRF24_SCK_CLOCK              RCC_APB2Periph_GPIOA
#define BSP_NRF24_MISO_PORT              GPIOA
#define BSP_NRF24_MISO_PIN               GPIO_Pin_6
#define BSP_NRF24_MISO_CLOCK             RCC_APB2Periph_GPIOA
#define BSP_NRF24_MOSI_PORT              GPIOA
#define BSP_NRF24_MOSI_PIN               GPIO_Pin_7
#define BSP_NRF24_MOSI_CLOCK             RCC_APB2Periph_GPIOA
#define BSP_NRF24_CE_PORT                GPIOB
#define BSP_NRF24_CE_PIN                 GPIO_Pin_13
#define BSP_NRF24_CE_CLOCK               RCC_APB2Periph_GPIOB
#define BSP_NRF24_IRQ_PORT               GPIOA
#define BSP_NRF24_IRQ_PIN                GPIO_Pin_4
#define BSP_NRF24_IRQ_CLOCK              RCC_APB2Periph_GPIOA

#endif /* BSP_PIN_H */
