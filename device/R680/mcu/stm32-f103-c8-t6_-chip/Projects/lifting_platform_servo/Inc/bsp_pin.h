#ifndef BSP_PIN_H
#define BSP_PIN_H

#include "bsp_pin_types.h"
#include "bsp_key.h"

/* Reserve PA13/PA14 for SWD from board startup; release JTAG pins for keys. */
#define BSP_DEBUG_KEEP_SWD               1

/* LED1: 1 Hz motor communication indicator. On-board LEDs are active-low. */
#define BSP_LED_STATUS_PORT              GPIOB
#define BSP_LED_STATUS_PIN               GPIO_Pin_12
#define BSP_LED_STATUS_CLOCK             RCC_APB2Periph_GPIOB
#define BSP_LED_STATUS_MODE              GPIO_Mode_Out_PP
#define BSP_LED_STATUS_SPEED             GPIO_Speed_2MHz
#define BSP_LED_STATUS_ACTIVE            BSP_PIN_ACTIVE_LOW

/* LED2: independent 1 Hz MCU heartbeat. LED3/PB14 stays a centre sensor input. */
#define BSP_LED_AUX_AVAILABLE             1
#define BSP_LED_AUX_PORT                  GPIOB
#define BSP_LED_AUX_PIN                   GPIO_Pin_13
#define BSP_LED_AUX_CLOCK                 RCC_APB2Periph_GPIOB
#define BSP_LED_AUX_MODE                  GPIO_Mode_Out_PP
#define BSP_LED_AUX_SPEED                 GPIO_Speed_2MHz
#define BSP_LED_AUX_ACTIVE                BSP_PIN_ACTIVE_LOW

/* K1 and K2 are active-low and have board pull-ups. */
#define BSP_KEY_PRESENT_MASK             (BSP_KEY_MASK(BSP_KEY_TOP_LEFT) | \
                                          BSP_KEY_MASK(BSP_KEY_TOP_RIGHT))
#define BSP_KEY_TOP_L_PORT               GPIOB
#define BSP_KEY_TOP_L_PIN                GPIO_Pin_4
#define BSP_KEY_TOP_L_CLOCK              RCC_APB2Periph_GPIOB
#define BSP_KEY_TOP_R_PORT               GPIOB
#define BSP_KEY_TOP_R_PIN                GPIO_Pin_5
#define BSP_KEY_TOP_R_CLOCK              RCC_APB2Periph_GPIOB

/* Active-high centre reference sensor. Position control uses the absolute encoder. */
#define BSP_POSITION_SENSOR_0_PORT       GPIOB
#define BSP_POSITION_SENSOR_0_PIN        GPIO_Pin_14
#define BSP_POSITION_SENSOR_0_CLOCK      RCC_APB2Periph_GPIOB

/* A high output energizes the relay and releases the motor brake. */
#define BSP_BRAKE_RELAY_PORT             GPIOA
#define BSP_BRAKE_RELAY_PIN              GPIO_Pin_6
#define BSP_BRAKE_RELAY_CLOCK            RCC_APB2Periph_GPIOA

/* On-board CH340N USB serial interface, USART1 at 115200 bps. */
#define BSP_DEBUG_UART                   USART1
#define BSP_DEBUG_UART_BAUD_RATE         115200U
#define BSP_UART_ASYNC_TX                1
#define BSP_DEBUG_UART_CLOCK_ENABLE()    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE)
#define BSP_DEBUG_UART_IRQN              USART1_IRQn
#define BSP_DEBUG_UART_IRQ_HANDLER       USART1_IRQHandler
#define BSP_DEBUG_UART_TX_PORT           GPIOA
#define BSP_DEBUG_UART_TX_PIN            GPIO_Pin_9
#define BSP_DEBUG_UART_TX_CLOCK          RCC_APB2Periph_GPIOA
#define BSP_DEBUG_UART_RX_PORT           GPIOA
#define BSP_DEBUG_UART_RX_PIN            GPIO_Pin_10
#define BSP_DEBUG_UART_RX_CLOCK          RCC_APB2Periph_GPIOA

/* ISL60-PR RS485 interface: USART2 and the board's SP3485 transceiver. */
#define BSP_ISL60PR_UART                 USART2
#define BSP_ISL60PR_BAUD_RATE            19200U
#define BSP_ISL60PR_SLAVE_ADDRESS        1U
#define BSP_ISL60PR_UART_CLOCK_ENABLE()  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE)
#define BSP_ISL60PR_TX_PORT              GPIOA
#define BSP_ISL60PR_TX_PIN               GPIO_Pin_2
#define BSP_ISL60PR_TX_CLOCK             RCC_APB2Periph_GPIOA
#define BSP_ISL60PR_RX_PORT              GPIOA
#define BSP_ISL60PR_RX_PIN               GPIO_Pin_3
#define BSP_ISL60PR_RX_CLOCK             RCC_APB2Periph_GPIOA
#define BSP_ISL60PR_DE_PORT              GPIOA
#define BSP_ISL60PR_DE_PIN               GPIO_Pin_4
#define BSP_ISL60PR_DE_CLOCK             RCC_APB2Periph_GPIOA

#endif /* BSP_PIN_H */
