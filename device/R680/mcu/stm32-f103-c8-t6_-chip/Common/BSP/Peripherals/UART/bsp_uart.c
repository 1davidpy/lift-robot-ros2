#include "bsp_uart.h"

#include <stddef.h>

#include "bsp_pin.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#define BSP_UART_TX_TIMEOUT_LOOPS  100000U
#define BSP_UART_RX_BUFFER_SIZE    128U

#ifndef BSP_UART_ASYNC_TX
#define BSP_UART_ASYNC_TX 0
#endif
#if BSP_UART_ASYNC_TX
#define BSP_UART_TX_BUFFER_SIZE 256U
static uint8_t uart_tx_buffer[BSP_UART_TX_BUFFER_SIZE];
static volatile uint16_t uart_tx_head;
static volatile uint16_t uart_tx_tail;
#endif

static volatile uint8_t uart_rx_buffer[BSP_UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_rx_head;
static volatile uint8_t uart_rx_tail;
static volatile uint32_t uart_rx_overflow_count;

void BSP_UART_DebugInit(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;

    RCC_APB2PeriphClockCmd(BSP_DEBUG_UART_TX_CLOCK | BSP_DEBUG_UART_RX_CLOCK, ENABLE);
    BSP_DEBUG_UART_CLOCK_ENABLE();

    gpio_init.GPIO_Pin = BSP_DEBUG_UART_TX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_DEBUG_UART_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_DEBUG_UART_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BSP_DEBUG_UART_RX_PORT, &gpio_init);

    USART_StructInit(&usart_init);
    usart_init.USART_BaudRate = BSP_DEBUG_UART_BAUD_RATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(BSP_DEBUG_UART, &usart_init);
    USART_ITConfig(BSP_DEBUG_UART, USART_IT_RXNE, ENABLE);
    USART_Cmd(BSP_DEBUG_UART, ENABLE);

    NVIC_InitTypeDef nvic_init = {
        .NVIC_IRQChannel = BSP_DEBUG_UART_IRQN,
        .NVIC_IRQChannelPreemptionPriority = 6U,
        .NVIC_IRQChannelSubPriority = 0U,
        .NVIC_IRQChannelCmd = ENABLE,
    };
    NVIC_Init(&nvic_init);
}

BspUartResult BSP_UART_DebugWrite(const uint8_t *data, uint16_t length)
{
#if BSP_UART_ASYNC_TX
    if (data == NULL || length == 0U) return BSP_UART_INVALID_ARGUMENT;
    /* One task owns TX. Publish only after copying the complete frame. */
    uint16_t head = uart_tx_head;
    uint16_t free_bytes = (uint16_t)((uart_tx_tail - head - 1U) &
                                    (BSP_UART_TX_BUFFER_SIZE - 1U));
    if (length > free_bytes)
    {
        return BSP_UART_TIMEOUT;
    }
    for (uint16_t i = 0U; i < length; ++i)
    {
        uart_tx_buffer[head] = data[i];
        head = (uint16_t)((head + 1U) & (BSP_UART_TX_BUFFER_SIZE - 1U));
    }
    __asm volatile ("" ::: "memory");
    uart_tx_head = head;
    USART_ITConfig(BSP_DEBUG_UART, USART_IT_TXE, ENABLE);
    return BSP_UART_OK;
#else
    uint32_t timeout;

    if (data == NULL || length == 0U)
    {
        return BSP_UART_INVALID_ARGUMENT;
    }

    for (uint16_t i = 0U; i < length; i++)
    {
        timeout = BSP_UART_TX_TIMEOUT_LOOPS;
        while (USART_GetFlagStatus(BSP_DEBUG_UART, USART_FLAG_TXE) == RESET)
        {
            if (timeout-- == 0U)
            {
                return BSP_UART_TIMEOUT;
            }
        }

        USART_SendData(BSP_DEBUG_UART, data[i]);
    }

    timeout = BSP_UART_TX_TIMEOUT_LOOPS;
    while (USART_GetFlagStatus(BSP_DEBUG_UART, USART_FLAG_TC) == RESET)
    {
        if (timeout-- == 0U)
        {
            return BSP_UART_TIMEOUT;
        }
    }

    return BSP_UART_OK;
#endif
}

bool BSP_UART_DebugTryRead(uint8_t *data)
{
    if (data == NULL || uart_rx_tail == uart_rx_head)
    {
        return false;
    }

    *data = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uint8_t)((uart_rx_tail + 1U) % BSP_UART_RX_BUFFER_SIZE);
    return true;
}

uint32_t BSP_UART_DebugRxOverflowCount(void)
{
    return uart_rx_overflow_count;
}

void BSP_DEBUG_UART_IRQ_HANDLER(void)
{
    uint8_t next_head;

    if (USART_GetITStatus(BSP_DEBUG_UART, USART_IT_RXNE) != RESET)
    {
        next_head = (uint8_t)((uart_rx_head + 1U) % BSP_UART_RX_BUFFER_SIZE);
        if (next_head != uart_rx_tail)
        {
            uart_rx_buffer[uart_rx_head] = (uint8_t)USART_ReceiveData(BSP_DEBUG_UART);
            uart_rx_head = next_head;
        }
        else
        {
            (void)USART_ReceiveData(BSP_DEBUG_UART);
            uart_rx_overflow_count++;
        }
    }
#if BSP_UART_ASYNC_TX
    if (USART_GetITStatus(BSP_DEBUG_UART, USART_IT_TXE) != RESET)
    {
        if (uart_tx_tail != uart_tx_head)
        {
            USART_SendData(BSP_DEBUG_UART, uart_tx_buffer[uart_tx_tail]);
            uart_tx_tail = (uint16_t)((uart_tx_tail + 1U) & (BSP_UART_TX_BUFFER_SIZE - 1U));
        }
        if (uart_tx_tail == uart_tx_head)
            USART_ITConfig(BSP_DEBUG_UART, USART_IT_TXE, DISABLE);
    }
#endif
}
