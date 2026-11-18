#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Debug UART operation result.
 */
typedef enum
{
    BSP_UART_OK = 0,
    BSP_UART_INVALID_ARGUMENT,
    BSP_UART_TIMEOUT,
} BspUartResult;

/**
 * @brief Initialize the project-configured debug UART.
 */
void BSP_UART_DebugInit(void);

/**
 * @brief Send bytes through the project-configured debug UART.
 * @param data Byte buffer to send.
 * @param length Number of bytes to send.
 * @return UART operation result.
 * @warning With BSP_UART_ASYNC_TX enabled by the project, OK means the entire
 * buffer was copied to the interrupt-driven TX queue, not sent on the wire.
 * Exactly one task must own calls to this API in asynchronous mode.
 * A full queue returns TIMEOUT without queuing any bytes. Do not reinitialize
 * the UART or disable its interrupt while queued data remains.
 */
BspUartResult BSP_UART_DebugWrite(const uint8_t *data, uint16_t length);

/**
 * @brief Read one received byte without blocking.
 * @param data Destination for the received byte.
 * @return True when a byte was read.
 */
bool BSP_UART_DebugTryRead(uint8_t *data);

/**
 * @brief Return the cumulative count of bytes dropped because the RX ring was full.
 */
uint32_t BSP_UART_DebugRxOverflowCount(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
