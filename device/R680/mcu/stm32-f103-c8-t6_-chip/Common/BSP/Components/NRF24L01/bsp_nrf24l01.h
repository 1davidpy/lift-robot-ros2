#ifndef BSP_NRF24L01_H
#define BSP_NRF24L01_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_NRF24L01_ADDRESS_WIDTH        5U
#define BSP_NRF24L01_MAX_PAYLOAD_LENGTH   32U
#define BSP_NRF24L01_DEFAULT_PAYLOAD_LEN  32U

#define BSP_NRF24L01_IRQ_RX_DR            0x40U
#define BSP_NRF24L01_IRQ_TX_DS            0x20U
#define BSP_NRF24L01_IRQ_MAX_RT           0x10U
#define BSP_NRF24L01_IRQ_ALL              0x70U

/**
 * @brief NRF24L01 operation result.
 */
typedef enum
{
    BSP_NRF24L01_OK = 0,
    BSP_NRF24L01_ERROR,
    BSP_NRF24L01_INVALID_ARGUMENT,
    BSP_NRF24L01_TIMEOUT,
    BSP_NRF24L01_TX_FAILED,
    BSP_NRF24L01_ACK_MISSING,
} BspNrf24l01Result;

/**
 * @brief NRF24L01 RF data rate.
 */
typedef enum
{
    BSP_NRF24L01_DATA_RATE_1MBPS = 0,
    BSP_NRF24L01_DATA_RATE_2MBPS,
    BSP_NRF24L01_DATA_RATE_250KBPS,
} BspNrf24l01DataRate;

/**
 * @brief NRF24L01 TX output power.
 */
typedef enum
{
    BSP_NRF24L01_POWER_NEG18DBM = 0,
    BSP_NRF24L01_POWER_NEG12DBM,
    BSP_NRF24L01_POWER_NEG6DBM,
    BSP_NRF24L01_POWER_0DBM,
} BspNrf24l01Power;

/**
 * @brief Millisecond delay callback used by the NRF24L01 driver.
 * @param ms Delay time in milliseconds.
 */
typedef void (*BspNrf24l01DelayMsCallback)(uint32_t ms);

/**
 * @brief Microsecond delay callback used by the NRF24L01 driver.
 * @param us Delay time in microseconds.
 */
typedef void (*BspNrf24l01DelayUsCallback)(uint32_t us);

/**
 * @brief Register delay callbacks used by NRF24L01 timing sequences.
 * @param delay_ms Millisecond delay callback. Pass null to use a blocking fallback.
 * @param delay_us Microsecond delay callback. Pass null to use a blocking fallback.
 */
void BSP_NRF24L01_SetDelayCallbacks(BspNrf24l01DelayMsCallback delay_ms,
                                    BspNrf24l01DelayUsCallback delay_us);

/**
 * @brief Initialize SPI1 GPIO pins, the SPI controller, and NRF24L01 defaults.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_Init(void);

/**
 * @brief Check whether the NRF24L01 register interface responds normally.
 * @return True if the setup register can be written and read back.
 */
bool BSP_NRF24L01_IsPresent(void);

/**
 * @brief Configure the RF channel.
 * @param channel RF channel from 0 to 125.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_SetChannel(uint8_t channel);

/**
 * @brief Configure the RF data rate and output power.
 * @param data_rate RF data rate.
 * @param power TX output power.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_SetRfConfig(BspNrf24l01DataRate data_rate,
                                           BspNrf24l01Power power);

/**
 * @brief Configure the TX address and pipe0 RX address used by auto ACK.
 * @param address Address buffer.
 * @param length Address length. NRF24L01 supports 3 to 5 bytes.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_SetAddress(const uint8_t *address, uint8_t length);

/**
 * @brief Configure fixed payload width for pipe0.
 * @param length Payload length from 1 to 32 bytes.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_SetPayloadLength(uint8_t length);

/**
 * @brief Enable NRF24L01+ ACK payload support on pipe0.
 * @return Operation result.
 * @warning The receiver must also support and preload ACK payload data.
 */
BspNrf24l01Result BSP_NRF24L01_EnableAckPayload(void);

/**
 * @brief Queue an ACK payload for one receive pipe.
 * @param pipe Receive pipe number from 0 to 5.
 * @param data ACK payload buffer.
 * @param length ACK payload length from 1 to 32 bytes.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_WriteAckPayload(uint8_t pipe,
                                               const uint8_t *data,
                                               uint8_t length);

/**
 * @brief Put the radio into receive mode.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_StartRx(void);

/**
 * @brief Put the radio into standby mode.
 */
void BSP_NRF24L01_Standby(void);

/**
 * @brief Send one payload and wait until TX succeeds or fails.
 * @param data Payload buffer.
 * @param length Payload length from 1 to the configured fixed payload width.
 * @param timeout_ms Polling timeout in milliseconds.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_Send(const uint8_t *data, uint8_t length, uint32_t timeout_ms);

/**
 * @brief Send one payload and read the ACK payload returned by the receiver.
 * @param data Payload buffer.
 * @param length Payload length from 1 to 32 bytes.
 * @param ack_buffer ACK payload destination buffer.
 * @param ack_buffer_size ACK payload destination buffer size.
 * @param ack_length Number of ACK payload bytes copied.
 * @param timeout_ms Polling timeout in milliseconds.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_SendWithAckPayload(const uint8_t *data,
                                                  uint8_t length,
                                                  uint8_t *ack_buffer,
                                                  uint8_t ack_buffer_size,
                                                  uint8_t *ack_length,
                                                  uint32_t timeout_ms);

/**
 * @brief Read one received payload from RX FIFO.
 * @param data Destination buffer.
 * @param max_length Destination buffer size.
 * @param out_length Number of bytes copied into data.
 * @return Operation result.
 */
BspNrf24l01Result BSP_NRF24L01_ReadPayload(uint8_t *data, uint8_t max_length, uint8_t *out_length);

/**
 * @brief Read the NRF24L01 status register.
 * @return Raw status register value.
 */
uint8_t BSP_NRF24L01_ReadStatus(void);

/**
 * @brief Clear IRQ flags in the status register.
 * @param irq_flags IRQ mask using BSP_NRF24L01_IRQ_* values.
 */
void BSP_NRF24L01_ClearIrq(uint8_t irq_flags);

/**
 * @brief Flush the TX FIFO.
 */
void BSP_NRF24L01_FlushTx(void);

/**
 * @brief Flush the RX FIFO.
 */
void BSP_NRF24L01_FlushRx(void);

/**
 * @brief Read the active-low IRQ pin state.
 * @return True when the NRF24L01 IRQ pin is low.
 */
bool BSP_NRF24L01_IsIrqActive(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_NRF24L01_H */
