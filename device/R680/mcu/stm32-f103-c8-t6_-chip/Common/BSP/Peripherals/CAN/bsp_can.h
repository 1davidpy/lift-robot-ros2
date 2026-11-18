#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_CAN_FRAME_DATA_MAX  8U

/**
 * @brief CAN operation result.
 */
typedef enum
{
    BSP_CAN_OK = 0,
    BSP_CAN_INVALID_ARGUMENT,
    BSP_CAN_INIT_ERROR,
    BSP_CAN_TIMEOUT,
    BSP_CAN_BUSY,
    BSP_CAN_BUS_OFF,
} BspCanResult;

/**
 * @brief One received CAN frame.
 */
typedef struct
{
    uint32_t id;
    uint8_t length;
    uint8_t data[BSP_CAN_FRAME_DATA_MAX];
    bool extended;
} BspCanFrame;

/**
 * @brief CAN transport counters and current bus-off state.
 */
typedef struct
{
    uint32_t rx_overflows; ///< Hardware overrun events plus software queue drops.
    uint32_t rx_rejected; ///< Invalid DLC, remote frames, or unwanted identifiers.
    uint32_t tx_failures; ///< Failed/expired frames and synchronous submit failures.
    uint32_t bus_off; ///< Nonzero while the controller reports bus-off.
} BspCanDiagnostics;

/**
 * @brief Initialize the project-configured CAN peripheral.
 * @return CAN operation result.
 * @warning IRQ reception requires one task consumer. Project configuration may
 *          select BSP_CAN_REMAP, BSP_CAN_SWJ_REMAP, BSP_CAN_RX_STANDARD_ID, BSP_CAN_RX_IRQ_ENABLED,
 *          BSP_CAN_NO_AUTO_RETRY, and BSP_CAN_TX_TIMEOUT_MS through bsp_pin.h.
 */
BspCanResult BSP_CAN_Init(void);

/**
 * @brief Transmit one extended CAN data frame.
 * @param id 29-bit extended frame identifier.
 * @param data Payload buffer.
 * @param length Payload length from 1 to 8 bytes.
 * @return CAN operation result.
 */
BspCanResult BSP_CAN_TransmitExtended(uint32_t id, const uint8_t *data, uint8_t length);

/**
 * @brief Submit one standard data frame without waiting for transmission.
 * @param id 11-bit standard identifier.
 * @param data Payload buffer; may be null only when length is zero.
 * @param length Payload length from 0 to 8 bytes.
 * @return OK means submitted, BUSY means no mailbox, BUS_OFF means unavailable.
 * @warning Use one task to own transmit APIs and call BSP_CAN_Service regularly.
 *          Submission is not confirmation that another node received the frame.
 */
BspCanResult BSP_CAN_TryTransmitStandard(uint16_t id, const uint8_t *data, uint8_t length);

/**
 * @brief Retire completed frames and cancel expired asynchronous transmissions.
 * @param now_ms Monotonic millisecond counter; unsigned wraparound is supported.
 * @warning Call after initialization and periodically from the transmit owner.
 *          A timeout requests cancellation; hardware completion is asynchronous.
 */
void BSP_CAN_Service(uint32_t now_ms);

/**
 * @brief Read transport counters without waiting or clearing them.
 * @param diagnostics Output snapshot; null is ignored.
 * @warning Each counter is an atomic read; an interrupt may occur between fields.
 */
void BSP_CAN_GetDiagnostics(BspCanDiagnostics *diagnostics);

/**
 * @brief Read one CAN frame without blocking.
 * @param frame Destination frame.
 * @return True when a frame was read.
 */
bool BSP_CAN_TryReceive(BspCanFrame *frame);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
