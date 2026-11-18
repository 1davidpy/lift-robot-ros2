#ifndef LIFT_UART_PROTOCOL_H
#define LIFT_UART_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LIFT_UART_PROTOCOL_VERSION 1U
#define LIFT_UART_SESSION_MS 500U
#define LIFT_UART_SET_CENTER_KEY 0x33455453L
#define LIFT_UART_RESET_KEY 0x52535421L

#define LIFT_UART_SOF_0 0xA5U
#define LIFT_UART_SOF_1 0x5AU
#define LIFT_UART_FRAME_REQUEST 0x01U
#define LIFT_UART_FRAME_ACK 0x81U
#define LIFT_UART_FRAME_TELEMETRY 0x90U
#define LIFT_UART_REQUEST_DATA_SIZE 8U
#define LIFT_UART_ACK_DATA_SIZE 8U
#define LIFT_UART_TELEMETRY_DATA_SIZE 9U
#define LIFT_UART_MAX_DATA_SIZE 16U
#define LIFT_UART_MAX_FRAME_SIZE (LIFT_UART_MAX_DATA_SIZE + 6U)

enum
{
    LIFT_UART_HELLO = 0x01, LIFT_UART_KEEPALIVE = 0x02,
    LIFT_UART_STOP = 0x03, LIFT_UART_INFO = 0x04, LIFT_UART_CLOSE = 0x05,
    LIFT_UART_MOVE_UM = 0x10, LIFT_UART_HOME = 0x11,
    LIFT_UART_SET_CENTER = 0x12, LIFT_UART_ABS_CHECK = 0x13,
    LIFT_UART_ABS_READ = 0x14, LIFT_UART_GOTO_POINT = 0x16,
    LIFT_UART_RESTORE_REF = 0x17, LIFT_UART_SPEED = 0x18,
    LIFT_UART_BOOT_HOME = 0x19, LIFT_UART_RESET = 0x1A
};

enum
{
    LIFT_UART_OK = 0, LIFT_UART_BAD_FRAME = 0x80,
    LIFT_UART_BAD_OPCODE = 0x81, LIFT_UART_NO_SESSION = 0x82,
    LIFT_UART_BUSY = 0x83, LIFT_UART_SEQ_CONFLICT = 0x84,
    LIFT_UART_BAD_VALUE = 0x85, LIFT_UART_LINK_LOST = 0x86,
    LIFT_UART_QUEUED = 0xFE
};

typedef struct
{
    uint8_t opcode;
    uint8_t sequence;
    int32_t value;
    uint8_t argument;
} LiftUartRequest;

typedef struct
{
    uint16_t sequence;
    int32_t current_um;
    int32_t target_um;
    int16_t speed_rpm;
    uint16_t fault;
    uint8_t target_status;
    uint8_t flags;
    uint8_t raw_mask;
    uint8_t stable_mask;
    uint8_t result;
    uint8_t guard;
    uint32_t uptime_ms;
    uint16_t feedback_age_ms;
} LiftUartTelemetry;

/** @brief Decode and validate the eight-byte command payload. */
uint8_t LiftUartProtocol_DecodeRequest(const uint8_t data[8], LiftUartRequest *request);

/** @brief Encode the eight-byte ACK payload. */
void LiftUartProtocol_EncodeAck(uint8_t opcode, uint8_t sequence, uint8_t result,
                                uint8_t flags, int32_t value, uint8_t data[8]);

/** @brief Encode one of four eight-byte telemetry pages. */
bool LiftUartProtocol_EncodeTelemetry(const LiftUartTelemetry *sample, unsigned page,
                                      uint8_t data[8]);

/** @brief Calculate CRC-16/CCITT-FALSE, polynomial 0x1021, initial 0xFFFF. */
uint16_t LiftUartProtocol_Crc16(const uint8_t *data, size_t length);

/**
 * @brief Wrap a typed data payload in the UART frame envelope.
 * @return Encoded byte count, or zero for invalid arguments/capacity.
 */
size_t LiftUartProtocol_EncodeFrame(uint8_t type, const uint8_t *data, uint8_t data_length,
                                    uint8_t *frame, size_t capacity);

/**
 * @brief Validate and unwrap one complete UART frame.
 * @return True only for an exact-length frame with a valid CRC.
 */
bool LiftUartProtocol_DecodeFrame(const uint8_t *frame, size_t frame_length,
                                  uint8_t *type, const uint8_t **data,
                                  uint8_t *data_length);

#endif /* LIFT_UART_PROTOCOL_H */
