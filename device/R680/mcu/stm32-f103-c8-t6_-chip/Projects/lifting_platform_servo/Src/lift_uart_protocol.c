#include "lift_uart_protocol.h"

#include <limits.h>
#include <stddef.h>

static void Put16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void Put32(uint8_t *data, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) data[i] = (uint8_t)(value >> (8U * i));
}

uint8_t LiftUartProtocol_DecodeRequest(const uint8_t data[8], LiftUartRequest *request)
{
    if (data == NULL || request == NULL || data[7] != LIFT_UART_PROTOCOL_VERSION)
        return LIFT_UART_BAD_FRAME;
    uint32_t bits = (uint32_t)data[2] | (uint32_t)data[3] << 8 |
                    (uint32_t)data[4] << 16 | (uint32_t)data[5] << 24;
    request->opcode = data[0];
    request->sequence = data[1];
    request->value = bits <= INT32_MAX ? (int32_t)bits : -1 - (int32_t)(UINT32_MAX - bits);
    request->argument = data[6];
    if (request->opcode == LIFT_UART_GOTO_POINT)
        return request->value == 0 && request->argument == 3U ?
            LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    if (request->argument != 0U) return LIFT_UART_BAD_VALUE;
    switch (request->opcode)
    {
    case LIFT_UART_HELLO:
    case LIFT_UART_KEEPALIVE:
    case LIFT_UART_CLOSE:
        return request->value != 0 ? LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_INFO:
        return request->value >= 0 && request->value <= 30 ? LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_MOVE_UM:
        return request->value >= -180000 && request->value <= 180000 ?
            LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_SPEED:
        return request->value >= 120 && request->value <= 3000 ?
            LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_BOOT_HOME:
        return request->value == 0 || request->value == 1 ?
            LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_SET_CENTER:
        return request->value == LIFT_UART_SET_CENTER_KEY ?
            LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_RESET:
        return request->value == LIFT_UART_RESET_KEY ? LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    case LIFT_UART_STOP:
    case LIFT_UART_HOME:
    case LIFT_UART_ABS_CHECK:
    case LIFT_UART_ABS_READ:
    case LIFT_UART_RESTORE_REF:
        return request->value == 0 ? LIFT_UART_OK : LIFT_UART_BAD_VALUE;
    default:
        return LIFT_UART_BAD_OPCODE;
    }
}

void LiftUartProtocol_EncodeAck(uint8_t opcode, uint8_t sequence, uint8_t result,
                                uint8_t flags, int32_t value, uint8_t data[8])
{
    if (data == NULL) return;
    data[0] = opcode;
    data[1] = sequence;
    data[2] = result;
    data[3] = flags;
    Put32(data + 4, (uint32_t)value);
}

bool LiftUartProtocol_EncodeTelemetry(const LiftUartTelemetry *sample, unsigned page,
                                      uint8_t data[8])
{
    if (sample == NULL || data == NULL || page > 3U) return false;
    Put16(data, sample->sequence);
    switch (page)
    {
    case 0:
        Put32(data + 2, (uint32_t)sample->current_um);
        Put16(data + 6, (uint16_t)sample->speed_rpm);
        break;
    case 1:
        Put32(data + 2, (uint32_t)sample->target_um);
        data[6] = sample->target_status;
        data[7] = sample->flags;
        break;
    case 2:
        Put16(data + 2, sample->fault);
        data[4] = sample->raw_mask;
        data[5] = sample->stable_mask;
        data[6] = sample->result;
        data[7] = sample->guard;
        break;
    default:
        Put32(data + 2, sample->uptime_ms);
        Put16(data + 6, sample->feedback_age_ms);
        break;
    }
    return true;
}

uint16_t LiftUartProtocol_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL && length != 0U) return 0U;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8U; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}

size_t LiftUartProtocol_EncodeFrame(uint8_t type, const uint8_t *data, uint8_t data_length,
                                    uint8_t *frame, size_t capacity)
{
    uint8_t payload_length = (uint8_t)(data_length + 1U);
    size_t total = (size_t)payload_length + 5U;
    if (frame == NULL || type == 0U || data_length > LIFT_UART_MAX_DATA_SIZE ||
        (data_length != 0U && data == NULL) || capacity < total) return 0U;
    frame[0] = LIFT_UART_SOF_0;
    frame[1] = LIFT_UART_SOF_1;
    frame[2] = payload_length;
    frame[3] = type;
    for (uint8_t i = 0; i < data_length; ++i) frame[4U + i] = data[i];
    uint16_t crc = LiftUartProtocol_Crc16(frame + 2, (size_t)payload_length + 1U);
    frame[4U + data_length] = (uint8_t)crc;
    frame[5U + data_length] = (uint8_t)(crc >> 8);
    return total;
}

bool LiftUartProtocol_DecodeFrame(const uint8_t *frame, size_t frame_length,
                                  uint8_t *type, const uint8_t **data,
                                  uint8_t *data_length)
{
    if (frame == NULL || type == NULL || data == NULL || data_length == NULL ||
        frame_length < 6U || frame[0] != LIFT_UART_SOF_0 || frame[1] != LIFT_UART_SOF_1)
        return false;
    uint8_t payload_length = frame[2];
    if (payload_length < 1U || payload_length > LIFT_UART_MAX_DATA_SIZE + 1U ||
        frame_length != (size_t)payload_length + 5U) return false;
    uint16_t expected = (uint16_t)frame[frame_length - 2U] |
                        (uint16_t)frame[frame_length - 1U] << 8;
    if (LiftUartProtocol_Crc16(frame + 2, (size_t)payload_length + 1U) != expected)
        return false;
    *type = frame[3];
    *data = frame + 4;
    *data_length = (uint8_t)(payload_length - 1U);
    return true;
}
