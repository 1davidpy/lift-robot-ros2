#include "excavator_protocol.h"

#include <stddef.h>

#define EXCAVATOR_PROTOCOL_CRC8_POLY  0x07U

static uint8_t ExcavatorProtocol_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    for (uint8_t i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x80U) != 0U)
                      ? (uint8_t)((crc << 1U) ^ EXCAVATOR_PROTOCOL_CRC8_POLY)
                      : (uint8_t)(crc << 1U);
        }
    }

    return crc;
}

bool ExcavatorProtocol_ValidateControlPacket(const ControlPacket_t *packet)
{
    if (packet == NULL ||
        packet->head1 != EXCAVATOR_PROTOCOL_HEAD1 ||
        packet->head2 != EXCAVATOR_PROTOCOL_HEAD2 ||
        packet->version != EXCAVATOR_PROTOCOL_VERSION ||
        packet->device_id != EXCAVATOR_PROTOCOL_DEVICE_ID ||
        packet->msg_type != EXCAVATOR_MSG_TYPE_VELOCITY_CMD ||
        packet->length != EXCAVATOR_PROTOCOL_CONTROL_LEN)
    {
        return false;
    }

    return packet->crc ==
           ExcavatorProtocol_Crc8((const uint8_t *)packet, (uint8_t)(sizeof(*packet) - 1U));
}
