#ifndef EXCAVATOR_PROTOCOL_H
#define EXCAVATOR_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define EXCAVATOR_PROTOCOL_HEAD1          0xAAU
#define EXCAVATOR_PROTOCOL_HEAD2          0x55U
#define EXCAVATOR_PROTOCOL_VERSION        0x01U
#define EXCAVATOR_PROTOCOL_DEVICE_ID      0x01U
#define EXCAVATOR_PROTOCOL_CONTROL_LEN    6U
#define EXCAVATOR_PROTOCOL_PACKET_LEN     14U
#define EXCAVATOR_PROTOCOL_ACK_LEN        2U
#define EXCAVATOR_PROTOCOL_ACK_OK_1       0x4FU
#define EXCAVATOR_PROTOCOL_ACK_OK_2       0x4BU

#define EXCAVATOR_TRACK_LEVEL_MAX         3
#define EXCAVATOR_JOYSTICK_LEVEL_MAX      10

typedef enum
{
    EXCAVATOR_MSG_TYPE_VELOCITY_CMD = 0x01U,
} ExcavatorMsgType;

typedef struct
{
    int8_t left_track;
    int8_t right_track;
    int8_t swing;
    int8_t boom;
    int8_t arm;
    int8_t bucket;
} ExcavatorVelocityCmd_t;

typedef struct
{
    uint8_t head1;
    uint8_t head2;
    uint8_t version;
    uint8_t device_id;
    uint8_t msg_type;
    uint8_t seq;
    uint8_t length;
    int8_t left_track;
    int8_t right_track;
    int8_t swing;
    int8_t boom;
    int8_t arm;
    int8_t bucket;
    uint8_t crc;
} ControlPacket_t;

_Static_assert(sizeof(ControlPacket_t) == EXCAVATOR_PROTOCOL_PACKET_LEN,
               "ControlPacket_t must stay byte-packed");

/**
 * @brief Validate a received excavator control packet.
 * @param packet Packet to validate.
 * @return True when header, type, length, and CRC are valid.
 */
bool ExcavatorProtocol_ValidateControlPacket(const ControlPacket_t *packet);

#endif /* EXCAVATOR_PROTOCOL_H */
