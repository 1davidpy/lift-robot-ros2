#ifndef EXCAVATOR_PROTOCOL_H
#define EXCAVATOR_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EXCAVATOR_PROTOCOL_HEAD1          0xAAU
#define EXCAVATOR_PROTOCOL_HEAD2          0x55U
#define EXCAVATOR_PROTOCOL_VERSION        0x02U
#define EXCAVATOR_PROTOCOL_DEVICE_ID      0x02U
#define EXCAVATOR_PROTOCOL_CONTROL_LEN    7U
#define EXCAVATOR_PROTOCOL_PACKET_LEN     15U
#define EXCAVATOR_PROTOCOL_ACK_LEN        2U
#define EXCAVATOR_PROTOCOL_ACK_OK_1       0x4FU
#define EXCAVATOR_PROTOCOL_ACK_OK_2       0x4BU

#define EXCAVATOR_TRACK_LEVEL_MIN         1
#define EXCAVATOR_TRACK_LEVEL_DEFAULT     3
#define EXCAVATOR_CHASSIS_TEST_MODE       0
#define EXCAVATOR_TRACK_LEVEL_MAX         6
#define EXCAVATOR_JOYSTICK_LEVEL_MAX      10

/**
 * @brief Excavator control message type.
 */
typedef enum
{
    EXCAVATOR_MSG_TYPE_VELOCITY_CMD = 0x01U,
} ExcavatorMsgType;

/**
 * @brief Excavator velocity command payload.
 */
typedef struct
{
    int8_t left_track;
    int8_t right_track;
    int8_t swing;
    int8_t boom;
    int8_t arm;
    int8_t bucket; ///< ID3 pitch from BL/BR.
    int8_t gripper; ///< ID8 opening from RX.
} ExcavatorVelocityCmd_t;

/**
 * @brief Raw ADC values used to build an excavator velocity command.
 */
typedef struct
{
    uint16_t left_y;
    uint16_t left_x;
    uint16_t right_y;
    uint16_t right_x;
    uint16_t wheel_left;
    uint16_t wheel_right;
} ExcavatorControlAdc_t;

/**
 * @brief Full excavator control packet.
 */
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
    int8_t bucket; ///< ID3 pitch from BL/BR.
    int8_t gripper; ///< ID8 opening from RX.
    uint8_t crc;
} ControlPacket_t;

_Static_assert(sizeof(ControlPacket_t) == 15U, "ControlPacket_t must stay byte-packed");

/**
 * @brief Convert both joysticks into velocity channels, selecting only the
 *        greater-displacement axis within each joystick.
 * @param adc Raw ADC input values.
 * @param command Velocity command to fill.
 */
void ExcavatorProtocol_BuildVelocityFromAdc(const ExcavatorControlAdc_t *adc,
                                            ExcavatorVelocityCmd_t *command);

/**
 * @brief Apply fixed chassis buttons and BL/BR pitch direction.
 * @param pressed_mask Button mask with TL, TR, BL, and BR at bits 0 through 3.
 * @param track_level Selected chassis speed level from 1 through 6.
 * @param command Velocity command to update.
 */
void ExcavatorProtocol_ApplyDriveButtons(uint8_t pressed_mask,
                                         uint8_t track_level,
                                         ExcavatorVelocityCmd_t *command);

/**
 * @brief Calculate CRC8 for a byte buffer.
 * @param data Data buffer.
 * @param length Number of bytes to include in the CRC.
 * @return CRC8 value.
 */
uint8_t ExcavatorProtocol_Crc8(const uint8_t *data, uint8_t length);

/**
 * @brief Fill a control packet from a velocity command.
 * @param packet Packet to fill.
 * @param seq Packet sequence number.
 * @param command Velocity command payload.
 */
void ExcavatorProtocol_BuildControlPacket(ControlPacket_t *packet,
                                          uint8_t seq,
                                          const ExcavatorVelocityCmd_t *command);

/**
 * @brief Validate packet header, length, and CRC8.
 * @param packet Packet to validate.
 * @return True if the packet is valid.
 */
bool ExcavatorProtocol_ValidateControlPacket(const ControlPacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* EXCAVATOR_PROTOCOL_H */
