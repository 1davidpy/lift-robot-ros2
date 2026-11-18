#include "excavator_protocol.h"

#include <stddef.h>

#define EXCAVATOR_PROTOCOL_CRC8_POLY  0x07U
#define EXCAVATOR_JOYSTICK_DEAD_ZONE  150U
#define EXCAVATOR_LEFT_X_CENTER       1967U
#define EXCAVATOR_LEFT_Y_CENTER       1967U
#define EXCAVATOR_RIGHT_X_CENTER      1975U
#define EXCAVATOR_RIGHT_Y_CENTER      2086U
#define EXCAVATOR_BUTTON_L1_MASK      (1U << 0U)
#define EXCAVATOR_BUTTON_R1_MASK      (1U << 1U)
#define EXCAVATOR_BUTTON_L3_MASK      (1U << 2U)
#define EXCAVATOR_BUTTON_R3_MASK      (1U << 3U)

static int8_t ExcavatorProtocol_MapJoystickAxis(uint16_t value,
                                                uint16_t center,
                                                bool invert)
{
    uint16_t neutral_low = (center > EXCAVATOR_JOYSTICK_DEAD_ZONE)
                               ? (uint16_t)(center - EXCAVATOR_JOYSTICK_DEAD_ZONE)
                               : 0U;
    uint16_t neutral_high = (uint16_t)(center + EXCAVATOR_JOYSTICK_DEAD_ZONE);
    int8_t level;

    if (value >= neutral_low && value <= neutral_high)
    {
        return 0;
    }

    level = (value < neutral_low) ? -EXCAVATOR_JOYSTICK_LEVEL_MAX
                                  : EXCAVATOR_JOYSTICK_LEVEL_MAX;

    return invert ? (int8_t)-level : level;
}

static uint16_t ExcavatorProtocol_JoystickDisplacement(uint16_t value,
                                                       uint16_t center)
{
    uint16_t displacement = (value >= center) ? (uint16_t)(value - center)
                                               : (uint16_t)(center - value);

    return (displacement > EXCAVATOR_JOYSTICK_DEAD_ZONE) ? displacement : 0U;
}

uint8_t ExcavatorProtocol_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ EXCAVATOR_PROTOCOL_CRC8_POLY);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

void ExcavatorProtocol_BuildVelocityFromAdc(const ExcavatorControlAdc_t *adc,
                                            ExcavatorVelocityCmd_t *command)
{
    uint16_t left_x_displacement;
    uint16_t left_y_displacement;
    uint16_t right_x_displacement;
    uint16_t right_y_displacement;

    if (adc == NULL || command == NULL)
    {
        return;
    }

    command->left_track = 0;
    command->right_track = 0;
    command->swing = 0;
    command->boom = 0;
    command->arm = 0;
    command->bucket = 0;
    command->gripper = 0;

    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        command->swing = ExcavatorProtocol_MapJoystickAxis(adc->left_x,
                                                            EXCAVATOR_LEFT_X_CENTER, true);
        return;
    }

    left_x_displacement = ExcavatorProtocol_JoystickDisplacement(adc->left_x,
                                                                  EXCAVATOR_LEFT_X_CENTER);
    left_y_displacement = ExcavatorProtocol_JoystickDisplacement(adc->left_y,
                                                                  EXCAVATOR_LEFT_Y_CENTER);
    right_x_displacement = ExcavatorProtocol_JoystickDisplacement(adc->right_x,
                                                                   EXCAVATOR_RIGHT_X_CENTER);
    right_y_displacement = ExcavatorProtocol_JoystickDisplacement(adc->right_y,
                                                                   EXCAVATOR_RIGHT_Y_CENTER);

    if (left_x_displacement >= left_y_displacement && left_x_displacement > 0U)
    {
        command->swing = ExcavatorProtocol_MapJoystickAxis(adc->left_x,
                                                            EXCAVATOR_LEFT_X_CENTER,
                                                            true);
    }
    else if (left_y_displacement > 0U)
    {
        command->arm = ExcavatorProtocol_MapJoystickAxis(adc->left_y,
                                                          EXCAVATOR_LEFT_Y_CENTER,
                                                          true);
    }

    if (right_y_displacement >= right_x_displacement && right_y_displacement > 0U)
    {
        command->boom = ExcavatorProtocol_MapJoystickAxis(adc->right_y,
                                                           EXCAVATOR_RIGHT_Y_CENTER,
                                                           false);
    }
    else if (right_x_displacement > 0U)
    {
        command->gripper = ExcavatorProtocol_MapJoystickAxis(adc->right_x,
                                                             EXCAVATOR_RIGHT_X_CENTER,
                                                             false);
    }
}

void ExcavatorProtocol_ApplyDriveButtons(uint8_t pressed_mask,
                                         uint8_t track_level,
                                         ExcavatorVelocityCmd_t *command)
{
    uint8_t drive_mask;
    int8_t level;

    if (command == NULL)
    {
        return;
    }

    track_level = EXCAVATOR_TRACK_LEVEL_DEFAULT;
    uint8_t pitch_buttons = pressed_mask & (EXCAVATOR_BUTTON_L3_MASK | EXCAVATOR_BUTTON_R3_MASK);
    command->bucket = pitch_buttons == EXCAVATOR_BUTTON_L3_MASK ? -EXCAVATOR_JOYSTICK_LEVEL_MAX :
                          pitch_buttons == EXCAVATOR_BUTTON_R3_MASK ? EXCAVATOR_JOYSTICK_LEVEL_MAX : 0;

    if (track_level < EXCAVATOR_TRACK_LEVEL_MIN)
    {
        track_level = EXCAVATOR_TRACK_LEVEL_MIN;
    }
    else if (track_level > EXCAVATOR_TRACK_LEVEL_MAX)
    {
        track_level = EXCAVATOR_TRACK_LEVEL_MAX;
    }

    level = (int8_t)track_level;
    drive_mask = pressed_mask &
                 (EXCAVATOR_BUTTON_L1_MASK |
                  EXCAVATOR_BUTTON_R1_MASK);
    command->left_track = 0;
    command->right_track = 0;

    if (pitch_buttons == (EXCAVATOR_BUTTON_L3_MASK | EXCAVATOR_BUTTON_R3_MASK))
    {
        command->left_track = (int8_t)-level;
        command->right_track = (int8_t)-level;
        return;
    }

    switch (drive_mask)
    {
        case EXCAVATOR_BUTTON_L1_MASK:
            command->left_track = (int8_t)-level;
            command->right_track = level;
            break;

        case EXCAVATOR_BUTTON_R1_MASK:
            command->left_track = level;
            command->right_track = (int8_t)-level;
            break;

        case EXCAVATOR_BUTTON_L1_MASK | EXCAVATOR_BUTTON_R1_MASK:
            command->left_track = level;
            command->right_track = level;
            break;

        default:
            break;
    }
}

void ExcavatorProtocol_BuildControlPacket(ControlPacket_t *packet,
                                          uint8_t seq,
                                          const ExcavatorVelocityCmd_t *command)
{
    if (packet == NULL || command == NULL)
    {
        return;
    }

    packet->head1 = EXCAVATOR_PROTOCOL_HEAD1;
    packet->head2 = EXCAVATOR_PROTOCOL_HEAD2;
    packet->version = EXCAVATOR_PROTOCOL_VERSION;
    packet->device_id = EXCAVATOR_PROTOCOL_DEVICE_ID;
    packet->msg_type = EXCAVATOR_MSG_TYPE_VELOCITY_CMD;
    packet->seq = seq;
    packet->length = EXCAVATOR_PROTOCOL_CONTROL_LEN;
    packet->left_track = command->left_track;
    packet->right_track = command->right_track;
    packet->swing = command->swing;
    packet->boom = command->boom;
    packet->arm = command->arm;
    packet->bucket = command->bucket;
    packet->gripper = command->gripper;
    packet->crc = ExcavatorProtocol_Crc8((const uint8_t *)packet, (uint8_t)(sizeof(*packet) - 1U));
}

bool ExcavatorProtocol_ValidateControlPacket(const ControlPacket_t *packet)
{
    uint8_t crc;

    if (packet == NULL)
    {
        return false;
    }

    if (packet->head1 != EXCAVATOR_PROTOCOL_HEAD1 ||
        packet->head2 != EXCAVATOR_PROTOCOL_HEAD2 ||
        packet->version != EXCAVATOR_PROTOCOL_VERSION ||
        packet->device_id != EXCAVATOR_PROTOCOL_DEVICE_ID ||
        packet->msg_type != EXCAVATOR_MSG_TYPE_VELOCITY_CMD ||
        packet->length != EXCAVATOR_PROTOCOL_CONTROL_LEN)
    {
        return false;
    }

    crc = ExcavatorProtocol_Crc8((const uint8_t *)packet, (uint8_t)(sizeof(*packet) - 1U));
    return crc == packet->crc;
}
