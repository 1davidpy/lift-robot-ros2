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
                                            uint8_t *active_axis,
                                            ExcavatorVelocityCmd_t *command)
{
    uint16_t displacement[4];
    int8_t level[4];
    uint16_t largest_displacement = 0U;
    uint8_t selected_axis = 4U;

    if (adc == NULL || active_axis == NULL || command == NULL)
    {
        return;
    }

    command->left_track = 0;
    command->right_track = 0;
    command->swing = 0;
    command->boom = 0;
    command->arm = 0;
    command->bucket = 0;

    displacement[0] = ExcavatorProtocol_JoystickDisplacement(adc->left_x,
                                                              EXCAVATOR_LEFT_X_CENTER);
    displacement[1] = ExcavatorProtocol_JoystickDisplacement(adc->right_y,
                                                              EXCAVATOR_RIGHT_Y_CENTER);
    displacement[2] = ExcavatorProtocol_JoystickDisplacement(adc->left_y,
                                                              EXCAVATOR_LEFT_Y_CENTER);
    displacement[3] = ExcavatorProtocol_JoystickDisplacement(adc->right_x,
                                                              EXCAVATOR_RIGHT_X_CENTER);
    level[0] = ExcavatorProtocol_MapJoystickAxis(adc->left_x,
                                                 EXCAVATOR_LEFT_X_CENTER,
                                                 true);
    level[1] = ExcavatorProtocol_MapJoystickAxis(adc->right_y,
                                                 EXCAVATOR_RIGHT_Y_CENTER,
                                                 false);
    level[2] = ExcavatorProtocol_MapJoystickAxis(adc->left_y,
                                                 EXCAVATOR_LEFT_Y_CENTER,
                                                 true);
    level[3] = ExcavatorProtocol_MapJoystickAxis(adc->right_x,
                                                 EXCAVATOR_RIGHT_X_CENTER,
                                                 false);

    if (*active_axis < 4U && displacement[*active_axis] > 0U)
    {
        selected_axis = *active_axis;
    }
    else
    {
        for (uint8_t axis = 0U; axis < 4U; axis++)
        {
            if (displacement[axis] > largest_displacement)
            {
                largest_displacement = displacement[axis];
                selected_axis = axis;
            }
        }
    }

    *active_axis = selected_axis;

    switch (selected_axis)
    {
        case 0U:
            command->swing = level[0];
            break;

        case 1U:
            command->boom = level[1];
            break;

        case 2U:
            command->arm = level[2];
            break;

        case 3U:
            command->bucket = level[3];
            break;

        default:
            break;
    }
}

void ExcavatorProtocol_ApplyDriveButtons(uint8_t pressed_mask,
                                         ExcavatorVelocityCmd_t *command)
{
    uint8_t drive_mask;

    if (command == NULL)
    {
        return;
    }

    drive_mask = pressed_mask &
                 (EXCAVATOR_BUTTON_L1_MASK |
                  EXCAVATOR_BUTTON_R1_MASK |
                  EXCAVATOR_BUTTON_L3_MASK |
                  EXCAVATOR_BUTTON_R3_MASK);
    command->left_track = 0;
    command->right_track = 0;

    switch (drive_mask)
    {
        case EXCAVATOR_BUTTON_L1_MASK:
            command->left_track = EXCAVATOR_TRACK_LEVEL_MAX;
            command->right_track = -EXCAVATOR_TRACK_LEVEL_MAX;
            break;

        case EXCAVATOR_BUTTON_R1_MASK:
            command->left_track = -EXCAVATOR_TRACK_LEVEL_MAX;
            command->right_track = EXCAVATOR_TRACK_LEVEL_MAX;
            break;

        case EXCAVATOR_BUTTON_L1_MASK | EXCAVATOR_BUTTON_R1_MASK:
            command->left_track = EXCAVATOR_TRACK_LEVEL_MAX;
            command->right_track = EXCAVATOR_TRACK_LEVEL_MAX;
            break;

        case EXCAVATOR_BUTTON_L3_MASK | EXCAVATOR_BUTTON_R3_MASK:
            command->left_track = -EXCAVATOR_TRACK_LEVEL_MAX;
            command->right_track = -EXCAVATOR_TRACK_LEVEL_MAX;
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
        packet->length != EXCAVATOR_PROTOCOL_CONTROL_LEN)
    {
        return false;
    }

    crc = ExcavatorProtocol_Crc8((const uint8_t *)packet, (uint8_t)(sizeof(*packet) - 1U));
    return crc == packet->crc;
}
