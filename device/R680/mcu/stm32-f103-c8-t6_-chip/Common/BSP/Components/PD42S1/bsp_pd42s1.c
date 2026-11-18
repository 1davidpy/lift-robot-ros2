#include "bsp_pd42s1.h"

#include <stddef.h>
#include <string.h>

#include "bsp_can.h"

#define PD42S1_CAN_EXTENDED_ID          0x1000U
#define PD42S1_FRAME_HEAD               0xC5U
#define PD42S1_FRAME_TAIL               0x5CU
#define PD42S1_FUNCTION_SPEED_MODE      0xF1U
#define PD42S1_FUNCTION_POSITION_MODE   0xF2U
#define PD42S1_FUNCTION_ZERO_POSITION   0xF8U
#define PD42S1_FUNCTION_READ_SPEED      0x29U
#define PD42S1_FUNCTION_READ_POSITION   0x2AU
#define PD42S1_FUNCTION_CLEAR_STATE     0xFBU
#define PD42S1_FUNCTION_STOP            0xFCU
#define PD42S1_SPEED_COMMAND_LENGTH     11U
#define PD42S1_POSITION_COMMAND_LENGTH  14U
#define PD42S1_READ_SPEED_RESPONSE_LEN  8U
#define PD42S1_READ_POSITION_RESPONSE_LEN  10U
#define PD42S1_RESPONSE_SUCCESS         0x01U
#define PD42S1_RESPONSE_TIMEOUT_LOOPS   1000000U

static uint8_t PD42S1_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0U;

    for (uint8_t i = 0U; i < length; i++)
    {
        checksum = (uint8_t)(checksum + data[i]);
    }

    return checksum;
}

static BspPd42s1Result PD42S1_Send(const uint8_t *data, uint8_t length)
{
    uint8_t offset = 0U;

    while (offset < length)
    {
        uint8_t remaining = (uint8_t)(length - offset);
        uint8_t frame_length = (remaining > BSP_CAN_FRAME_DATA_MAX)
                                   ? BSP_CAN_FRAME_DATA_MAX
                                   : remaining;

        if (BSP_CAN_TransmitExtended(PD42S1_CAN_EXTENDED_ID,
                                     &data[offset],
                                     frame_length) != BSP_CAN_OK)
        {
            return BSP_PD42S1_CAN_ERROR;
        }
        offset = (uint8_t)(offset + frame_length);
    }

    return BSP_PD42S1_OK;
}

static BspPd42s1Result PD42S1_SendSimpleCommand(uint8_t address, uint8_t function)
{
    uint8_t command[5];

    if (address == 0U)
    {
        return BSP_PD42S1_INVALID_ARGUMENT;
    }

    command[0] = PD42S1_FRAME_HEAD;
    command[1] = address;
    command[2] = function;
    command[3] = PD42S1_Checksum(command, 3U);
    command[4] = PD42S1_FRAME_TAIL;
    return PD42S1_Send(command, sizeof(command));
}

static BspPd42s1Result PD42S1_ReadResponse(uint8_t address,
                                           uint8_t function,
                                           uint8_t *response,
                                           uint8_t response_length)
{
    BspCanFrame frame;
    uint32_t timeout = PD42S1_RESPONSE_TIMEOUT_LOOPS;
    uint8_t offset = 0U;

    if (address == BSP_PD42S1_BROADCAST_ADDRESS || response == NULL ||
        response_length < 5U)
    {
        return BSP_PD42S1_INVALID_ARGUMENT;
    }

    if (PD42S1_SendSimpleCommand(address, function) != BSP_PD42S1_OK)
    {
        return BSP_PD42S1_CAN_ERROR;
    }

    while (timeout-- > 0U)
    {
        uint8_t copy_length;

        if (!BSP_CAN_TryReceive(&frame))
        {
            continue;
        }

        if (!frame.extended || frame.length == 0U)
        {
            continue;
        }

        if (offset == 0U)
        {
            if (frame.length < 3U || frame.data[0] != PD42S1_FRAME_HEAD ||
                frame.data[1] != address || frame.data[2] != function)
            {
                continue;
            }
        }

        copy_length = frame.length;
        if (copy_length > (uint8_t)(response_length - offset))
        {
            copy_length = (uint8_t)(response_length - offset);
        }
        memcpy(&response[offset], frame.data, copy_length);
        offset = (uint8_t)(offset + copy_length);

        if (offset == response_length)
        {
            if (response[response_length - 1U] != PD42S1_FRAME_TAIL ||
                response[response_length - 2U] !=
                    PD42S1_Checksum(response, (uint8_t)(response_length - 2U)) ||
                response[3] != PD42S1_RESPONSE_SUCCESS)
            {
                return BSP_PD42S1_PROTOCOL_ERROR;
            }

            return BSP_PD42S1_OK;
        }
    }

    return BSP_PD42S1_TIMEOUT;
}

BspPd42s1Result BSP_PD42S1_Init(void)
{
    return (BSP_CAN_Init() == BSP_CAN_OK) ? BSP_PD42S1_OK : BSP_PD42S1_CAN_ERROR;
}

BspPd42s1Result BSP_PD42S1_SetSpeed(uint8_t address,
                                    uint8_t direction,
                                    uint8_t acceleration,
                                    float speed_rpm)
{
    uint8_t command[PD42S1_SPEED_COMMAND_LENGTH] = {0};
    uint32_t speed_bits;

    if (direction > BSP_PD42S1_DIRECTION_CCW ||
        acceleration > BSP_PD42S1_ACCELERATION_MAX ||
        !(speed_rpm >= 0.0F && speed_rpm <= BSP_PD42S1_SPEED_RPM_MAX))
    {
        return BSP_PD42S1_INVALID_ARGUMENT;
    }

    memcpy(&speed_bits, &speed_rpm, sizeof(speed_bits));
    command[0] = PD42S1_FRAME_HEAD;
    command[1] = address;
    command[2] = PD42S1_FUNCTION_SPEED_MODE;
    command[3] = direction;
    command[4] = acceleration;
    command[5] = (uint8_t)(speed_bits >> 24U);
    command[6] = (uint8_t)(speed_bits >> 16U);
    command[7] = (uint8_t)(speed_bits >> 8U);
    command[8] = (uint8_t)speed_bits;
    command[9] = PD42S1_Checksum(command, 9U);
    command[10] = PD42S1_FRAME_TAIL;

    return PD42S1_Send(command, sizeof(command));
}

BspPd42s1Result BSP_PD42S1_SetAbsolutePosition(uint8_t address,
                                               uint8_t direction,
                                               uint8_t acceleration,
                                               uint16_t speed_rpm,
                                               uint32_t position_pulses)
{
    uint8_t command[PD42S1_POSITION_COMMAND_LENGTH] = {0};

    if (address == BSP_PD42S1_BROADCAST_ADDRESS ||
        direction > BSP_PD42S1_DIRECTION_CCW ||
        acceleration > BSP_PD42S1_ACCELERATION_MAX ||
        speed_rpm == 0U || (float)speed_rpm > BSP_PD42S1_SPEED_RPM_MAX)
    {
        return BSP_PD42S1_INVALID_ARGUMENT;
    }

    command[0] = PD42S1_FRAME_HEAD;
    command[1] = address;
    command[2] = PD42S1_FUNCTION_POSITION_MODE;
    command[3] = direction;
    command[4] = acceleration;
    command[5] = (uint8_t)(speed_rpm >> 8U);
    command[6] = (uint8_t)speed_rpm;
    command[7] = (uint8_t)(position_pulses >> 24U);
    command[8] = (uint8_t)(position_pulses >> 16U);
    command[9] = (uint8_t)(position_pulses >> 8U);
    command[10] = (uint8_t)position_pulses;
    command[11] = 0U;
    command[12] = PD42S1_Checksum(command, 12U);
    command[13] = PD42S1_FRAME_TAIL;

    return PD42S1_Send(command, sizeof(command));
}

BspPd42s1Result BSP_PD42S1_ZeroPosition(uint8_t address)
{
    return PD42S1_SendSimpleCommand(address, PD42S1_FUNCTION_ZERO_POSITION);
}

BspPd42s1Result BSP_PD42S1_ClearState(uint8_t address)
{
    return PD42S1_SendSimpleCommand(address, PD42S1_FUNCTION_CLEAR_STATE);
}

BspPd42s1Result BSP_PD42S1_Stop(uint8_t address)
{
    return PD42S1_SendSimpleCommand(address, PD42S1_FUNCTION_STOP);
}

BspPd42s1Result BSP_PD42S1_ReadSpeed(uint8_t address, int16_t *speed_rpm)
{
    uint8_t response[PD42S1_READ_SPEED_RESPONSE_LEN];
    BspPd42s1Result result;

    if (address == 0U || speed_rpm == NULL)
    {
        return BSP_PD42S1_INVALID_ARGUMENT;
    }

    result = PD42S1_ReadResponse(address,
                                 PD42S1_FUNCTION_READ_SPEED,
                                 response,
                                 sizeof(response));
    if (result == BSP_PD42S1_OK)
    {
        *speed_rpm = (int16_t)(((uint16_t)response[4] << 8U) | response[5]);
    }

    return result;
}

BspPd42s1Result BSP_PD42S1_ReadPosition(uint8_t address, int32_t *position_pulses)
{
    uint8_t response[PD42S1_READ_POSITION_RESPONSE_LEN];
    BspPd42s1Result result;
    uint32_t position_bits;

    if (address == BSP_PD42S1_BROADCAST_ADDRESS || position_pulses == NULL)
    {
        return BSP_PD42S1_INVALID_ARGUMENT;
    }

    result = PD42S1_ReadResponse(address,
                                 PD42S1_FUNCTION_READ_POSITION,
                                 response,
                                 sizeof(response));
    if (result == BSP_PD42S1_OK)
    {
        position_bits = ((uint32_t)response[4] << 24U) |
                        ((uint32_t)response[5] << 16U) |
                        ((uint32_t)response[6] << 8U) |
                        (uint32_t)response[7];
        *position_pulses = (int32_t)position_bits;
    }

    return result;
}
