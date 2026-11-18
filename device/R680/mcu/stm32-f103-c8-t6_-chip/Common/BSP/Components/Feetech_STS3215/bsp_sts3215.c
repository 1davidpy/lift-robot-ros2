#include "bsp_sts3215.h"

#include <stddef.h>
#include <stdbool.h>

#include "bsp_pin.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#define STS3215_BROADCAST_ID            0xFEU
#define STS3215_INST_SYNC_WRITE         0x83U
#define STS3215_INST_READ               0x02U
#define STS3215_INST_WRITE              0x03U
#define STS3215_ADDR_OPERATING_MODE     33U
#define STS3215_ADDR_POSITION_OFFSET    31U
#define STS3215_ADDR_POSITION_DEADBAND  26U
#define STS3215_ADDR_TORQUE_ENABLE      40U
#define STS3215_ADDR_ACCELERATION       41U
#define STS3215_ADDR_GOAL_POSITION      42U
#define STS3215_ADDR_GOAL_SPEED         46U
#define STS3215_ADDR_PRESENT_POSITION   56U
#define STS3215_POSITION_DATA_LENGTH    7U
#define STS3215_UART_TIMEOUT_LOOPS      100000U

static volatile BspSts3215Diagnostics receive_diagnostics;

static bool Sts3215_WriteAllowed(uint8_t id, uint8_t address)
{
#if defined(BSP_STS3215_READ_ONLY) && BSP_STS3215_READ_ONLY
    (void)id; (void)address;
    return false;
#else
    if (id == 0U || id >= STS3215_BROADCAST_ID) return false;
#ifdef BSP_STS3215_SPEED_WRITE_ID_MASK
    if (address == STS3215_ADDR_GOAL_SPEED &&
        (id > 32U || (BSP_STS3215_SPEED_WRITE_ID_MASK & (1UL << (id - 1U))) == 0U)) return false;
#endif
#ifdef BSP_STS3215_CALIBRATION_ID_MASK
    if (address == STS3215_ADDR_ACCELERATION &&
        (id > 32U || (BSP_STS3215_CALIBRATION_ID_MASK & (1UL << (id - 1U))) == 0U)) return false;
#else
    if (address == STS3215_ADDR_ACCELERATION) return false;
#endif
#ifdef BSP_STS3215_MODE_WRITE_ID_MASK
    if (address == STS3215_ADDR_OPERATING_MODE &&
        (id > 32U || (BSP_STS3215_MODE_WRITE_ID_MASK & (1UL << (id - 1U))) == 0U)) return false;
#endif
#ifdef BSP_STS3215_TORQUE_WRITE_ID_MASK
    if (address == STS3215_ADDR_TORQUE_ENABLE &&
        (id > 32U || (BSP_STS3215_TORQUE_WRITE_ID_MASK & (1UL << (id - 1U))) == 0U)) return false;
#endif
    return true;
#endif
}
#ifdef BSP_STS3215_UART_IRQN
#define STS3215_RX_BUFFER_SIZE 128U
static volatile uint8_t receive_buffer[STS3215_RX_BUFFER_SIZE];
static volatile uint8_t receive_head;
static volatile uint8_t receive_tail;

void BSP_STS3215_UART_IRQ_HANDLER(void)
{
    uint32_t status = BSP_STS3215_UART->SR;
    if ((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_FE |
                   USART_FLAG_NE | USART_FLAG_PE)) != 0U)
    {
        uint8_t byte = (uint8_t)USART_ReceiveData(BSP_STS3215_UART);
        if ((status & USART_FLAG_ORE) != 0U) receive_diagnostics.overruns++;
        if ((status & (USART_FLAG_FE | USART_FLAG_NE | USART_FLAG_PE)) != 0U)
        {
            receive_diagnostics.framing_errors++;
            return;
        }
        if ((status & USART_FLAG_RXNE) != 0U)
        {
            uint8_t next = (uint8_t)((receive_head + 1U) & (STS3215_RX_BUFFER_SIZE - 1U));
            receive_diagnostics.rx_bytes++;
            if (next == receive_tail)
            {
                receive_diagnostics.buffer_drops++;
            }
            else
            {
                receive_buffer[receive_head] = byte;
                receive_head = next;
            }
        }
    }
}
#endif

void BSP_STS3215_GetDiagnostics(BspSts3215Diagnostics *diagnostics)
{
    if (diagnostics != NULL) *diagnostics = receive_diagnostics;
}

static BspSts3215Result Sts3215_SendBytes(const uint8_t *data, uint16_t length)
{
    uint32_t timeout;

    for (uint16_t i = 0U; i < length; i++)
    {
        timeout = STS3215_UART_TIMEOUT_LOOPS;
        while (USART_GetFlagStatus(BSP_STS3215_UART, USART_FLAG_TXE) == RESET)
        {
            if (timeout-- == 0U)
            {
                return BSP_STS3215_TIMEOUT;
            }
        }

        USART_SendData(BSP_STS3215_UART, data[i]);
    }

    timeout = STS3215_UART_TIMEOUT_LOOPS;
    while (USART_GetFlagStatus(BSP_STS3215_UART, USART_FLAG_TC) == RESET)
    {
        if (timeout-- == 0U)
        {
            return BSP_STS3215_TIMEOUT;
        }
    }

    return BSP_STS3215_OK;
}

static void Sts3215_FlushReceiveBuffer(void)
{
#ifdef BSP_STS3215_UART_IRQN
    receive_tail = receive_head;
#else
    while (USART_GetFlagStatus(BSP_STS3215_UART, USART_FLAG_RXNE) != RESET)
    {
        (void)USART_ReceiveData(BSP_STS3215_UART);
    }
#endif
}

static BspSts3215Result Sts3215_ReadByte(uint8_t *data)
{
    uint32_t timeout = STS3215_UART_TIMEOUT_LOOPS;

#ifdef BSP_STS3215_UART_IRQN
    while (receive_tail == receive_head)
#else
    while (USART_GetFlagStatus(BSP_STS3215_UART, USART_FLAG_RXNE) == RESET)
#endif
    {
        if (timeout-- == 0U)
        {
            receive_diagnostics.timeouts++;
            return BSP_STS3215_TIMEOUT;
        }
    }

#ifdef BSP_STS3215_UART_IRQN
    *data = receive_buffer[receive_tail];
    receive_tail = (uint8_t)((receive_tail + 1U) & (STS3215_RX_BUFFER_SIZE - 1U));
#else
    *data = (uint8_t)USART_ReceiveData(BSP_STS3215_UART);
    receive_diagnostics.rx_bytes++;
#endif
    return BSP_STS3215_OK;
}

static BspSts3215Result Sts3215_ReadResponse(uint8_t id, uint8_t data_length,
                                           uint8_t *response,
                                           const uint8_t *echo, uint8_t echo_length)
{
    const uint8_t expected_length = (uint8_t)(data_length + 6U);
    uint8_t used = 0U;
    for (uint8_t received = 0U; received < 64U; received++)
    {
        uint8_t byte;
        BspSts3215Result result = Sts3215_ReadByte(&byte);
        if (result != BSP_STS3215_OK) return result;
        if (used == expected_length)
        {
            for (uint8_t i = 1U; i < used; i++) response[i - 1U] = response[i];
            used--;
        }
        response[used++] = byte;
        if (used == expected_length && response[0] == 0xFFU && response[1] == 0xFFU &&
            response[2] == id && response[3] == data_length + 2U)
        {
            uint8_t checksum = 0U;
            for (uint8_t i = 2U; i < expected_length - 1U; i++) checksum += response[i];
            if ((uint8_t)~checksum == response[expected_length - 1U])
            {
                bool is_echo = echo_length == expected_length;
                for (uint8_t i = 0U; is_echo && i < expected_length; i++)
                    is_echo = response[i] == echo[i];
                if (!is_echo) return BSP_STS3215_OK;
            }
        }
    }
    return BSP_STS3215_PROTOCOL_ERROR;
}

static BspSts3215Result Sts3215_ReadRegisters(uint8_t id,
                                               uint8_t address,
                                               uint8_t *data,
                                               uint8_t length)
{
    uint8_t request[8];
    uint8_t response[8];
    uint8_t checksum;
    BspSts3215Result result;

    if (data == NULL || length == 0U || length > 2U || id == STS3215_BROADCAST_ID)
    {
        return BSP_STS3215_INVALID_ARGUMENT;
    }

    request[0] = 0xFFU;
    request[1] = 0xFFU;
    request[2] = id;
    request[3] = 4U;
    request[4] = STS3215_INST_READ;
    request[5] = address;
    request[6] = length;
    request[7] = (uint8_t)~(id + 4U + STS3215_INST_READ + address + length);

    Sts3215_FlushReceiveBuffer();
    result = Sts3215_SendBytes(request, sizeof(request));
    if (result != BSP_STS3215_OK)
    {
        return result;
    }

    result = Sts3215_ReadResponse(id, length, response, request, sizeof(request));
    if (result != BSP_STS3215_OK) return result;

    if (response[0] != 0xFFU ||
        response[1] != 0xFFU ||
        response[2] != id ||
        response[3] != (uint8_t)(length + 2U))
    {
        return BSP_STS3215_PROTOCOL_ERROR;
    }

    checksum = (uint8_t)(response[2] + response[3] + response[4]);
    for (uint8_t i = 0U; i < length; i++)
    {
        data[i] = response[5U + i];
        checksum = (uint8_t)(checksum + data[i]);
    }

    if ((uint8_t)~checksum != response[5U + length])
    {
        return BSP_STS3215_PROTOCOL_ERROR;
    }

    return BSP_STS3215_OK;
}

static BspSts3215Result Sts3215_WriteRegisters(uint8_t id,
                                                uint8_t address,
                                                const uint8_t *data,
                                                uint8_t length)
{
#if defined(BSP_STS3215_READ_ONLY) && BSP_STS3215_READ_ONLY
    (void)id; (void)address; (void)data; (void)length;
    return BSP_STS3215_INVALID_ARGUMENT;
#else
    uint8_t packet[14];
    uint8_t response[6];
    uint8_t checksum;
    uint8_t index = 0U;
    BspSts3215Result result;

    if (data == NULL || length == 0U || length > 7U || !Sts3215_WriteAllowed(id, address))
    {
        return BSP_STS3215_INVALID_ARGUMENT;
    }

    packet[index++] = 0xFFU;
    packet[index++] = 0xFFU;
    packet[index++] = id;
    packet[index++] = (uint8_t)(length + 3U);
    packet[index++] = STS3215_INST_WRITE;
    packet[index++] = address;
    checksum = (uint8_t)(id + length + 3U + STS3215_INST_WRITE + address);

    for (uint8_t i = 0U; i < length; i++)
    {
        packet[index++] = data[i];
        checksum = (uint8_t)(checksum + data[i]);
    }

    packet[index++] = (uint8_t)~checksum;
    Sts3215_FlushReceiveBuffer();
    result = Sts3215_SendBytes(packet, index);
    if (result != BSP_STS3215_OK)
    {
        return result;
    }

    result = Sts3215_ReadResponse(id, 0U, response, packet, index);
    if (result == BSP_STS3215_TIMEOUT) return BSP_STS3215_OK;
    if (result != BSP_STS3215_OK) return result;

    if (response[0] != 0xFFU ||
        response[1] != 0xFFU ||
        response[2] != id ||
        response[3] != 2U ||
        (uint8_t)~(response[2] + response[3] + response[4]) != response[5])
    {
        return BSP_STS3215_PROTOCOL_ERROR;
    }

    return response[4] == 0U ? BSP_STS3215_OK : BSP_STS3215_PROTOCOL_ERROR;
#endif
}

BspSts3215Result BSP_STS3215_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;

    RCC_APB2PeriphClockCmd(BSP_STS3215_UART_TX_CLOCK | BSP_STS3215_UART_RX_CLOCK, ENABLE);
    RCC_APB1PeriphClockCmd(BSP_STS3215_UART_CLOCK, ENABLE);

    gpio_init.GPIO_Pin = BSP_STS3215_UART_TX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_STS3215_UART_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_STS3215_UART_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BSP_STS3215_UART_RX_PORT, &gpio_init);

    USART_StructInit(&usart_init);
    usart_init.USART_BaudRate = BSP_STS3215_UART_BAUD_RATE;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(BSP_STS3215_UART, &usart_init);
    receive_diagnostics = (BspSts3215Diagnostics){0};
#ifdef BSP_STS3215_UART_IRQN
    receive_head = 0U;
    receive_tail = 0U;
    /* This ISR never calls RTOS APIs; receive must preempt the servo/radio tasks. */
    NVIC_SetPriority(BSP_STS3215_UART_IRQN, BSP_STS3215_UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(BSP_STS3215_UART_IRQN);
    USART_ITConfig(BSP_STS3215_UART, USART_IT_RXNE, ENABLE);
#endif
    USART_Cmd(BSP_STS3215_UART, ENABLE);

    return BSP_STS3215_OK;
}

BspSts3215Result BSP_STS3215_SyncWritePosition(const BspSts3215PositionCommand *commands,
                                                uint8_t count)
{
#if (defined(BSP_STS3215_READ_ONLY) && BSP_STS3215_READ_ONLY) || \
    (defined(BSP_STS3215_POSITION_WRITES) && !BSP_STS3215_POSITION_WRITES)
    (void)commands; (void)count;
    return BSP_STS3215_INVALID_ARGUMENT;
#else
    uint8_t packet[7U + (BSP_STS3215_MAX_SYNC_SERVOS * (STS3215_POSITION_DATA_LENGTH + 1U)) + 1U];
    uint8_t packet_length;
    uint8_t checksum;
    uint16_t index = 0U;

    if (commands == NULL || count == 0U || count > BSP_STS3215_MAX_SYNC_SERVOS)
    {
        return BSP_STS3215_INVALID_ARGUMENT;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        if (commands[i].position > BSP_STS3215_POSITION_MAX)
        {
            return BSP_STS3215_INVALID_ARGUMENT;
        }
    }

    packet_length = (uint8_t)(4U + ((STS3215_POSITION_DATA_LENGTH + 1U) * count));
    packet[index++] = 0xFFU;
    packet[index++] = 0xFFU;
    packet[index++] = STS3215_BROADCAST_ID;
    packet[index++] = packet_length;
    packet[index++] = STS3215_INST_SYNC_WRITE;
    packet[index++] = STS3215_ADDR_ACCELERATION;
    packet[index++] = STS3215_POSITION_DATA_LENGTH;
    checksum = (uint8_t)(STS3215_BROADCAST_ID + packet_length + STS3215_INST_SYNC_WRITE +
                         STS3215_ADDR_ACCELERATION + STS3215_POSITION_DATA_LENGTH);

    for (uint8_t i = 0U; i < count; i++)
    {
        const BspSts3215PositionCommand *command = &commands[i];

        packet[index++] = command->id;
        packet[index++] = command->acceleration;
        packet[index++] = (uint8_t)(command->position & 0xFFU);
        packet[index++] = (uint8_t)(command->position >> 8U);
        packet[index++] = 0U;
        packet[index++] = 0U;
        packet[index++] = (uint8_t)(command->speed & 0xFFU);
        packet[index++] = (uint8_t)(command->speed >> 8U);

        for (uint8_t data_index = 0U; data_index <= STS3215_POSITION_DATA_LENGTH; data_index++)
        {
            checksum = (uint8_t)(checksum + packet[index - STS3215_POSITION_DATA_LENGTH - 1U + data_index]);
        }
    }

    packet[index++] = (uint8_t)~checksum;
    return Sts3215_SendBytes(packet, index);
#endif
}

BspSts3215Result BSP_STS3215_ReadPresentPosition(uint8_t id, uint16_t *position)
{
    uint8_t data[2];
    BspSts3215Result result;

    if (position == NULL || id == STS3215_BROADCAST_ID)
    {
        return BSP_STS3215_INVALID_ARGUMENT;
    }

    result = Sts3215_ReadRegisters(id, STS3215_ADDR_PRESENT_POSITION, data, sizeof(data));
    if (result != BSP_STS3215_OK)
    {
        return result;
    }

    *position = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    if (*position > BSP_STS3215_POSITION_MAX)
    {
        return BSP_STS3215_PROTOCOL_ERROR;
    }

    return BSP_STS3215_OK;
}

BspSts3215Result BSP_STS3215_ReadGoalPosition(uint8_t id, uint16_t *position)
{
    uint8_t data[2];
    if (position == NULL) return BSP_STS3215_INVALID_ARGUMENT;
    BspSts3215Result result = Sts3215_ReadRegisters(id, STS3215_ADDR_GOAL_POSITION, data, 2U);
    if (result == BSP_STS3215_OK)
    {
        *position = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
        if (*position > BSP_STS3215_POSITION_MAX) return BSP_STS3215_PROTOCOL_ERROR;
    }
    return result;
}

BspSts3215Result BSP_STS3215_ReadPositionOffset(uint8_t id, uint16_t *offset)
{
    uint8_t data[2];
    if (offset == NULL) return BSP_STS3215_INVALID_ARGUMENT;
    BspSts3215Result result = Sts3215_ReadRegisters(id, STS3215_ADDR_POSITION_OFFSET, data, 2U);
    if (result == BSP_STS3215_OK) *offset = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return result;
}

BspSts3215Result BSP_STS3215_ReadPositionDeadband(uint8_t id, uint8_t *positive, uint8_t *negative)
{
    uint8_t data[2];
    if (positive == NULL || negative == NULL) return BSP_STS3215_INVALID_ARGUMENT;
    BspSts3215Result result = Sts3215_ReadRegisters(id, STS3215_ADDR_POSITION_DEADBAND, data, 2U);
    if (result == BSP_STS3215_OK) { *positive = data[0]; *negative = data[1]; }
    return result;
}

BspSts3215Result BSP_STS3215_ReadRegisterWord(uint8_t id, uint8_t address, uint16_t *value)
{
    uint8_t data[2];
    if (value == NULL || address == 255U) return BSP_STS3215_INVALID_ARGUMENT;
    BspSts3215Result result = Sts3215_ReadRegisters(id, address, data, 2U);
    if (result == BSP_STS3215_OK) *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return result;
}

static BspSts3215Result Sts3215_WriteBoundedPosition(const BspSts3215PositionCommand *command,
                                                    uint16_t maximum_difference, uint16_t maximum_speed, bool require_torque)
{
#if !defined(BSP_STS3215_CALIBRATION_ID_MASK) || (defined(BSP_STS3215_READ_ONLY) && BSP_STS3215_READ_ONLY)
    (void)command;
    (void)maximum_difference;
    (void)maximum_speed;
    (void)require_torque;
    return BSP_STS3215_INVALID_ARGUMENT;
#else
    uint8_t mode;
    uint16_t current;
    if (command == NULL || command->position > BSP_STS3215_POSITION_MAX ||
        command->speed == 0U || command->speed > maximum_speed ||
        !Sts3215_WriteAllowed(command->id, STS3215_ADDR_ACCELERATION)) return BSP_STS3215_INVALID_ARGUMENT;
    if (BSP_STS3215_ReadOperatingMode(command->id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_POSITION ||
        BSP_STS3215_ReadPresentPosition(command->id, &current) != BSP_STS3215_OK)
        return BSP_STS3215_PROTOCOL_ERROR;
    uint16_t difference = current > command->position ? current - command->position : command->position - current;
    if (difference > maximum_difference) return BSP_STS3215_INVALID_ARGUMENT;
    uint8_t torque;
    if (require_torque && (BSP_STS3215_ReadTorqueEnabled(command->id, &torque) != BSP_STS3215_OK || torque != 1U))
        return BSP_STS3215_PROTOCOL_ERROR;
    uint8_t data[7] = {command->acceleration, (uint8_t)command->position,
                      (uint8_t)(command->position >> 8U), 0U, 0U,
                      (uint8_t)command->speed, (uint8_t)(command->speed >> 8U)};
    BspSts3215Result result = Sts3215_WriteRegisters(command->id, STS3215_ADDR_ACCELERATION, data, 7U);
    uint16_t goal, speed;
    if (result == BSP_STS3215_OK && (BSP_STS3215_ReadGoalPosition(command->id, &goal) != BSP_STS3215_OK ||
        goal != command->position || BSP_STS3215_ReadGoalSpeed(command->id, &speed) != BSP_STS3215_OK ||
        speed != command->speed)) return BSP_STS3215_PROTOCOL_ERROR;
    return result;
#endif
}

BspSts3215Result BSP_STS3215_WriteCalibrationPosition(const BspSts3215PositionCommand *command)
{
#ifdef BSP_STS3215_CALIBRATION_STEP_MAX
    return Sts3215_WriteBoundedPosition(command, BSP_STS3215_CALIBRATION_STEP_MAX, BSP_STS3215_CALIBRATION_SPEED_MAX, false);
#else
    return Sts3215_WriteBoundedPosition(command, 0U, 0U, false);
#endif
}

BspSts3215Result BSP_STS3215_WriteRemotePosition(const BspSts3215PositionCommand *command)
{
#if defined(BSP_STS3215_REMOTE_STEP_MAX) && defined(BSP_STS3215_REMOTE_SPEED_MAX) && BSP_STS3215_REMOTE_STEP_MAX > 0
    return Sts3215_WriteBoundedPosition(command, BSP_STS3215_REMOTE_STEP_MAX, BSP_STS3215_REMOTE_SPEED_MAX, true);
#else
    (void)command;
    return BSP_STS3215_INVALID_ARGUMENT;
#endif
}

BspSts3215Result BSP_STS3215_ReadOperatingMode(uint8_t id, uint8_t *mode)
{
    return Sts3215_ReadRegisters(id, STS3215_ADDR_OPERATING_MODE, mode, 1U);
}

BspSts3215Result BSP_STS3215_ReadGoalSpeed(uint8_t id, uint16_t *speed)
{
    uint8_t data[2];
    if (speed == NULL) return BSP_STS3215_INVALID_ARGUMENT;
    BspSts3215Result result = Sts3215_ReadRegisters(id, STS3215_ADDR_GOAL_SPEED, data, 2U);
    if (result == BSP_STS3215_OK) *speed = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return result;
}

BspSts3215Result BSP_STS3215_SetOperatingMode(uint8_t id, uint8_t mode)
{
#ifdef BSP_STS3215_MODE_WRITE_VALUE
    uint8_t torque;
    bool mode_allowed = mode == BSP_STS3215_MODE_WRITE_VALUE;
#ifdef BSP_STS3215_CALIBRATION_ID_MASK
    bool position_id = id > 0U && id <= 32U &&
                       (BSP_STS3215_CALIBRATION_ID_MASK & (1UL << (id - 1U))) != 0U;
    mode_allowed = position_id ? mode == BSP_STS3215_MODE_POSITION : mode_allowed;
#endif
    if (!mode_allowed || !Sts3215_WriteAllowed(id, STS3215_ADDR_OPERATING_MODE)) return BSP_STS3215_INVALID_ARGUMENT;
    if (BSP_STS3215_ReadTorqueEnabled(id, &torque) != BSP_STS3215_OK || torque != 0U)
        return BSP_STS3215_PROTOCOL_ERROR;
#endif
    return Sts3215_WriteRegisters(id, STS3215_ADDR_OPERATING_MODE, &mode, 1U);
}

BspSts3215Result BSP_STS3215_ReadTorqueEnabled(uint8_t id, uint8_t *enabled)
{
    if (enabled == NULL) return BSP_STS3215_INVALID_ARGUMENT;
    BspSts3215Result result = Sts3215_ReadRegisters(id, STS3215_ADDR_TORQUE_ENABLE, enabled, 1U);
    if (result == BSP_STS3215_OK && *enabled > 1U) return BSP_STS3215_PROTOCOL_ERROR;
    return result;
}

BspSts3215Result BSP_STS3215_SetTorqueEnabled(uint8_t id, uint8_t enabled)
{
    if (enabled > 1U || !Sts3215_WriteAllowed(id, STS3215_ADDR_TORQUE_ENABLE))
        return BSP_STS3215_INVALID_ARGUMENT;
#ifdef BSP_STS3215_MODE_WRITE_VALUE
    if (enabled != 0U)
    {
        uint8_t mode;
        uint16_t speed;
        if (BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK)
            return BSP_STS3215_PROTOCOL_ERROR;
#ifdef BSP_STS3215_CALIBRATION_ID_MASK
        bool position_id = id > 0U && id <= 32U &&
                           (BSP_STS3215_CALIBRATION_ID_MASK & (1UL << (id - 1U))) != 0U;
        if (position_id)
        {
            uint16_t goal, current;
            if (mode != BSP_STS3215_MODE_POSITION ||
                BSP_STS3215_ReadGoalPosition(id, &goal) != BSP_STS3215_OK ||
                BSP_STS3215_ReadPresentPosition(id, &current) != BSP_STS3215_OK ||
                (uint16_t)(goal > current ? goal - current : current - goal) > 2U ||
                BSP_STS3215_ReadGoalSpeed(id, &speed) != BSP_STS3215_OK || speed == 0U ||
                speed > BSP_STS3215_CALIBRATION_SPEED_MAX) return BSP_STS3215_PROTOCOL_ERROR;
        }
        else
#endif
        if (mode != BSP_STS3215_MODE_WHEEL || BSP_STS3215_ReadGoalSpeed(id, &speed) != BSP_STS3215_OK || speed != 0U)
            return BSP_STS3215_PROTOCOL_ERROR;
    }
#endif
    BspSts3215Result result = Sts3215_WriteRegisters(id, STS3215_ADDR_TORQUE_ENABLE, &enabled, 1U);
#ifdef BSP_STS3215_MODE_WRITE_VALUE
    uint8_t verified;
    if (result == BSP_STS3215_OK &&
        (BSP_STS3215_ReadTorqueEnabled(id, &verified) != BSP_STS3215_OK || verified != enabled))
        return BSP_STS3215_PROTOCOL_ERROR;
#endif
    return result;
}

BspSts3215Result BSP_STS3215_WriteGoalSpeed(uint8_t id, uint16_t speed)
{
    if (!Sts3215_WriteAllowed(id, STS3215_ADDR_GOAL_SPEED)) return BSP_STS3215_INVALID_ARGUMENT;
#ifdef BSP_STS3215_SPEED_WRITE_MAX
    if ((speed & 0x7FFFU) > BSP_STS3215_SPEED_WRITE_MAX) return BSP_STS3215_INVALID_ARGUMENT;
#endif
#if defined(BSP_STS3215_SPEED_REQUIRE_WHEEL) && BSP_STS3215_SPEED_REQUIRE_WHEEL
    uint8_t mode;
    if (BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_WHEEL)
        return BSP_STS3215_PROTOCOL_ERROR;
    uint8_t torque;
    if (speed != 0U && (BSP_STS3215_ReadTorqueEnabled(id, &torque) != BSP_STS3215_OK || torque != 1U))
        return BSP_STS3215_PROTOCOL_ERROR;
#endif
    uint8_t data[2] = {
        (uint8_t)(speed & 0xFFU),
        (uint8_t)(speed >> 8U),
    };

    BspSts3215Result result = Sts3215_WriteRegisters(id, STS3215_ADDR_GOAL_SPEED, data, sizeof(data));
#if defined(BSP_STS3215_SPEED_REQUIRE_WHEEL) && BSP_STS3215_SPEED_REQUIRE_WHEEL
    uint16_t verified_speed;
    if (result == BSP_STS3215_OK &&
        (BSP_STS3215_ReadGoalSpeed(id, &verified_speed) != BSP_STS3215_OK || verified_speed != speed))
        return BSP_STS3215_PROTOCOL_ERROR;
#endif
    return result;
}
