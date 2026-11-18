#include "bsp_isl60pr.h"

#include <stddef.h>
#include <string.h>

#include "bsp_pin.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#define ISL60PR_FUNCTION_READ_HOLDING       0x03U
#define ISL60PR_FUNCTION_WRITE_SINGLE       0x06U
#define ISL60PR_FUNCTION_WRITE_MULTIPLE     0x10U

#define ISL60PR_REG_CONTROL_MODE            0x0200U  /* P02-00 */
#define ISL60PR_REG_DO1_FUNCTION            0x0400U  /* P04-00 */
#define ISL60PR_REG_EEPROM_WRITE_ENABLE     0x0C0DU  /* P0C-13 */
#define ISL60PR_REG_WORD_ORDER              0x0C1AU  /* P0C-26 */
#define ISL60PR_REG_BUS_CONTROL             0x0D08U  /* P0D-08 */
#define ISL60PR_REG_FORCE_DIDO_ENABLE       0x0D11U  /* P0D-17 */
#define ISL60PR_REG_FORCE_DI_VALUE          0x0D12U  /* P0D-18 */
#define ISL60PR_REG_BUS_OPERATION_MODE      0x1003U  /* P10-03 */
#define ISL60PR_REG_POSITION_ARRIVAL_WINDOW 0x1007U  /* P10-07 */
#define ISL60PR_REG_PP_TARGET_POSITION      0x100EU  /* P10-14 */
#define ISL60PR_REG_PP_SPEED                0x1019U  /* P10-25 */
#define ISL60PR_REG_PV_ACCELERATION         0x101BU  /* P10-27 */
#define ISL60PR_REG_PV_DECELERATION         0x101DU  /* P10-29 */
#define ISL60PR_REG_PV_SPEED                0x102AU  /* P10-42 */

#define ISL60PR_CONTROL_MODE_MODBUS         9U
#define ISL60PR_OPERATION_MODE_PP           1U
#define ISL60PR_OPERATION_MODE_PV           3U
#define ISL60PR_DO_FUNCTION_BRAKE           9U
#define ISL60PR_FORCE_DI_ONLY               1U
#define ISL60PR_FORCE_DIDO_DISABLED         0U
#define ISL60PR_ALL_DI_INACTIVE             0x01FFU
#define ISL60PR_DI3_SERVO_ENABLED           0x01FBU
#define ISL60PR_LOW_WORD_FIRST              1U
#define ISL60PR_CONTROL_RELATIVE_POSITION   0x0001U
#define ISL60PR_CONTROL_STOP                0x0100U
#define ISL60PR_CONTROL_EMERGENCY_STOP      0x0200U

#define ISL60PR_READ_REQUEST_LENGTH         8U
#define ISL60PR_READ_RESPONSE_LENGTH        7U
#define ISL60PR_READ32_RESPONSE_LENGTH      9U
#define ISL60PR_WRITE_REQUEST_LENGTH        8U
#define ISL60PR_WRITE_RESPONSE_LENGTH       8U
#define ISL60PR_WRITE32_REQUEST_LENGTH      13U
#define ISL60PR_EXCEPTION_RESPONSE_LENGTH   5U

#ifndef BSP_ISL60PR_RESPONSE_TIMEOUT_LOOPS
#define BSP_ISL60PR_RESPONSE_TIMEOUT_LOOPS  1500000U
#endif

#ifndef BSP_ISL60PR_TX_TIMEOUT_LOOPS
#define BSP_ISL60PR_TX_TIMEOUT_LOOPS        200000U
#endif

static uint8_t isl60pr_last_exception;
static uint8_t isl60pr_last_function;
static uint16_t isl60pr_last_register_address;
static uint32_t isl60pr_last_write_value;

static uint16_t ISL60PR_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length-- > 0U)
    {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static void ISL60PR_AppendCrc(uint8_t *frame, uint16_t data_length)
{
    uint16_t crc = ISL60PR_Crc16(frame, data_length);

    frame[data_length] = (uint8_t)(crc & 0xFFU);
    frame[data_length + 1U] = (uint8_t)(crc >> 8U);
}

static void ISL60PR_SetTransmitMode(bool transmit)
{
    if (transmit)
    {
        GPIO_SetBits(BSP_ISL60PR_DE_PORT, BSP_ISL60PR_DE_PIN);
    }
    else
    {
        GPIO_ResetBits(BSP_ISL60PR_DE_PORT, BSP_ISL60PR_DE_PIN);
    }
}

static void ISL60PR_InterFrameDelay(void)
{
    volatile uint32_t count = SystemCoreClock / 1000U;

    while (count-- > 0U)
    {
        __NOP();
    }
}

static void ISL60PR_FlushReceive(void)
{
    while ((BSP_ISL60PR_UART->SR & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                                    USART_SR_FE | USART_SR_PE)) != 0U)
    {
        (void)BSP_ISL60PR_UART->DR;
    }
}

static BspIsl60prResult ISL60PR_Transmit(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0U; i < length; i++)
    {
        uint32_t timeout = BSP_ISL60PR_TX_TIMEOUT_LOOPS;

        while (USART_GetFlagStatus(BSP_ISL60PR_UART, USART_FLAG_TXE) == RESET)
        {
            if (timeout-- == 0U)
            {
                return BSP_ISL60PR_TIMEOUT;
            }
        }
        USART_SendData(BSP_ISL60PR_UART, data[i]);
    }

    uint32_t timeout = BSP_ISL60PR_TX_TIMEOUT_LOOPS;
    while (USART_GetFlagStatus(BSP_ISL60PR_UART, USART_FLAG_TC) == RESET)
    {
        if (timeout-- == 0U)
        {
            return BSP_ISL60PR_TIMEOUT;
        }
    }

    return BSP_ISL60PR_OK;
}

static BspIsl60prResult ISL60PR_ReceiveByte(uint8_t *value)
{
    uint32_t timeout = BSP_ISL60PR_RESPONSE_TIMEOUT_LOOPS;

    while ((BSP_ISL60PR_UART->SR & USART_SR_RXNE) == 0U)
    {
        uint32_t status = BSP_ISL60PR_UART->SR;

        if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
        {
            (void)BSP_ISL60PR_UART->DR;
            return BSP_ISL60PR_IO_ERROR;
        }
        if (timeout-- == 0U)
        {
            return BSP_ISL60PR_TIMEOUT;
        }
    }

    *value = (uint8_t)(USART_ReceiveData(BSP_ISL60PR_UART) & 0xFFU);
    return BSP_ISL60PR_OK;
}

static BspIsl60prResult ISL60PR_Transaction(const uint8_t *request,
                                            uint16_t request_length,
                                            uint8_t *response,
                                            uint16_t normal_response_length)
{
    uint16_t expected_length = normal_response_length;
    uint16_t received_length = 0U;
    BspIsl60prResult result;

    isl60pr_last_exception = 0U;
    ISL60PR_InterFrameDelay();
    ISL60PR_FlushReceive();
    ISL60PR_SetTransmitMode(true);
    result = ISL60PR_Transmit(request, request_length);
    ISL60PR_SetTransmitMode(false);
    if (result != BSP_ISL60PR_OK)
    {
        return result;
    }

    while (received_length < expected_length)
    {
        result = ISL60PR_ReceiveByte(&response[received_length]);
        if (result != BSP_ISL60PR_OK)
        {
            return result;
        }
        received_length++;

        if (received_length == 2U &&
            response[1] == (uint8_t)(request[1] | 0x80U))
        {
            expected_length = ISL60PR_EXCEPTION_RESPONSE_LENGTH;
        }
    }

    uint16_t response_crc = (uint16_t)response[expected_length - 2U] |
                            ((uint16_t)response[expected_length - 1U] << 8U);
    if (ISL60PR_Crc16(response, (uint16_t)(expected_length - 2U)) != response_crc)
    {
        return BSP_ISL60PR_CRC_ERROR;
    }
    if (response[0] != BSP_ISL60PR_SLAVE_ADDRESS)
    {
        return BSP_ISL60PR_BAD_RESPONSE;
    }
    if ((response[1] & 0x80U) != 0U)
    {
        isl60pr_last_exception = response[2];
        return BSP_ISL60PR_EXCEPTION;
    }
    if (response[1] != request[1])
    {
        return BSP_ISL60PR_BAD_RESPONSE;
    }

    return BSP_ISL60PR_OK;
}

static BspIsl60prResult ISL60PR_WriteIfDifferent(uint16_t register_address, uint16_t value)
{
    uint16_t current_value;
    BspIsl60prResult result = BSP_ISL60PR_ReadRegister(register_address, &current_value);

    if (result != BSP_ISL60PR_OK)
    {
        return result;
    }
    if (current_value == value)
    {
        return BSP_ISL60PR_OK;
    }
    return BSP_ISL60PR_WriteRegister(register_address, value);
}

static BspIsl60prResult ISL60PR_PulseControlWord(uint16_t command)
{
    BspIsl60prResult result =
        BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_CONTROL, command);

    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_CONTROL, 0U);
    }

    return result;
}

void BSP_ISL60PR_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    USART_InitTypeDef usart_init;

    RCC_APB2PeriphClockCmd(BSP_ISL60PR_TX_CLOCK | BSP_ISL60PR_RX_CLOCK |
                          BSP_ISL60PR_DE_CLOCK,
                          ENABLE);
    BSP_ISL60PR_UART_CLOCK_ENABLE();

    gpio_init.GPIO_Pin = BSP_ISL60PR_TX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_ISL60PR_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_ISL60PR_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BSP_ISL60PR_RX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_ISL60PR_DE_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(BSP_ISL60PR_DE_PORT, &gpio_init);
    ISL60PR_SetTransmitMode(false);

    USART_StructInit(&usart_init);
    usart_init.USART_BaudRate = BSP_ISL60PR_BAUD_RATE;
    usart_init.USART_WordLength = USART_WordLength_9b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_Even;
    usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(BSP_ISL60PR_UART, &usart_init);
    USART_Cmd(BSP_ISL60PR_UART, ENABLE);
    ISL60PR_FlushReceive();
}

BspIsl60prResult BSP_ISL60PR_ReadRegister(uint16_t register_address, uint16_t *value)
{
    uint8_t request[ISL60PR_READ_REQUEST_LENGTH] = {
        BSP_ISL60PR_SLAVE_ADDRESS,
        ISL60PR_FUNCTION_READ_HOLDING,
        (uint8_t)(register_address >> 8U),
        (uint8_t)(register_address & 0xFFU),
        0x00U,
        0x01U,
        0x00U,
        0x00U,
    };
    uint8_t response[ISL60PR_READ_RESPONSE_LENGTH];

    isl60pr_last_function = ISL60PR_FUNCTION_READ_HOLDING;
    isl60pr_last_register_address = register_address;

    if (value == NULL)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }
    ISL60PR_AppendCrc(request, ISL60PR_READ_REQUEST_LENGTH - 2U);
    BspIsl60prResult result = ISL60PR_Transaction(request,
                                                  ISL60PR_READ_REQUEST_LENGTH,
                                                  response,
                                                  ISL60PR_READ_RESPONSE_LENGTH);
    if (result != BSP_ISL60PR_OK)
    {
        return result;
    }
    if (response[2] != 2U)
    {
        return BSP_ISL60PR_BAD_RESPONSE;
    }

    *value = ((uint16_t)response[3] << 8U) | response[4];
    return BSP_ISL60PR_OK;
}

BspIsl60prResult BSP_ISL60PR_ReadRegister32(uint16_t register_address, uint32_t *value)
{
    uint8_t request[ISL60PR_READ_REQUEST_LENGTH] = {
        BSP_ISL60PR_SLAVE_ADDRESS,
        ISL60PR_FUNCTION_READ_HOLDING,
        (uint8_t)(register_address >> 8U),
        (uint8_t)(register_address & 0xFFU),
        0x00U,
        0x02U,
        0x00U,
        0x00U,
    };
    uint8_t response[ISL60PR_READ32_RESPONSE_LENGTH];
    uint16_t low_word;
    uint16_t high_word;

    isl60pr_last_function = ISL60PR_FUNCTION_READ_HOLDING;
    isl60pr_last_register_address = register_address;

    if (value == NULL)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }

    ISL60PR_AppendCrc(request, ISL60PR_READ_REQUEST_LENGTH - 2U);
    BspIsl60prResult result = ISL60PR_Transaction(request,
                                                  ISL60PR_READ_REQUEST_LENGTH,
                                                  response,
                                                  ISL60PR_READ32_RESPONSE_LENGTH);
    if (result != BSP_ISL60PR_OK)
    {
        return result;
    }
    if (response[2] != 4U)
    {
        return BSP_ISL60PR_BAD_RESPONSE;
    }

    low_word = ((uint16_t)response[3] << 8U) | response[4];
    high_word = ((uint16_t)response[5] << 8U) | response[6];
    *value = ((uint32_t)high_word << 16U) | low_word;
    return BSP_ISL60PR_OK;
}

BspIsl60prResult BSP_ISL60PR_WriteRegister(uint16_t register_address, uint16_t value)
{
    uint8_t request[ISL60PR_WRITE_REQUEST_LENGTH] = {
        BSP_ISL60PR_SLAVE_ADDRESS,
        ISL60PR_FUNCTION_WRITE_SINGLE,
        (uint8_t)(register_address >> 8U),
        (uint8_t)(register_address & 0xFFU),
        (uint8_t)(value >> 8U),
        (uint8_t)(value & 0xFFU),
        0x00U,
        0x00U,
    };
    uint8_t response[ISL60PR_WRITE_RESPONSE_LENGTH];

    isl60pr_last_function = ISL60PR_FUNCTION_WRITE_SINGLE;
    isl60pr_last_register_address = register_address;
    isl60pr_last_write_value = value;

    ISL60PR_AppendCrc(request, ISL60PR_WRITE_REQUEST_LENGTH - 2U);
    BspIsl60prResult result = ISL60PR_Transaction(request,
                                                  ISL60PR_WRITE_REQUEST_LENGTH,
                                                  response,
                                                  ISL60PR_WRITE_RESPONSE_LENGTH);
    if (result != BSP_ISL60PR_OK)
    {
        return result;
    }
    if (memcmp(request, response, ISL60PR_WRITE_RESPONSE_LENGTH) != 0)
    {
        return BSP_ISL60PR_BAD_RESPONSE;
    }

    return BSP_ISL60PR_OK;
}

BspIsl60prResult BSP_ISL60PR_WriteRegister32(uint16_t register_address, uint32_t value)
{
    uint16_t low_word = (uint16_t)(value & 0xFFFFU);
    uint16_t high_word = (uint16_t)(value >> 16U);
    uint8_t request[ISL60PR_WRITE32_REQUEST_LENGTH] = {
        BSP_ISL60PR_SLAVE_ADDRESS,
        ISL60PR_FUNCTION_WRITE_MULTIPLE,
        (uint8_t)(register_address >> 8U),
        (uint8_t)(register_address & 0xFFU),
        0x00U,
        0x02U,
        0x04U,
        (uint8_t)(low_word >> 8U),
        (uint8_t)(low_word & 0xFFU),
        (uint8_t)(high_word >> 8U),
        (uint8_t)(high_word & 0xFFU),
        0x00U,
        0x00U,
    };
    uint8_t response[ISL60PR_WRITE_RESPONSE_LENGTH];

    isl60pr_last_function = ISL60PR_FUNCTION_WRITE_MULTIPLE;
    isl60pr_last_register_address = register_address;
    isl60pr_last_write_value = value;

    ISL60PR_AppendCrc(request, ISL60PR_WRITE32_REQUEST_LENGTH - 2U);
    BspIsl60prResult result = ISL60PR_Transaction(request,
                                                  ISL60PR_WRITE32_REQUEST_LENGTH,
                                                  response,
                                                  ISL60PR_WRITE_RESPONSE_LENGTH);
    if (result != BSP_ISL60PR_OK)
    {
        return result;
    }
    if (memcmp(&request[2], &response[2], 4U) != 0)
    {
        return BSP_ISL60PR_BAD_RESPONSE;
    }

    return BSP_ISL60PR_OK;
}

BspIsl60prResult BSP_ISL60PR_ConfigurePosition(uint32_t speed_pulses_per_second,
                                              uint32_t acceleration_pulses_per_second2,
                                              uint32_t deceleration_pulses_per_second2,
                                              uint16_t arrival_window_pulses)
{
    BspIsl60prResult result;

    if (speed_pulses_per_second == 0U || speed_pulses_per_second > 6000000U ||
        acceleration_pulses_per_second2 == 0U ||
        deceleration_pulses_per_second2 == 0U ||
        arrival_window_pulses == 0U)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }

    result = ISL60PR_WriteIfDifferent(ISL60PR_REG_EEPROM_WRITE_ENABLE, 0U);
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_CONTROL_MODE,
                                          ISL60PR_CONTROL_MODE_MODBUS);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DI_VALUE,
                                          ISL60PR_ALL_DI_INACTIVE);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DIDO_ENABLE,
                                          ISL60PR_FORCE_DI_ONLY);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_Stop();
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_WORD_ORDER,
                                          ISL60PR_LOW_WORD_FIRST);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_OPERATION_MODE,
                                          ISL60PR_OPERATION_MODE_PP);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_POSITION_ARRIVAL_WINDOW,
                                            arrival_window_pulses);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PP_SPEED,
                                            speed_pulses_per_second);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_ACCELERATION,
                                            acceleration_pulses_per_second2);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_DECELERATION,
                                            deceleration_pulses_per_second2);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_SetServoEnabled(false);
    }

    return result;
}

BspIsl60prResult BSP_ISL60PR_SetServoEnabled(bool enabled)
{
    return BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DI_VALUE,
                                     enabled ? ISL60PR_DI3_SERVO_ENABLED
                                             : ISL60PR_ALL_DI_INACTIVE);
}

BspIsl60prResult BSP_ISL60PR_MoveRelative(int32_t position_delta)
{
    BspIsl60prResult result =
        BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PP_TARGET_POSITION,
                                    (uint32_t)position_delta);

    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_PulseControlWord(ISL60PR_CONTROL_RELATIVE_POSITION);
    }

    return result;
}

BspIsl60prResult BSP_ISL60PR_MoveAbsolute(int32_t position)
{
    if (position < -1073741824L || position > 1073741824L)
        return BSP_ISL60PR_INVALID_ARGUMENT;
    BspIsl60prResult result = BSP_ISL60PR_WriteRegister32(
        ISL60PR_REG_PP_TARGET_POSITION, (uint32_t)position);
    if (result == BSP_ISL60PR_OK)
        result = ISL60PR_PulseControlWord(3U);
    return result;
}

BspIsl60prResult BSP_ISL60PR_Stop(void)
{
    return ISL60PR_PulseControlWord(ISL60PR_CONTROL_STOP);
}

BspIsl60prResult BSP_ISL60PR_EmergencyStop(void)
{
    return ISL60PR_PulseControlWord(ISL60PR_CONTROL_EMERGENCY_STOP);
}

BspIsl60prResult BSP_ISL60PR_ConfigureJogRs485(
    uint32_t speed_pulses_per_second,
    uint32_t acceleration_pulses_per_second2,
    uint32_t deceleration_pulses_per_second2)
{
    BspIsl60prResult result;

    if (speed_pulses_per_second > 6000000U ||
        acceleration_pulses_per_second2 == 0U ||
        deceleration_pulses_per_second2 == 0U)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }

    result = ISL60PR_WriteIfDifferent(ISL60PR_REG_EEPROM_WRITE_ENABLE, 0U);
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_CONTROL_MODE,
                                          ISL60PR_CONTROL_MODE_MODBUS);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DI_VALUE,
                                          ISL60PR_ALL_DI_INACTIVE);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DIDO_ENABLE,
                                          ISL60PR_FORCE_DI_ONLY);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_Stop();
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_WORD_ORDER,
                                          ISL60PR_LOW_WORD_FIRST);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_OPERATION_MODE,
                                          ISL60PR_OPERATION_MODE_PV);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_ACCELERATION,
                                            acceleration_pulses_per_second2);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_DECELERATION,
                                            deceleration_pulses_per_second2);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_SPEED,
                                            speed_pulses_per_second);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_SetServoEnabled(false);
    }

    return result;
}

BspIsl60prResult BSP_ISL60PR_SetJogSpeed(uint32_t speed_pulses_per_second)
{
    if (speed_pulses_per_second > 6000000U)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }

    return BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_SPEED,
                                       speed_pulses_per_second);
}

BspIsl60prResult BSP_ISL60PR_ConfigureJog(uint32_t speed_pulses_per_second,
                                         uint32_t acceleration_pulses_per_second2,
                                         uint32_t deceleration_pulses_per_second2,
                                         bool configure_brake_output)
{
    BspIsl60prResult result;

    if (speed_pulses_per_second == 0U || speed_pulses_per_second > 6000000U ||
        acceleration_pulses_per_second2 == 0U ||
        deceleration_pulses_per_second2 == 0U)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }

    /* Stop first, then temporarily force every DI inactive for stop-only writes. */
    result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_CONTROL,
                                      BSP_ISL60PR_JOG_STOP);
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DI_VALUE,
                                          ISL60PR_ALL_DI_INACTIVE);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DIDO_ENABLE,
                                          ISL60PR_FORCE_DI_ONLY);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_EEPROM_WRITE_ENABLE, 0U);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_CONTROL_MODE,
                                          ISL60PR_CONTROL_MODE_MODBUS);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_WORD_ORDER,
                                          ISL60PR_LOW_WORD_FIRST);
    }
    if (result == BSP_ISL60PR_OK && configure_brake_output)
    {
        result = ISL60PR_WriteIfDifferent(ISL60PR_REG_DO1_FUNCTION,
                                          ISL60PR_DO_FUNCTION_BRAKE);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_OPERATION_MODE,
                                          ISL60PR_OPERATION_MODE_PV);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_ACCELERATION,
                                            acceleration_pulses_per_second2);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_DECELERATION,
                                            deceleration_pulses_per_second2);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister32(ISL60PR_REG_PV_SPEED,
                                            speed_pulses_per_second);
    }
    if (result == BSP_ISL60PR_OK)
    {
        result = BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_CONTROL,
                                          BSP_ISL60PR_JOG_STOP);
    }

    /* Restore physical DI3 enable and DI4/DI5 travel-limit inputs before motion. */
    BspIsl60prResult restore_result =
        BSP_ISL60PR_WriteRegister(ISL60PR_REG_FORCE_DIDO_ENABLE,
                                  ISL60PR_FORCE_DIDO_DISABLED);
    if (result == BSP_ISL60PR_OK)
    {
        result = restore_result;
    }

    return result;
}

BspIsl60prResult BSP_ISL60PR_Jog(BspIsl60prJogCommand command)
{
    if (command != BSP_ISL60PR_JOG_STOP &&
        command != BSP_ISL60PR_JOG_POSITIVE &&
        command != BSP_ISL60PR_JOG_NEGATIVE)
    {
        return BSP_ISL60PR_INVALID_ARGUMENT;
    }

    return BSP_ISL60PR_WriteRegister(ISL60PR_REG_BUS_CONTROL, (uint16_t)command);
}

uint8_t BSP_ISL60PR_GetLastException(void)
{
    return isl60pr_last_exception;
}

uint8_t BSP_ISL60PR_GetLastFunction(void)
{
    return isl60pr_last_function;
}

uint16_t BSP_ISL60PR_GetLastRegisterAddress(void)
{
    return isl60pr_last_register_address;
}

uint32_t BSP_ISL60PR_GetLastWriteValue(void)
{
    return isl60pr_last_write_value;
}
