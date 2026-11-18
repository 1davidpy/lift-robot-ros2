#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "bsp_pin.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "bsp_sts3215.h"

USART_TypeDef test_uart;
static uint8_t request[160];
static unsigned request_size, frames;
static bool silent;
static unsigned irq_priority;
static uint16_t goals[9];
static uint16_t position_goals[9];
static uint8_t modes[9];
static uint8_t torque[9];
static unsigned writes;
static bool ignore_write;

static void incoming(uint8_t byte)
{
    test_uart.SR = USART_FLAG_RXNE;
    test_uart.DR = byte;
    TestServoUart_IRQHandler();
}
int USART_GetFlagStatus(USART_TypeDef *uart, uint16_t flag)
{
    (void)uart;
    if (flag == USART_FLAG_TC && request_size > 0)
    {
        frames++;
        const bool is_write = request[4] == 0x03;
#ifdef TEST_CHASSIS_WRITE_MODE
        if (is_write)
        {
            const unsigned id = request[2];
#ifdef TEST_CALIBRATION_WRITE_MODE
            assert(id >= 1 && id <= 8 && id != 7);
#else
            assert(id >= 4 && id <= 6);
#endif
            writes++;
            if (request[5] == 46)
            {
                const uint16_t goal = (uint16_t)request[6] | ((uint16_t)request[7] << 8U);
                assert(id >= 4 && id <= 6 && modes[id] == 1 && (goal & 0x7FFFU) <= 3000U);
                if (!ignore_write) goals[id] = goal;
            }
            else if (request[5] == 33)
            {
                assert(torque[id] == 0 && request[6] == ((id <= 3 || id == 8) ? 0 : 1));
                modes[id] = request[6];
            }
            else if (request[5] == 41)
            {
                assert((id <= 3 || id == 8) && modes[id] == 0 && request_size == 14);
                const uint16_t target = (uint16_t)request[7] | ((uint16_t)request[8] << 8U);
                const uint16_t speed = (uint16_t)request[11] | ((uint16_t)request[12] << 8U);
                assert(target >= 1138 && target <= 1330 && speed > 0 && speed <= 500);
                if (!ignore_write) { position_goals[id] = target; goals[id] = speed; }
            }
            else
            {
                assert(request[5] == 40 && request[6] <= 1);
                if (request[6])
                {
                    if (id <= 3 || id == 8) assert(modes[id] == 0 && position_goals[id] == 1234 && goals[id] <= 100);
                    else assert(modes[id] == 1 && goals[id] == 0);
                }
                torque[id] = request[6];
            }
        }
#else
        assert(!is_write); // Every emitted frame must be a READ.
#endif
        if (!silent)
        {
            // Receive the entire burst before the task starts reading it.
            incoming(0x12);
            for (unsigned i = 0; i < request_size; i++) incoming(request[i]);
            const uint8_t data_size = is_write ? 0U : request[6];
            uint8_t reply[8] = {0xFF, 0xFF, request[2], (uint8_t)(data_size + 2), 0, 0xD2, 0x04, 0};
            const unsigned size = data_size + 6;
            uint8_t checksum = 0;
            if (!is_write && request[5] == 33) reply[5] = modes[request[2]];
            if (!is_write && request[5] == 40) reply[5] = torque[request[2]];
            if (!is_write && request[5] == 46)
            {
                reply[5] = (uint8_t)goals[request[2]];
                reply[6] = (uint8_t)(goals[request[2]] >> 8U);
            }
            if (!is_write && request[5] == 42)
            {
                reply[5] = (uint8_t)position_goals[request[2]];
                reply[6] = (uint8_t)(position_goals[request[2]] >> 8U);
            }
            for (unsigned i = 2; i < size - 1; i++) checksum += reply[i];
            reply[size - 1] = (uint8_t)~checksum;
            // Corrupt checksum first, then deliver a valid status frame.
            for (unsigned i = 0; i < size; i++) incoming(reply[i] ^ (i == size - 1 ? 1 : 0));
            for (unsigned i = 0; i < size; i++) incoming(reply[i]);
        }
        request_size = 0;
    }
    return SET;
}
void USART_SendData(USART_TypeDef *uart, uint16_t data)
{
    (void)uart;
    assert(request_size < sizeof(request));
    request[request_size++] = (uint8_t)data;
}
uint16_t USART_ReceiveData(USART_TypeDef *uart)
{
    uint16_t result = (uint16_t)uart->DR;
    uart->SR = 0;
    return result;
}
void USART_StructInit(USART_InitTypeDef *init) { memset(init, 0, sizeof(*init)); }
void USART_Init(USART_TypeDef *uart, USART_InitTypeDef *init) { (void)uart; (void)init; }
void USART_ITConfig(USART_TypeDef *uart, uint16_t interrupt, int enable) { (void)uart; (void)interrupt; (void)enable; }
void USART_Cmd(USART_TypeDef *uart, int enable) { (void)uart; (void)enable; }
void NVIC_SetPriority(unsigned irq, unsigned priority) { (void)irq; irq_priority = priority; }
void NVIC_EnableIRQ(unsigned irq) { (void)irq; }
void GPIO_Init(void *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
void RCC_APB2PeriphClockCmd(uint32_t clock, int enable) { (void)clock; (void)enable; }
void RCC_APB1PeriphClockCmd(uint32_t clock, int enable) { (void)clock; (void)enable; }

int main(void)
{
    for (unsigned id = 4; id <= 6; id++) { modes[id] = 1; torque[id] = 1; }
    assert(BSP_STS3215_Init() == BSP_STS3215_OK);
    assert(irq_priority == 4);
    uint16_t position = 0;
    uint8_t mode = 99;
    assert(BSP_STS3215_ReadPresentPosition(1, &position) == BSP_STS3215_OK && position == 1234);
    uint16_t encoder_offset = 0;
    assert(BSP_STS3215_ReadPositionOffset(1, &encoder_offset) == BSP_STS3215_OK && encoder_offset == 1234);
    uint8_t positive_deadband, negative_deadband;
    assert(BSP_STS3215_ReadPositionDeadband(1, &positive_deadband, &negative_deadband) == BSP_STS3215_OK);
    assert(positive_deadband == 0xD2 && negative_deadband == 0x04);
    uint16_t register_value;
    assert(BSP_STS3215_ReadRegisterWord(1, 21, &register_value) == BSP_STS3215_OK && register_value == 1234);
    assert(BSP_STS3215_ReadRegisterWord(1, 255, &register_value) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_ReadRegisterWord(1, 21, NULL) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_ReadOperatingMode(1, &mode) == BSP_STS3215_OK && mode == 0);
    const unsigned reads = frames;
    BspSts3215PositionCommand command = {.id = 1, .position = 2000, .speed = 100};
    assert(BSP_STS3215_WriteGoalSpeed(1, 0) == BSP_STS3215_INVALID_ARGUMENT);
#ifndef TEST_CALIBRATION_WRITE_MODE
    assert(BSP_STS3215_SetOperatingMode(1, 0) == BSP_STS3215_INVALID_ARGUMENT);
#endif
    assert(BSP_STS3215_SyncWritePosition(&command, 1) == BSP_STS3215_INVALID_ARGUMENT);
    assert(frames == reads && request_size == 0);
#ifndef TEST_CALIBRATION_WRITE_MODE
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
#endif
#ifdef TEST_CHASSIS_WRITE_MODE
    assert(BSP_STS3215_SetOperatingMode(4, 0) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_SetOperatingMode(4, 1) == BSP_STS3215_PROTOCOL_ERROR);
#ifndef TEST_CALIBRATION_WRITE_MODE
    assert(BSP_STS3215_SetTorqueEnabled(1, 0) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_SetTorqueEnabled(8, 1) == BSP_STS3215_INVALID_ARGUMENT);
#endif
    assert(BSP_STS3215_SetTorqueEnabled(4, 128) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_SetTorqueEnabled(4, 0) == BSP_STS3215_OK);
    assert(BSP_STS3215_WriteGoalSpeed(4, 100) == BSP_STS3215_PROTOCOL_ERROR);
    assert(BSP_STS3215_SetOperatingMode(4, 1) == BSP_STS3215_OK);
    assert(BSP_STS3215_WriteGoalSpeed(4, 0) == BSP_STS3215_OK);
    assert(BSP_STS3215_SetTorqueEnabled(4, 1) == BSP_STS3215_OK);
    command.id = 4;
    assert(BSP_STS3215_SyncWritePosition(&command, 1) == BSP_STS3215_INVALID_ARGUMENT);
    for (uint8_t id = 4; id <= 6; id++)
    {
        assert(BSP_STS3215_WriteGoalSpeed(id, 300) == BSP_STS3215_OK && goals[id] == 300);
        assert(BSP_STS3215_WriteGoalSpeed(id, 0x8000U | 300U) == BSP_STS3215_OK);
        assert(BSP_STS3215_WriteGoalSpeed(id, 0) == BSP_STS3215_OK && goals[id] == 0);
    }
    const unsigned allowed_writes = writes;
    assert(BSP_STS3215_WriteGoalSpeed(8, 0) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_WriteGoalSpeed(3, 0) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_WriteGoalSpeed(2, 0) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_WriteGoalSpeed(4, 3001) == BSP_STS3215_INVALID_ARGUMENT);
    modes[5] = 0;
    assert(BSP_STS3215_WriteGoalSpeed(5, 0) == BSP_STS3215_PROTOCOL_ERROR);
    assert(writes == allowed_writes);
    ignore_write = true;
    assert(BSP_STS3215_WriteGoalSpeed(4, 100) == BSP_STS3215_PROTOCOL_ERROR);
    ignore_write = false;
    puts("PASS: IDs4/5/6 whitelist, torque-off mode setup, zero-before-enable, speed cap, protected axes blocked");
#else
    (void)writes; (void)ignore_write;
#endif
#ifdef TEST_CALIBRATION_WRITE_MODE
    command.id = 8; command.position = 1234; command.speed = 100; command.acceleration = 10;
    modes[8] = 1; torque[8] = 1; position_goals[8] = 4000;
    assert(BSP_STS3215_SetOperatingMode(8, 0) == BSP_STS3215_PROTOCOL_ERROR);
    assert(BSP_STS3215_SetTorqueEnabled(8, 0) == BSP_STS3215_OK);
    assert(BSP_STS3215_SetOperatingMode(8, 1) == BSP_STS3215_INVALID_ARGUMENT);
    assert(BSP_STS3215_SetOperatingMode(8, 0) == BSP_STS3215_OK);
    assert(BSP_STS3215_SetTorqueEnabled(8, 1) == BSP_STS3215_PROTOCOL_ERROR);
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_OK && position_goals[8] == 1234);
    assert(BSP_STS3215_SetTorqueEnabled(8, 1) == BSP_STS3215_OK);
    command.position = 1244;
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_OK);
    const unsigned manual_writes = writes;
    command.position = 1245;
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    command.position = 1234; command.speed = 101;
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    command.speed = 100; command.id = 4;
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    assert(writes == manual_writes);
    command.id = 8; ignore_write = true;
    assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_PROTOCOL_ERROR);
    ignore_write = false;
    const unsigned before_remote = writes;
    command.position = 1330;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_OK);
    command.position = 1331;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    command.position = 1138;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_OK);
    command.position = 1137;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    command.position = 1234; command.speed = 501;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    command.speed = 100; command.id = 4;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_INVALID_ARGUMENT);
    command.id = 8; torque[8] = 0;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_PROTOCOL_ERROR);
    torque[8] = 1; modes[8] = 1;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_PROTOCOL_ERROR);
    modes[8] = 0;
    assert(writes == before_remote + 2);
    command.position = 1234; command.speed = 500;
    assert(BSP_STS3215_WriteRemotePosition(&command) == BSP_STS3215_OK && goals[8] == 500);
    command.speed = 100;
    puts("PASS: manual calibration mode/torque ordering, ID whitelist, small steps, speed cap and goal readback");
    for (uint8_t id = 1; id <= 3; id++)
    {
        modes[id] = 1; torque[id] = 1; position_goals[id] = 4000;
        command.id = id;
        assert(BSP_STS3215_SetTorqueEnabled(id, 0) == BSP_STS3215_OK);
        assert(BSP_STS3215_SetOperatingMode(id, 0) == BSP_STS3215_OK);
        assert(BSP_STS3215_WriteCalibrationPosition(&command) == BSP_STS3215_OK);
        assert(BSP_STS3215_SetTorqueEnabled(id, 1) == BSP_STS3215_OK);
    }
#else
    (void)position_goals;
#endif
    silent = true;
    assert(BSP_STS3215_ReadPresentPosition(8, &position) == BSP_STS3215_TIMEOUT);
    BspSts3215Diagnostics diagnostics;
    BSP_STS3215_GetDiagnostics(&diagnostics);
    assert(diagnostics.rx_bytes > 0 && diagnostics.overruns == 0 && diagnostics.buffer_drops == 0);
    assert(diagnostics.timeouts == 1);
    puts("PASS: buffered reply bursts and echo/noise rejection");
    return 0;
}
