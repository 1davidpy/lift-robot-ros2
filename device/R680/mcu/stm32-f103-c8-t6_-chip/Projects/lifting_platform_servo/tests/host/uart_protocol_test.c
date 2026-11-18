/* Pure host-side wire-contract regression. Compile with Src/lift_uart_protocol.c. */
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lift_uart_protocol.h"

#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

static uint8_t frame[8];
static LiftUartRequest request;

static uint8_t Decode(uint8_t opcode, int32_t value, uint8_t argument)
{
    uint32_t bits = (uint32_t)value;
    frame[0] = opcode; frame[1] = 0xA7;
    for (unsigned i = 0; i < 4; ++i) frame[i + 2] = (uint8_t)(bits >> (8U * i));
    frame[6] = argument; frame[7] = LIFT_UART_PROTOCOL_VERSION;
    return LiftUartProtocol_DecodeRequest(frame, &request);
}

static void TestDecode(void)
{
    CHECK(Decode(LIFT_UART_MOVE_UM, -180000, 0) == LIFT_UART_OK);
    const uint8_t negative[] = { 0x10, 0xA7, 0xE0, 0x40, 0xFD, 0xFF, 0, 1 };
    CHECK(!memcmp(frame, negative, 8));
    CHECK(request.opcode == LIFT_UART_MOVE_UM && request.sequence == 0xA7 &&
          request.value == -180000 && !request.argument);
    CHECK(Decode(LIFT_UART_MOVE_UM, 180000, 0) == LIFT_UART_OK && request.value == 180000);
    CHECK(Decode(LIFT_UART_MOVE_UM, -1, 0) == LIFT_UART_OK && request.value == -1);
    CHECK(Decode(LIFT_UART_MOVE_UM, -180001, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_MOVE_UM, 180001, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_MOVE_UM, INT32_MIN, 0) == LIFT_UART_BAD_VALUE && request.value == INT32_MIN);
    CHECK(Decode(LIFT_UART_MOVE_UM, INT32_MAX, 0) == LIFT_UART_BAD_VALUE && request.value == INT32_MAX);
    CHECK(Decode(LIFT_UART_MOVE_UM, 0, 1) == LIFT_UART_BAD_VALUE);
    CHECK(LiftUartProtocol_DecodeRequest(NULL, &request) == LIFT_UART_BAD_FRAME);
    CHECK(LiftUartProtocol_DecodeRequest(frame, NULL) == LIFT_UART_BAD_FRAME);
    frame[7] = 0; CHECK(LiftUartProtocol_DecodeRequest(frame, &request) == LIFT_UART_BAD_FRAME);
    frame[7] = 2; CHECK(LiftUartProtocol_DecodeRequest(frame, &request) == LIFT_UART_BAD_FRAME);
    frame[7] = 3; CHECK(LiftUartProtocol_DecodeRequest(frame, &request) == LIFT_UART_BAD_FRAME);
    CHECK(Decode(0xFF, 0, 0) == LIFT_UART_BAD_OPCODE);

    const uint8_t zero_ops[] = { LIFT_UART_STOP, LIFT_UART_HOME, LIFT_UART_ABS_CHECK,
        LIFT_UART_ABS_READ, LIFT_UART_RESTORE_REF };
    for (unsigned i = 0; i < sizeof(zero_ops); ++i)
    {
        CHECK(Decode(zero_ops[i], 0, 0) == LIFT_UART_OK);
        CHECK(Decode(zero_ops[i], 1, 0) == LIFT_UART_BAD_VALUE);
        CHECK(Decode(zero_ops[i], -1, 0) == LIFT_UART_BAD_VALUE);
        CHECK(Decode(zero_ops[i], 0, 1) == LIFT_UART_BAD_VALUE);
    }
    const uint8_t nonce_ops[] = { LIFT_UART_HELLO, LIFT_UART_KEEPALIVE, LIFT_UART_CLOSE };
    for (unsigned i = 0; i < sizeof(nonce_ops); ++i)
    {
        CHECK(Decode(nonce_ops[i], 0, 0) == LIFT_UART_BAD_VALUE);
        CHECK(Decode(nonce_ops[i], 1, 0) == LIFT_UART_OK);
        CHECK(Decode(nonce_ops[i], INT32_MIN, 0) == LIFT_UART_OK);
        CHECK(Decode(nonce_ops[i], -1, 0) == LIFT_UART_OK);
    }
    CHECK(Decode(LIFT_UART_GOTO_POINT, 0, 3) == LIFT_UART_OK && request.argument == 3);
    CHECK(Decode(LIFT_UART_GOTO_POINT, 0, 2) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_GOTO_POINT, 1, 3) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_SET_CENTER, LIFT_UART_SET_CENTER_KEY, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_SET_CENTER, 0, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_SPEED, 120, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_SPEED, 3000, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_SPEED, 119, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_SPEED, 3001, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_BOOT_HOME, 0, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_BOOT_HOME, 1, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_BOOT_HOME, -1, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_BOOT_HOME, 2, 0) == LIFT_UART_BAD_VALUE);
    for (int32_t page = 0; page <= 30; ++page) CHECK(Decode(LIFT_UART_INFO, page, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_INFO, -1, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_INFO, 31, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_RESET, LIFT_UART_RESET_KEY, 0) == LIFT_UART_OK);
    CHECK(Decode(LIFT_UART_RESET, 0, 0) == LIFT_UART_BAD_VALUE);
    CHECK(Decode(LIFT_UART_RESET, LIFT_UART_RESET_KEY, 1) == LIFT_UART_BAD_VALUE);
}

static void TestEnvelope(void)
{
    uint8_t packet[LIFT_UART_MAX_FRAME_SIZE];
    uint8_t request_data[8] = { LIFT_UART_MOVE_UM, 7, 0xB0, 0x3C, 0xFF, 0xFF,
                                0, LIFT_UART_PROTOCOL_VERSION };
    size_t length = LiftUartProtocol_EncodeFrame(LIFT_UART_FRAME_REQUEST,
        request_data, sizeof(request_data), packet, sizeof(packet));
    CHECK(length == 14U);
    CHECK(packet[0] == 0xA5 && packet[1] == 0x5A && packet[2] == 9 &&
          packet[3] == LIFT_UART_FRAME_REQUEST);
    uint8_t type, data_length;
    const uint8_t *data;
    CHECK(LiftUartProtocol_DecodeFrame(packet, length, &type, &data, &data_length));
    CHECK(type == LIFT_UART_FRAME_REQUEST && data_length == 8U &&
          !memcmp(data, request_data, sizeof(request_data)));
    packet[length - 1U] ^= 1U;
    CHECK(!LiftUartProtocol_DecodeFrame(packet, length, &type, &data, &data_length));
    packet[length - 1U] ^= 1U;
    CHECK(!LiftUartProtocol_DecodeFrame(packet, length - 1U, &type, &data, &data_length));
    CHECK(LiftUartProtocol_EncodeFrame(0, request_data, sizeof(request_data),
                                      packet, sizeof(packet)) == 0U);
}

static void TestEncode(void)
{
    uint8_t data[8];
    LiftUartProtocol_EncodeAck(0x10, 0xFE, 0x85, 0x52, -50000, data);
    const uint8_t ack[] = { 0x10, 0xFE, 0x85, 0x52, 0xB0, 0x3C, 0xFF, 0xFF };
    CHECK(!memcmp(data, ack, 8));
    LiftUartProtocol_EncodeAck(0, 0, 0, 0, INT32_MIN, data);
    CHECK(data[4] == 0 && data[5] == 0 && data[6] == 0 && data[7] == 0x80);
    LiftUartProtocol_EncodeAck(0, 0, 0, 0, 0, NULL);
    LiftUartTelemetry sample = { 0 };
    sample.sequence = 0xFFFE; sample.current_um = -100000; sample.target_um = 200000;
    sample.speed_rpm = -3000; sample.fault = 0x0731; sample.target_status = 0xE1;
    sample.flags = 0xB5; sample.raw_mask = 8; sample.stable_mask = 4;
    sample.result = 11; sample.guard = 0x73; sample.uptime_ms = 0xFEDCBA98U;
    sample.feedback_age_ms = 65535;
    static const uint8_t expected[4][8] = {
        { 0xFE, 0xFF, 0x60, 0x79, 0xFE, 0xFF, 0x48, 0xF4 },
        { 0xFE, 0xFF, 0x40, 0x0D, 0x03, 0x00, 0xE1, 0xB5 },
        { 0xFE, 0xFF, 0x31, 0x07, 0x08, 0x04, 0x0B, 0x73 },
        { 0xFE, 0xFF, 0x98, 0xBA, 0xDC, 0xFE, 0xFF, 0xFF }
    };
    for (unsigned page = 0; page < 4; ++page)
    {
        CHECK(LiftUartProtocol_EncodeTelemetry(&sample, page, data));
        CHECK(!memcmp(data, expected[page], 8));
    }
    CHECK(!LiftUartProtocol_EncodeTelemetry(&sample, 4, data));
    CHECK(!LiftUartProtocol_EncodeTelemetry(NULL, 0, data));
    CHECK(!LiftUartProtocol_EncodeTelemetry(&sample, 0, NULL));
    sample.sequence = 0;
    CHECK(LiftUartProtocol_EncodeTelemetry(&sample, 0, data) && !data[0] && !data[1]);
}

int main(void)
{
    TestDecode(); TestEncode(); TestEnvelope();
    puts("PASS UART wire contract: exact frame validation, signed ranges, opcode values, ACK and four telemetry pages");
    return 0;
}

