#include "lift_uart_task.h"

#include <limits.h>
#include <string.h>

#include "bsp_uart.h"
#include "lift_positions.h"
#include "lift_uart_protocol.h"
#include "servo_app_config.h"
#include "servo_jog_task.h"

#define CACHE_CAPACITY 16U
#define TELEMETRY_PERIOD_MS 100U

typedef struct
{
    bool used;
    bool waiting;
    uint8_t request[8];
    uint8_t ack[8];
    uint32_t updated_ms;
} RequestCache;

typedef struct
{
    uint8_t frame[LIFT_UART_MAX_FRAME_SIZE];
    uint8_t used;
    uint8_t expected;
} StreamParser;

static struct
{
    bool session;
    bool tx_error;
    int32_t nonce;
    uint32_t heartbeat_ms;
    uint32_t last_telemetry_ms;
    uint32_t parser_errors;
    uint32_t observed_rx_overflows;
    uint16_t sample_sequence;
    RequestCache cache[CACHE_CAPACITY];
    StreamParser parser;
} uart;

static uint32_t NowMs(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void Snapshot(ServoJogDiagnostics *snapshot)
{
    taskENTER_CRITICAL();
    memcpy(snapshot, (const void *)&g_servo_jog_diagnostics, sizeof(*snapshot));
    taskEXIT_CRITICAL();
}

static uint8_t Flags(const ServoJogDiagnostics *snapshot)
{
    return (snapshot->position_valid ? 1U : 0U) |
           (snapshot->target_height_valid ? 2U : 0U) |
           (snapshot->brake_released ? 4U : 0U) |
           ((snapshot->commanded_speed || snapshot->route_active ||
             snapshot->speed_rpm > SERVO_STOP_FEEDBACK_TOLERANCE_RPM ||
             snapshot->speed_rpm < -SERVO_STOP_FEEDBACK_TOLERANCE_RPM) ? 8U : 0U) |
           16U |
           ((snapshot->profile_flags & LIFT_PROFILE_BOOT_HOME_ARMED) ? 32U : 0U) |
           (uart.session ? 64U : 0U) |
           (snapshot->motion_inhibited ? 128U : 0U);
}

static uint8_t CurrentFlags(void)
{
    ServoJogDiagnostics snapshot;
    Snapshot(&snapshot);
    return Flags(&snapshot);
}

static bool SendFrame(uint8_t type, const uint8_t *data, uint8_t length)
{
    uint8_t frame[LIFT_UART_MAX_FRAME_SIZE];
    size_t frame_length = LiftUartProtocol_EncodeFrame(type, data, length,
                                                       frame, sizeof(frame));
    if (frame_length == 0U ||
        BSP_UART_DebugWrite(frame, (uint16_t)frame_length) != BSP_UART_OK)
    {
        uart.tx_error = true;
        return false;
    }
    return true;
}

static void SendAck(uint8_t opcode, uint8_t sequence, uint8_t result, int32_t value)
{
    uint8_t ack[8];
    LiftUartProtocol_EncodeAck(opcode, sequence, result, CurrentFlags(), value, ack);
    (void)SendFrame(LIFT_UART_FRAME_ACK, ack, sizeof(ack));
}

static void CacheReply(RequestCache *entry, uint8_t result, int32_t value, uint32_t now)
{
    LiftUartProtocol_EncodeAck(entry->request[0], entry->request[1], result,
                               CurrentFlags(), value, entry->ack);
    entry->updated_ms = now;
    (void)SendFrame(LIFT_UART_FRAME_ACK, entry->ack, sizeof(entry->ack));
}

static void LoseSession(uint32_t now)
{
    if (uart.session || ServoJog_RemoteActive()) ServoJog_CancelRemote();
    uart.session = false;
    for (unsigned i = 0; i < CACHE_CAPACITY; ++i)
    {
        RequestCache *entry = &uart.cache[i];
        if (entry->used && entry->waiting)
        {
            LiftUartRequest request;
            (void)LiftUartProtocol_DecodeRequest(entry->request, &request);
            entry->waiting = false;
            CacheReply(entry, LIFT_UART_LINK_LOST, request.value, now);
        }
    }
}

static RequestCache *FindCache(uint8_t sequence, uint32_t now)
{
    for (unsigned i = 0; i < CACHE_CAPACITY; ++i)
    {
        RequestCache *entry = &uart.cache[i];
        if (entry->used && !entry->waiting &&
            now - entry->updated_ms >= LIFT_UART_SESSION_MS) entry->used = false;
        if (entry->used && entry->request[1] == sequence) return entry;
    }
    return NULL;
}

static RequestCache *ReserveCache(const uint8_t request[8], uint32_t now)
{
    RequestCache *oldest = NULL;
    for (unsigned i = 0; i < CACHE_CAPACITY; ++i)
    {
        RequestCache *entry = &uart.cache[i];
        if (!entry->used) { oldest = entry; break; }
        if (!entry->waiting && now - entry->updated_ms >= LIFT_UART_SESSION_MS &&
            (oldest == NULL || now - entry->updated_ms > now - oldest->updated_ms))
            oldest = entry;
    }
    if (oldest == NULL) return NULL;
    memset(oldest, 0, sizeof(*oldest));
    oldest->used = true;
    oldest->updated_ms = now;
    memcpy(oldest->request, request, 8);
    return oldest;
}

static bool MapCommand(const LiftUartRequest *request, LiftCommand *command, int32_t *value)
{
    *value = request->value;
    switch (request->opcode)
    {
    case LIFT_UART_MOVE_UM: *command = LIFT_CMD_HEIGHT_UM; break;
    case LIFT_UART_HOME: *command = LIFT_CMD_HOME; break;
    case LIFT_UART_SET_CENTER: *command = LIFT_CMD_SET; *value = 3; break;
    case LIFT_UART_ABS_CHECK: *command = LIFT_CMD_ABS_CHECK; break;
    case LIFT_UART_ABS_READ: *command = LIFT_CMD_ABS_READ; break;
    case LIFT_UART_GOTO_POINT: *command = LIFT_CMD_GOTO; *value = request->argument; break;
    case LIFT_UART_RESTORE_REF: *command = LIFT_CMD_RECOVER; break;
    case LIFT_UART_SPEED: *command = LIFT_CMD_SPEED; break;
    case LIFT_UART_BOOT_HOME: *command = LIFT_CMD_BOOT_HOME; break;
    default: return false;
    }
    return true;
}

static int32_t SignedBits(uint32_t bits)
{
    return bits <= INT32_MAX ? (int32_t)bits : -1 - (int32_t)(UINT32_MAX - bits);
}

static int32_t InfoValue(const ServoJogDiagnostics *snapshot, int32_t page)
{
    if (page >= 10 && page <= 14)
        return (int32_t)((int64_t)snapshot->saved_points[page - 10] * 1000 /
                         LIFT_UNITS_PER_MM);
    switch (page)
    {
    case 0: return LIFT_UART_PROTOCOL_VERSION;
    case 1: return (int32_t)snapshot->version;
    case 2: return snapshot->operating_min_um;
    case 3: return snapshot->operating_max_um;
    case 4: return (int32_t)snapshot->absolute_stage;
    case 5: return (int32_t)snapshot->profile_flags;
    case 6: return (int32_t)snapshot->profile_cruise_rpm;
    case 7: return snapshot->lower_edge_um;
    case 8: return snapshot->upper_edge_um;
    case 9: return (int32_t)snapshot->saved_mask;
    case 15: return (int32_t)snapshot->absolute_read_valid;
    case 16: return (int32_t)snapshot->motor_code;
    case 17: return (int32_t)snapshot->command_units_per_rev;
    case 18: return SignedBits(snapshot->absolute_low);
    case 19: return SignedBits(snapshot->absolute_high);
    case 20: return SignedBits(snapshot->absolute_system);
    case 21: return SignedBits(snapshot->absolute_turns);
    case 22: return SignedBits(snapshot->absolute_single);
    case 23: return SignedBits(snapshot->mechanical_low);
    case 24: return SignedBits(snapshot->mechanical_high);
    case 25: return (int32_t)snapshot->ready_for_command;
    case 26: return (int32_t)snapshot->reference_pending;
    case 27: return (int32_t)snapshot->state;
    case 28: return (int32_t)snapshot->communication_errors;
    case 29: return (int32_t)snapshot->guard_reason;
    case 30: return (int32_t)snapshot->reset_pending;
    default: return 0;
    }
}

static void HandleRequest(const uint8_t payload[8], uint32_t now)
{
    LiftUartRequest request;
    uint8_t result = LiftUartProtocol_DecodeRequest(payload, &request);
    if (result != LIFT_UART_OK)
    {
        SendAck(payload[0], payload[1], result, 0);
        return;
    }
    if (request.opcode == LIFT_UART_STOP)
    {
        ServoJog_RequestStop();
        SendAck(request.opcode, request.sequence, LIFT_UART_OK, 0);
        return;
    }
    if (request.opcode == LIFT_UART_HELLO)
    {
        if ((uart.session && request.value != uart.nonce) ||
            (!uart.session && ServoJog_RemoteActive()))
        {
            SendAck(request.opcode, request.sequence, LIFT_UART_BUSY, request.value);
            return;
        }
        if (!uart.session)
        {
            if (request.value == uart.nonce)
            {
                SendAck(request.opcode, request.sequence, LIFT_UART_NO_SESSION, request.value);
                return;
            }
            memset(uart.cache, 0, sizeof(uart.cache));
            uart.nonce = request.value;
            uart.session = true;
        }
        uart.heartbeat_ms = now;
        SendAck(request.opcode, request.sequence, LIFT_UART_OK, request.value);
        return;
    }
    if (request.opcode != LIFT_UART_INFO && !uart.session)
    {
        SendAck(request.opcode, request.sequence, LIFT_UART_NO_SESSION, request.value);
        return;
    }

    RequestCache *entry = uart.session ? FindCache(request.sequence, now) : NULL;
    if (entry != NULL)
    {
        if (memcmp(entry->request, payload, 8) != 0)
            SendAck(request.opcode, request.sequence, LIFT_UART_SEQ_CONFLICT, request.value);
        else
            (void)SendFrame(LIFT_UART_FRAME_ACK, entry->ack, sizeof(entry->ack));
        return;
    }
    if ((request.opcode == LIFT_UART_KEEPALIVE || request.opcode == LIFT_UART_CLOSE) &&
        request.value != uart.nonce)
    {
        SendAck(request.opcode, request.sequence, LIFT_UART_NO_SESSION, request.value);
        return;
    }

    if (request.opcode == LIFT_UART_INFO)
    {
        ServoJogDiagnostics snapshot;
        Snapshot(&snapshot);
        SendAck(request.opcode, request.sequence, LIFT_UART_OK,
                InfoValue(&snapshot, request.value));
        return;
    }

    entry = ReserveCache(payload, now);
    if (entry == NULL)
    {
        SendAck(request.opcode, request.sequence, LIFT_UART_BUSY, request.value);
        return;
    }
    if (request.opcode == LIFT_UART_KEEPALIVE)
    {
        uart.heartbeat_ms = now;
        CacheReply(entry, LIFT_UART_OK, request.value, now);
        return;
    }
    if (request.opcode == LIFT_UART_CLOSE)
    {
        uart.session = false;
        ServoJog_CancelRemote();
        CacheReply(entry, LIFT_UART_OK, request.value, now);
        return;
    }

    if (request.opcode == LIFT_UART_RESET)
    {
        CacheReply(entry, ServoJog_RequestReset() ? LIFT_UART_OK : LIFT_UART_BUSY,
                   request.value, now);
        return;
    }

    LiftCommand command;
    int32_t value;
    if (!MapCommand(&request, &command, &value))
        CacheReply(entry, LIFT_UART_BAD_OPCODE, request.value, now);
    else if (!ServoJog_QueueRemoteCommand(command, value,
             (uint16_t)((uint16_t)request.opcode << 8 | request.sequence)))
        CacheReply(entry, LIFT_UART_BUSY, request.value, now);
    else
    {
        entry->waiting = true;
        CacheReply(entry, LIFT_UART_QUEUED, request.value, now);
    }
}

static void ReceiveApplicationReplies(uint32_t now)
{
    uint16_t token;
    uint32_t result;
    while (ServoJog_TakeRemoteReply(&token, &result))
    {
        for (unsigned i = 0; i < CACHE_CAPACITY; ++i)
        {
            RequestCache *entry = &uart.cache[i];
            if (entry->used && entry->waiting &&
                token == (uint16_t)((uint16_t)entry->request[0] << 8 | entry->request[1]))
            {
                LiftUartRequest request;
                (void)LiftUartProtocol_DecodeRequest(entry->request, &request);
                entry->waiting = false;
                CacheReply(entry, result <= 11U ? (uint8_t)result : LIFT_UART_BAD_VALUE,
                           request.value, now);
                break;
            }
        }
    }
}

static void FeedByte(uint8_t byte, uint32_t now)
{
    StreamParser *parser = &uart.parser;
    if (parser->used == 0U)
    {
        if (byte == LIFT_UART_SOF_0) parser->frame[parser->used++] = byte;
        return;
    }
    if (parser->used == 1U)
    {
        if (byte == LIFT_UART_SOF_1) parser->frame[parser->used++] = byte;
        else if (byte != LIFT_UART_SOF_0) parser->used = 0U;
        return;
    }
    parser->frame[parser->used++] = byte;
    if (parser->used == 3U)
    {
        if (byte < 1U || byte > LIFT_UART_MAX_DATA_SIZE + 1U)
        {
            parser->used = 0U;
            uart.parser_errors++;
            return;
        }
        parser->expected = (uint8_t)(byte + 5U);
    }
    if (parser->expected == 0U || parser->used < parser->expected) return;

    uint8_t type;
    uint8_t data_length;
    const uint8_t *data;
    if (LiftUartProtocol_DecodeFrame(parser->frame, parser->used,
                                     &type, &data, &data_length) &&
        type == LIFT_UART_FRAME_REQUEST && data_length == LIFT_UART_REQUEST_DATA_SIZE)
        HandleRequest(data, now);
    else
        uart.parser_errors++;
    parser->used = 0U;
    parser->expected = 0U;
}

static void SendTelemetry(uint32_t now)
{
    if (now - uart.last_telemetry_ms < TELEMETRY_PERIOD_MS) return;
    uart.last_telemetry_ms = now;
    ServoJogDiagnostics snapshot;
    Snapshot(&snapshot);
    int32_t speed = snapshot.speed_rpm;
    if (speed > INT16_MAX) speed = INT16_MAX;
    if (speed < INT16_MIN) speed = INT16_MIN;
    uint32_t age = ((uint32_t)xTaskGetTickCount() - snapshot.position_sample_tick) *
                   portTICK_PERIOD_MS;
    uint32_t overflow = BSP_UART_DebugRxOverflowCount();
    if (overflow != uart.observed_rx_overflows) uart.observed_rx_overflows = overflow;
    LiftUartTelemetry sample = {
        .sequence = ++uart.sample_sequence,
        .current_um = snapshot.height_um,
        .target_um = snapshot.target_height_um,
        .speed_rpm = (int16_t)speed,
        .fault = (uint16_t)snapshot.fault_code,
        .target_status = (uint8_t)snapshot.target_status,
        .flags = Flags(&snapshot),
        .raw_mask = (uint8_t)snapshot.position_sensor_raw_mask,
        .stable_mask = (uint8_t)snapshot.position_sensor_mask,
        .result = (uint8_t)snapshot.command_result,
        .guard = (snapshot.lower_limit_latched ? 1U : 0U) |
                 (snapshot.upper_limit_latched ? 2U : 0U) |
                 (uart.observed_rx_overflows ? 4U : 0U) |
                 (uart.parser_errors ? 8U : 0U) |
                 (uart.tx_error ? 16U : 0U),
        .uptime_ms = now,
        .feedback_age_ms = age > UINT16_MAX ? UINT16_MAX : (uint16_t)age
    };
    for (unsigned page = 0; page < 4U; ++page)
    {
        uint8_t encoded[9];
        encoded[0] = (uint8_t)page;
        (void)LiftUartProtocol_EncodeTelemetry(&sample, page, encoded + 1);
        (void)SendFrame(LIFT_UART_FRAME_TELEMETRY, encoded, sizeof(encoded));
    }
}

static void LiftUart_Task(void *argument)
{
    (void)argument;
    memset(&uart, 0, sizeof(uart));
    BSP_UART_DebugInit();
    for (;;)
    {
        uint32_t now = NowMs();
        uint8_t byte;
        while (BSP_UART_DebugTryRead(&byte)) FeedByte(byte, now);
        ReceiveApplicationReplies(now);
        if (uart.session && now - uart.heartbeat_ms > LIFT_UART_SESSION_MS)
            LoseSession(now);
        SendTelemetry(now);
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

void LiftUart_TaskCreate(void)
{
    (void)xTaskCreate(LiftUart_Task, LIFT_UART_TASK_NAME,
                      LIFT_UART_TASK_STACK_WORDS, NULL,
                      LIFT_UART_TASK_PRIORITY, NULL);
}
