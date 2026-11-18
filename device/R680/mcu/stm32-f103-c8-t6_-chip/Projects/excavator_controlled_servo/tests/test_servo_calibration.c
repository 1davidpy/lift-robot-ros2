#include <assert.h>
#include <string.h>

/* Exercise the real parser and motion state machine with a simulated bus. */
#include "../App/Tasks/servo_control_task.c"

static char reply[256];
static const char *serial_input;
static size_t serial_index;
static uint16_t feedback[9];
static unsigned position_writes;
static unsigned sync_writes;
static unsigned speed_writes;
static unsigned queue_writes;
static BspSts3215PositionCommand last_position;
static bool fail_read;
static bool fail_offset;
static TickType_t ticks;
static uint16_t goals[9];
static uint16_t position_goals[9];
static int16_t pending_feedback_shift[9];
static TickType_t feedback_ready_at[9];
static uint8_t modes[9];
static uint8_t torque[9];
static unsigned mode_writes;
static ExcavatorVelocityCmd_t queued;

TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t delay) { ticks += delay; }
BaseType_t xTaskCreate(void (*entry)(void *), const char *name, unsigned stack,
                      void *argument, unsigned priority, void *handle)
{
    (void)entry; (void)name; (void)stack; (void)argument; (void)priority; (void)handle;
    return pdPASS;
}
QueueHandle_t xQueueCreate(unsigned count, unsigned size)
{
    (void)count; (void)size;
    return (void *)1;
}
static bool queue_ready;
BaseType_t xQueueReceive(QueueHandle_t queue, void *value, TickType_t wait)
{
    (void)queue; (void)wait;
    if (queue_ready) { *(ExcavatorVelocityCmd_t *)value = queued; queue_ready = false; return pdPASS; }
    return 0;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *value)
{
    (void)queue;
    queued = *(const ExcavatorVelocityCmd_t *)value;
    queue_writes++;
    return pdPASS;
}
void BSP_LED_On(BspLed led) { (void)led; }
void BSP_UART_DebugInit(void) {}
bool BSP_UART_DebugTryRead(uint8_t *data)
{
    if (serial_input != NULL && serial_input[serial_index] != '\0')
    { *data = (uint8_t)serial_input[serial_index++]; return true; }
    return false;
}
BspUartResult BSP_UART_DebugWrite(const uint8_t *data, uint16_t length)
{
    assert(length < sizeof(reply));
    memcpy(reply, data, length);
    reply[length] = '\0';
    return BSP_UART_OK;
}
BspSts3215Result BSP_STS3215_Init(void) { return BSP_STS3215_OK; }
void BSP_STS3215_GetDiagnostics(BspSts3215Diagnostics *value) { memset(value, 0, sizeof(*value)); }
BspSts3215Result BSP_STS3215_ReadPresentPosition(uint8_t id, uint16_t *position)
{
    if (fail_read) { return BSP_STS3215_TIMEOUT; }
    if (pending_feedback_shift[id] != 0 && ticks >= feedback_ready_at[id])
    {
        feedback[id] = (uint16_t)((int32_t)feedback[id] + pending_feedback_shift[id]);
        pending_feedback_shift[id] = 0;
    }
    *position = feedback[id];
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_ReadOperatingMode(uint8_t id, uint8_t *mode)
{
    *mode = modes[id];
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_SetOperatingMode(uint8_t id, uint8_t mode)
{
    assert(torque[id] == 0 && ((id >= 4 && id <= 6 && mode == 1) ||
           ((id <= 3 || id == 8) && mode == 0)));
    if (EXCAVATOR_POSITION_REMOTE_MODE && id <= 3 && modes[id] == 1 && mode == 0 && pending_feedback_shift[id] == 0)
        pending_feedback_shift[id] = -85;
    modes[id] = mode;
    if (pending_feedback_shift[id] != 0) feedback_ready_at[id] = ticks + 50;
    mode_writes++;
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_ReadTorqueEnabled(uint8_t id, uint8_t *enabled)
{
    *enabled = torque[id];
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_SetTorqueEnabled(uint8_t id, uint8_t enabled)
{
    assert(id >= 1 && id <= 8 && id != 7 && enabled <= 1);
    if (enabled)
    {
        if (id >= 4 && id <= 6) assert(modes[id] == 1 && goals[id] == 0);
        else assert(modes[id] == 0 && position_goals[id] == feedback[id] && goals[id] <= 100);
    }
    torque[id] = enabled;
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_ReadGoalSpeed(uint8_t id, uint16_t *speed)
{
    *speed = goals[id];
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_WriteGoalSpeed(uint8_t id, uint16_t speed)
{
    assert(id >= 4U && id <= 6U && (speed & 0x7FFFU) <= EXCAVATOR_CHASSIS_TEST_SPEED_MAX);
    if (modes[id] != BSP_STS3215_MODE_WHEEL) return BSP_STS3215_PROTOCOL_ERROR;
    goals[id] = speed;
    speed_writes++;
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_SyncWritePosition(const BspSts3215PositionCommand *commands, uint8_t count)
{
    sync_writes++;
    assert(count > 0U);
    last_position = commands[count - 1U];
    position_writes++;
    return BSP_STS3215_OK;
}

BspSts3215Result BSP_STS3215_ReadGoalPosition(uint8_t id, uint16_t *position)
{
    *position = position_goals[id];
    return BSP_STS3215_OK;
}
BspSts3215Result BSP_STS3215_ReadPositionOffset(uint8_t id, uint16_t *offset)
{
    (void)id; *offset = fail_offset ? 84 : 85;
    return BSP_STS3215_OK;
}
static BspSts3215Result write_bounded_position(const BspSts3215PositionCommand *value, uint16_t window, uint16_t speed_max)
{
    if (modes[value->id] != BSP_STS3215_MODE_POSITION) return BSP_STS3215_PROTOCOL_ERROR;
    assert((value->id <= 3 || value->id == 8) && modes[value->id] == 0 && value->speed <= speed_max);
    assert((value->position > feedback[value->id] ? value->position - feedback[value->id] :
            feedback[value->id] - value->position) <= window);
    last_position = *value;
    position_goals[value->id] = value->position;
    goals[value->id] = value->speed;
    position_writes++;
    return BSP_STS3215_OK;
}

BspSts3215Result BSP_STS3215_ReadPositionDeadband(uint8_t id, uint8_t *positive, uint8_t *negative)
{
    (void)id; *positive = 16; *negative = 16;
    return BSP_STS3215_OK;
}

BspSts3215Result BSP_STS3215_ReadRegisterWord(uint8_t id, uint8_t address, uint16_t *value)
{
    (void)id; (void)address; *value = 1234;
    return BSP_STS3215_OK;
}

BspSts3215Result BSP_STS3215_WriteCalibrationPosition(const BspSts3215PositionCommand *value)
{
    return write_bounded_position(value, 10, 100);
}

BspSts3215Result BSP_STS3215_WriteRemotePosition(const BspSts3215PositionCommand *value)
{
    assert(torque[value->id] == 1);
    return write_bounded_position(value, EXCAVATOR_REMOTE_POSITION_WINDOW_MAX, EXCAVATOR_REMOTE_SPEED_MAX);
}

static void command(const char *text, const char *expected)
{
    char line[96];
    strcpy(line, text);
    reply[0] = '\0';
    ServoControl_ProcessCommand(line);
    assert(strcmp(reply, expected) == 0);
}

int main(void)
{
    for (unsigned i = 4; i <= 6; i++) { modes[i] = BSP_STS3215_MODE_WHEEL; torque[i] = 1; }
#if EXCAVATOR_READ_ONLY_DIAGNOSTICS
    ServoControl_LoadParameters();
    for (unsigned i = 1; i < 9; i++) feedback[i] = (uint16_t)(1000 + i);
    ServoControl_UpdateFeedback();
    ServoControl_UpdateMotion();
    assert(position_writes == 0U && speed_writes == 0U);
    assert(g_servo_read_positions[8] == 1008U && g_servo_read_positions[5] == 1005U);
    command("MOVE 4 2010 100", "ERR READ_ONLY\r\n");
    command("JOG 3 1 100", "ERR READ_ONLY\r\n");
    command("STOP 4", "ERR READ_ONLY\r\n");
    command("LIMIT 4 1950 2050", "ERR READ_ONLY\r\n");
    command("CLEAR 4", "ERR READ_ONLY\r\n");
    command("WHEEL 0 0", "ERR READ_ONLY\r\n");
    command("WHEEL_STOP_ALL", "ERR READ_ONLY\r\n");
    assert(position_writes == 0U && speed_writes == 0U);
    command("READINGS", "READINGS 1001 1002 1003 1004 1005 1006 1008\r\n");
    command("BUS_DIAG", "BUS_DIAG 0 0 0 0 0\r\n");
    wireless_command_queue = (void *)1;
    ExcavatorVelocityCmd_t wireless = {.boom = 10, .left_track = 6};
    assert(ServoControl_SubmitWirelessCommand(&wireless));
    ServoControl_UpdateMotion();
    assert(queue_writes == 0U && position_writes == 0U && speed_writes == 0U);
    puts("PASS: read-only firmware performs reads and rejects every motor write");
#elif EXCAVATOR_CHASSIS_TEST_MODE
    ServoControl_LoadParameters();
    for (unsigned id = 1; id < 9; id++) feedback[id] = (uint16_t)(1000 + id);
    ServoControl_UpdateFeedback();
    ServoControl_UpdateMotion();
    assert(speed_writes == 0 && position_writes == 0 && mode_writes == 0);
    assert(!position_axes[0].limits_enabled && !position_axes[1].limits_enabled);
    command("LIMIT 0 1000 1010", "OK LIMIT\r\n");
    command("MOVE 0 1005 100", "ERR AXIS_LOCKED\r\n");
    command("MOVE 4 1008 100", "ERR AXIS_LOCKED\r\n");
    command("JOG 2 1 100", "ERR AXIS_LOCKED\r\n");
    command("STOP 4", "OK STOP_LOCKED\r\n");
    command("JOG 3 1 100", "ERR RADIO_ONLY\r\n");
    command("WHEEL 0 300", "ERR RADIO_ONLY\r\n");
    assert(speed_writes == 0 && position_writes == 0);
    wireless_command_queue = (void *)1;
    ExcavatorVelocityCmd_t movement = {.left_track = 6, .right_track = -6, .swing = 10,
                                       .boom = 10, .arm = 10, .bucket = 10};
    assert(ServoControl_SubmitWirelessCommand(&movement));
    assert(queued.boom == 0 && queued.arm == 0 && queued.bucket == 0);
    ServoControl_HandleWireless(&movement);
    assert(!g_chassis_armed && speed_writes == 0);
    ExcavatorVelocityCmd_t neutral = {0};
    ticks = 1;
    ServoControl_HandleWireless(&neutral);
    ticks = 1000;
    ServoControl_HandleWireless(&neutral);
    assert(!g_chassis_armed && speed_writes == 0);
    ticks = 1001;
    ServoControl_HandleWireless(&neutral);
    assert(g_chassis_armed && speed_writes == 3 && goals[4] == 0 && goals[5] == 0 && goals[6] == 0);
    ServoControl_HandleWireless(&movement);
    assert(goals[4] == 1000 && goals[5] == (0x8000U | 3000U) && goals[6] == (0x8000U | 3000U));
    ServoControl_HandleWireless(&neutral);
    assert(goals[4] == 0 && goals[5] == 0 && goals[6] == 0);
    ServoControl_HandleWireless(&movement);
    assert(position_writes == 0 && mode_writes == 0);
    ticks = 1202;
    ServoControl_UpdateMotion();
    assert(!g_chassis_armed && goals[4] == 0 && goals[5] == 0 && goals[6] == 0);
    ServoControl_HandleWireless(&movement);
    assert(!g_chassis_armed);
    ticks = 1300;
    ServoControl_HandleWireless(&neutral);
    ticks = 2300;
    ServoControl_HandleWireless(&neutral);
    assert(g_chassis_armed);
    ServoControl_HandleWireless(&movement);
    ServoControl_NotifyWirelessTimeout();
    ServoControl_ProcessWirelessInput();
    assert(!g_chassis_armed && goals[4] == 0 && goals[5] == 0 && goals[6] == 0);
    modes[5] = BSP_STS3215_MODE_POSITION;
    ServoControl_UpdateFeedback();
    ticks = 2400;
    ServoControl_HandleWireless(&neutral);
    ticks = 3400;
    ServoControl_HandleWireless(&neutral);
    assert(!g_chassis_armed && mode_writes == 0 && position_writes == 0);
    command("CLEAR 0", "OK CLEAR\r\n");
    assert(!position_axes[0].limits_enabled);
    modes[4] = modes[5] = modes[6] = 0;
    goals[4] = 500; goals[5] = 600; goals[6] = 700;
    assert(ServoControl_PrepareChassis());
    for (unsigned id = 4; id <= 6; id++) assert(modes[id] == 1 && torque[id] == 1 && goals[id] == 0);
    assert(mode_writes == 3 && position_writes == 0 && !g_chassis_armed);
    command("CAL_PREPARE 4", "ERR UNCALIBRATED\r\n");
    command("LIMIT 4 990 1020", "OK LIMIT\r\n");
    modes[8] = 1; torque[8] = 1; position_goals[8] = 4000;
    command("CAL_PREPARE 4", "OK CAL_PREPARE\r\n");
    assert(position_goals[8] == 1008 && last_position.position == 1008 && torque[8] == 1 && modes[8] == 0);
    command("MOVE 4 1018 101", "ERR SPEED\r\n");
    command("MOVE 4 1019 100", "ERR RANGE\r\n");
    command("MOVE 4 1018 100", "OK MOVE\r\n");
    command("MOVE 4 1020 100", "ERR BUSY\r\n");
    feedback[8] = 1018;
    command("MOVE 4 1021 100", "ERR RANGE\r\n");
    command("MOVE 4 1020 100", "OK MOVE\r\n");
    command("STOP 4", "OK STOP\r\n");
    assert(position_goals[8] == 1018);
    const unsigned manual_writes = position_writes;
    ServoControl_HandleWireless(&movement);
    ServoControl_UpdateMotion();
    assert(position_writes == manual_writes);
    command("CLEAR 4", "OK CLEAR\r\n");
    command("MOVE 4 1019 100", "ERR AXIS_LOCKED\r\n");
    assert(position_writes == manual_writes);
    command("CAL_PREPARE 3", "ERR NO_LIMIT\r\n");
    for (uint8_t test_axis = 0; test_axis <= 2; test_axis++)
    {
        const uint8_t id = position_axes[test_axis].id;
        char text[64];
        modes[id] = 1; torque[id] = 1; position_goals[id] = 4000;
        (void)snprintf(text, sizeof(text), "LIMIT %u 990 1030", test_axis);
        command(text, "OK LIMIT\r\n");
        (void)snprintf(text, sizeof(text), "CAL_PREPARE %u", test_axis);
        command(text, "OK CAL_PREPARE\r\n");
        assert(position_goals[id] == feedback[id] && modes[id] == 0 && torque[id] == 1);
    }
    const unsigned prepared_writes = position_writes;
    modes[2] = 1;
    ServoControl_UpdateFeedback();
    assert((calibration_ready_mask & (1U << 1)) == 0 && torque[2] == 0);
    command("MOVE 1 1003 100", "ERR AXIS_LOCKED\r\n");
    assert(position_writes == prepared_writes);
    puts("PASS: chassis-only motion, locked position axes, speed cap, neutral interlock and loss stop");
#else
    ServoControl_LoadParameters();
    assert(position_axes[4].id == 8U);
    assert(position_axes[2].speed <= 100U && position_axes[4].speed <= 100U);
    position_axes[2].limits_enabled = false;
    position_axes[4].limits_enabled = false;
    for (unsigned i = 0; i < EXCAVATOR_POSITION_AXIS_COUNT; ++i)
    {
        position_axes[i].online = true;
        position_axes[i].operating_mode_verified = true;
        feedback[position_axes[i].id] = 2000;
        position_axes[i].current_position = 2000;
    }
    for (unsigned i = 0; i < EXCAVATOR_WHEEL_AXIS_COUNT; ++i)
    {
        wheel_axes[i].online = true;
        wheel_axes[i].wheel_mode_verified = true;
    }
    command("MOVE 4 2010 100", "ERR UNCALIBRATED\r\n");
    command("MOVE 2 2010 100", "ERR UNCALIBRATED\r\n");
    command("JOG 4 1 100", "ERR UNCALIBRATED\r\n");
    assert(position_writes == 0U);

    command("LIMIT 4 1950 2050", "OK LIMIT\r\n");
    assert(last_position.id == 8U && last_position.position == 2000U);
    command("MOVE 4 2010 100", "OK MOVE\r\n");
    ServoControl_UpdateMotion();
    assert(last_position.id == 8U && last_position.position == 2010U);
    command("MOVE 4 2051 100", "ERR RANGE\r\n");
    command("MOVE 4 2000 101", "ERR SPEED\r\n");

    /* Limit changes use fresh feedback and replace a goal already on the bus. */
    feedback[8] = 2020;
    command("LIMIT 4 2015 2030", "OK LIMIT\r\n");
    assert(last_position.position == 2020U && !position_axes[4].command_pending);
    command("LIMIT 4 1900 2000", "ERR RANGE\r\n");
    assert(position_axes[4].minimum_position == 2015U);
    fail_read = true;
    command("LIMIT 4 2000 2030", "ERR OFFLINE\r\n");
    fail_read = false;
    command("STOP 4", "OK STOP\r\n");
    ServoControl_UpdateMotion();
    assert(last_position.id == 8U && last_position.position == 2020U);

    command("LIMIT 3 0 4095", "ERR NO_LIMIT\r\n");
    command("JOG 3 1 100", "ERR CALIBRATION\r\n");
    command("WHEEL 0 100", "ERR CALIBRATION\r\n");
    assert(speed_writes == 0U);
    wireless_command_queue = (void *)1;
    ExcavatorVelocityCmd_t wireless = {.boom = 10, .left_track = 6};
    assert(ServoControl_SubmitWirelessCommand(&wireless));
    assert(queue_writes == 0U);
    position_axes[4].limits_enabled = true;
    ServoControl_SendStatus();
    assert(strncmp(reply, "STATUS2 191 19 ", 15) == 0);
    command("CLEAR 4", "OK CLEAR\r\n");
    assert(position_axes[4].limits_enabled ==
           g_excavator_position_axis_parameters[4].default_limits_enabled);
    puts("PASS: seven-servo calibration guards and ID8 motion");
#endif
    return 0;
}
