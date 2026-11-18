#include "servo_control_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_led.h"
#include "bsp_sts3215.h"
#include "bsp_uart.h"
#include "excavator_servo_config.h"
#include "excavator_position_control.h"
#include "queue.h"

#define SERVO_CONTROL_LINE_BUFFER_SIZE   96U
#define SERVO_CONTROL_TOKEN_COUNT_MAX    9U
#define SERVO_CONTROL_RX_BUDGET          64U

typedef struct
{
    uint8_t id;
    uint16_t current_position;
    uint16_t target_position;
    uint16_t minimum_position;
    uint16_t maximum_position;
    uint16_t speed;
    TickType_t last_jog_command;
    int8_t jog_direction;
    bool command_pending;
    bool operating_mode_verified;
    bool limits_enabled;
    bool online;
} PositionAxisState;

typedef struct
{
    uint8_t id;
    int8_t forward_direction;
    int8_t level;
    bool wheel_mode_verified;
    bool online;
} WheelAxisState;

static PositionAxisState position_axes[EXCAVATOR_POSITION_AXIS_COUNT];
static WheelAxisState wheel_axes[EXCAVATOR_WHEEL_AXIS_COUNT];
static QueueHandle_t wireless_command_queue;
volatile uint16_t g_servo_read_positions[9];
volatile uint8_t g_servo_read_modes[9];
volatile uint8_t g_servo_read_torque[9];
volatile uint16_t g_chassis_goal_speeds[3];
volatile uint8_t g_chassis_armed;
static volatile bool wireless_timeout_pending;
static bool chassis_neutral_seen;
static TickType_t chassis_neutral_since;
static TickType_t last_chassis_command;
static ExcavatorVelocityCmd_t last_wireless_input;
static TickType_t last_wireless_input_time;
static uint32_t wireless_input_count;
/* Radio remains available during a UART session and preempts the whole host action. */
static bool host_owned;
static bool host_streaming;
static bool host_preempted;
static bool host_input_neutral;
static bool startup_attempted;
static bool startup_waiting;
static TickType_t startup_neutral_since;
static uint16_t startup_previous[EXCAVATOR_POSITION_AXIS_COUNT];
static const char *startup_state;
static uint8_t startup_error;
static bool radio_armed;
static bool radio_neutral_seen;
static TickType_t radio_neutral_since;
static TickType_t last_host_command;
static uint8_t calibration_ready_mask;
static uint8_t remote_ready_mask;
static uint8_t remote_goal_active_mask;
static bool remote_enabled;
static bool remote_armed;
static bool remote_neutral_seen;
static TickType_t remote_neutral_since;
static int8_t remote_directions[EXCAVATOR_POSITION_AXIS_COUNT];

static bool ServoControl_RemoteNeutral(const ExcavatorVelocityCmd_t *command);
static bool ServoControl_RadioActive(void);
static void ServoControl_EndHost(void);
static void ServoControl_StopChassis(void);
static void ServoControl_SuspendRemote(void);
static void ServoControl_StopRemotePositions(void);
static void ServoControl_HandleRemotePositions(const ExcavatorVelocityCmd_t *command);
static void ServoControl_UpdateRemoteReadiness(void);
static void ServoControl_UpdateRemoteMotion(void);

static bool ServoControl_IsLimitableAxis(uint8_t axis)
{
    return axis < EXCAVATOR_POSITION_AXIS_COUNT &&
           g_excavator_position_axis_parameters[axis].supports_limits;
}

static uint16_t ServoControl_GetLowerLimit(const PositionAxisState *state)
{
    return state->limits_enabled ? state->minimum_position : BSP_STS3215_POSITION_MIN;
}

static uint16_t ServoControl_GetUpperLimit(const PositionAxisState *state)
{
    return state->limits_enabled ? state->maximum_position : BSP_STS3215_POSITION_MAX;
}

static bool ServoControl_IsContinuousAxis(uint8_t axis)
{
    return axis == EXCAVATOR_AXIS_SWING;
}

static BspSts3215Result ServoControl_GetReferenceAdjustment(uint8_t axis, int16_t *adjustment)
{
    *adjustment = 0;
    if (!EXCAVATOR_POSITION_REMOTE_MODE || !ServoControl_IsLimitableAxis(axis)) return BSP_STS3215_OK;
    const ExcavatorPositionAxisParameters *parameters = &g_excavator_position_axis_parameters[axis];
    uint8_t mode;
    uint16_t offset;
    if (BSP_STS3215_ReadOperatingMode(parameters->servo_id, &mode) != BSP_STS3215_OK || mode > 1U ||
        BSP_STS3215_ReadPositionOffset(parameters->servo_id, &offset) != BSP_STS3215_OK || offset != parameters->encoder_offset)
        return BSP_STS3215_PROTOCOL_ERROR;
    *adjustment = (int16_t)(((int16_t)parameters->reference_mode - (int16_t)mode) * (int16_t)offset);
    return BSP_STS3215_OK;
}

static BspSts3215Result ServoControl_ReadAxisPosition(uint8_t axis, uint16_t *position)
{
    int16_t adjustment;
    uint16_t raw;
    if (ServoControl_GetReferenceAdjustment(axis, &adjustment) != BSP_STS3215_OK ||
        BSP_STS3215_ReadPresentPosition(position_axes[axis].id, &raw) != BSP_STS3215_OK) return BSP_STS3215_PROTOCOL_ERROR;
    int32_t reference = (int32_t)raw + adjustment;
    if (reference < 0 || reference > (int32_t)BSP_STS3215_POSITION_MAX) return BSP_STS3215_PROTOCOL_ERROR;
    *position = (uint16_t)reference;
    return BSP_STS3215_OK;
}

static BspSts3215Result ServoControl_ReadAxisGoal(uint8_t axis, uint16_t *goal)
{
    int16_t adjustment;
    uint16_t raw;
    if (ServoControl_GetReferenceAdjustment(axis, &adjustment) != BSP_STS3215_OK ||
        BSP_STS3215_ReadGoalPosition(position_axes[axis].id, &raw) != BSP_STS3215_OK) return BSP_STS3215_PROTOCOL_ERROR;
    int32_t reference = (int32_t)raw + adjustment;
    if (reference < 0 || reference > (int32_t)BSP_STS3215_POSITION_MAX) return BSP_STS3215_PROTOCOL_ERROR;
    *goal = (uint16_t)reference;
    return BSP_STS3215_OK;
}

static uint16_t ServoControl_EncodeWheelSpeed(int16_t signed_speed)
{
    uint16_t magnitude = (uint16_t)((signed_speed < 0) ? -signed_speed : signed_speed);

    return (signed_speed < 0) ? (uint16_t)(magnitude | EXCAVATOR_WHEEL_DIRECTION_BIT)
                              : magnitude;
}

static void ServoControl_LoadParameters(void)
{
    host_owned = false;
    host_streaming = false;
    host_preempted = false;
    host_input_neutral = true;
    startup_attempted = startup_waiting = false;
    startup_state = "WAIT_SERVO";
    startup_error = 0U;
    memset(startup_previous, 0, sizeof(startup_previous));
    radio_armed = false;
    radio_neutral_seen = false;
    calibration_ready_mask = 0U;
    remote_ready_mask = 0U;
    remote_goal_active_mask = 0U;
    remote_enabled = EXCAVATOR_POSITION_REMOTE_MODE && !EXCAVATOR_READ_ONLY_DIAGNOSTICS;
    remote_armed = false;
    remote_neutral_seen = false;
    memset(remote_directions, 0, sizeof(remote_directions));
    memset(&last_wireless_input, 0, sizeof(last_wireless_input));
    last_wireless_input_time = 0U;
    wireless_input_count = 0U;
    for (uint8_t i = 0U; i < EXCAVATOR_POSITION_AXIS_COUNT; i++)
    {
        position_axes[i].id = g_excavator_position_axis_parameters[i].servo_id;
        position_axes[i].minimum_position =
            g_excavator_position_axis_parameters[i].minimum_position;
        position_axes[i].maximum_position =
            g_excavator_position_axis_parameters[i].maximum_position;
        position_axes[i].speed = (i == EXCAVATOR_AXIS_PITCH || i == EXCAVATOR_AXIS_GRIPPER)
                                    ? EXCAVATOR_CALIBRATION_SPEED_MAX
                                    : EXCAVATOR_SERVO_DEFAULT_SPEED;
        position_axes[i].limits_enabled = (EXCAVATOR_POSITION_REMOTE_MODE || !EXCAVATOR_CHASSIS_TEST_MODE) &&
            g_excavator_position_axis_parameters[i].default_limits_enabled;
    }

    for (uint8_t i = 0U; i < EXCAVATOR_WHEEL_AXIS_COUNT; i++)
    {
        wheel_axes[i].id = g_excavator_wheel_axis_parameters[i].servo_id;
        wheel_axes[i].forward_direction =
            g_excavator_wheel_axis_parameters[i].forward_direction;
    }
    memset((void *)g_servo_read_modes, 0xFF, sizeof(g_servo_read_modes));
    memset((void *)g_servo_read_torque, 0xFF, sizeof(g_servo_read_torque));
}

static void ServoControl_SendText(const char *text)
{
    if (text != NULL)
    {
        (void)BSP_UART_DebugWrite((const uint8_t *)text, (uint16_t)strlen(text));
    }
}

static void ServoControl_SendStatus(void)
{
    char message[128];
    uint32_t online_mask = 0U;
    uint32_t limit_mask = 0U;
    int length;

    for (uint8_t i = 0U; i < EXCAVATOR_POSITION_AXIS_COUNT; i++)
    {
        if (position_axes[i].online)
        {
            online_mask |= (uint32_t)(1UL << (position_axes[i].id - 1U));
        }
        if (position_axes[i].limits_enabled)
        {
            limit_mask |= (uint32_t)(1UL << i);
        }
    }

    for (uint8_t i = 0U; i < EXCAVATOR_WHEEL_AXIS_COUNT; i++)
    {
        if (wheel_axes[i].online)
        {
            online_mask |= (uint32_t)(1UL << (wheel_axes[i].id - 1U));
        }
    }

    length = snprintf(message,
                      sizeof(message),
                      "STATUS2 %lu %lu %u %u %u %u %u %d %d\r\n",
                      (unsigned long)online_mask,
                      (unsigned long)limit_mask,
                      (unsigned int)position_axes[EXCAVATOR_AXIS_BOOM].current_position,
                      (unsigned int)position_axes[EXCAVATOR_AXIS_ARM].current_position,
                      (unsigned int)position_axes[EXCAVATOR_AXIS_PITCH].current_position,
                      (unsigned int)position_axes[EXCAVATOR_AXIS_SWING].current_position,
                      (unsigned int)position_axes[EXCAVATOR_AXIS_GRIPPER].current_position,
                      (int)wheel_axes[EXCAVATOR_WHEEL_LEFT].level,
                      (int)wheel_axes[EXCAVATOR_WHEEL_RIGHT].level);

    if (length > 0 && (size_t)length < sizeof(message))
    {
        (void)BSP_UART_DebugWrite((const uint8_t *)message, (uint16_t)length);
    }
    if (EXCAVATOR_MANUAL_CALIBRATION && !EXCAVATOR_READ_ONLY_DIAGNOSTICS)
    {
        (void)snprintf(message, sizeof(message), "CAL_STATE %u\r\n", calibration_ready_mask);
        ServoControl_SendText(message);
    }
    if (EXCAVATOR_POSITION_REMOTE_MODE)
    {
        uint8_t recovery = 0U;
        for (uint8_t axis = 0; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
            if (ServoControl_IsLimitableAxis(axis) && position_axes[axis].online &&
                (position_axes[axis].current_position < position_axes[axis].minimum_position ||
                 position_axes[axis].current_position > position_axes[axis].maximum_position)) recovery |= (uint8_t)(1U << axis);
        (void)snprintf(message, sizeof(message), "REMOTE_STATE %u %u %u %u\r\n",
                       remote_enabled, remote_armed, remote_ready_mask, recovery);
        ServoControl_SendText(message);
        (void)snprintf(message, sizeof(message), "CONTROL_STATE %u %u %u %u\r\n",
                       host_owned, host_streaming, radio_armed, ServoControl_RadioActive());
        ServoControl_SendText(message);
        (void)snprintf(message, sizeof(message), "STARTUP_STATE %s %u\r\n", startup_state, startup_error);
        ServoControl_SendText(message);
    }
}

static bool ServoControl_ParseUnsigned(const char *text,
                                       uint32_t maximum,
                                       uint16_t *value)
{
    char *end;
    unsigned long parsed;

    if (text == NULL || value == NULL)
    {
        return false;
    }

    parsed = strtoul(text, &end, 10);
    if (end == text || *end != '\0' || parsed > maximum)
    {
        return false;
    }

    *value = (uint16_t)parsed;
    return true;
}

static bool ServoControl_ParseSigned(const char *text,
                                     int16_t minimum,
                                     int16_t maximum,
                                     int16_t *value)
{
    char *end;
    long parsed;

    if (text == NULL || value == NULL)
    {
        return false;
    }

    parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < minimum || parsed > maximum)
    {
        return false;
    }

    *value = (int16_t)parsed;
    return true;
}

static uint8_t ServoControl_Tokenize(char *line,
                                     char *tokens[SERVO_CONTROL_TOKEN_COUNT_MAX])
{
    uint8_t count = 0U;
    char *token = strtok(line, " ");

    while (token != NULL && count < SERVO_CONTROL_TOKEN_COUNT_MAX)
    {
        tokens[count++] = token;
        token = strtok(NULL, " ");
    }

    return token != NULL ? 0U : count;
}

static bool ServoControl_ParseAxis(const char *text, uint8_t *axis)
{
    uint16_t parsed_axis;

    if (!ServoControl_ParseUnsigned(text,
                                    EXCAVATOR_POSITION_AXIS_COUNT - 1U,
                                    &parsed_axis))
    {
        return false;
    }

    *axis = (uint8_t)parsed_axis;
    return true;
}

static bool ServoControl_EnsurePositionMode(PositionAxisState *state)
{
    uint8_t mode;

    if (state->operating_mode_verified)
    {
        return true;
    }

    if (BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
        BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK)
    {
        return false;
    }

    if (mode != BSP_STS3215_MODE_POSITION)
    {
        if (BSP_STS3215_SetOperatingMode(state->id,
                                         BSP_STS3215_MODE_POSITION) != BSP_STS3215_OK ||
            BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
            BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK ||
            mode != BSP_STS3215_MODE_POSITION)
        {
            return false;
        }
    }

    state->operating_mode_verified = true;
    return true;
}

static bool ServoControl_EnsureContinuousMode(PositionAxisState *state)
{
    uint8_t mode;

    if (state->operating_mode_verified)
    {
        return true;
    }

    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        state->operating_mode_verified =
            BSP_STS3215_ReadOperatingMode(state->id, &mode) == BSP_STS3215_OK &&
            mode == BSP_STS3215_MODE_WHEEL;
        return state->operating_mode_verified;
    }

    if (BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
        BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK)
    {
        return false;
    }

    if (mode != BSP_STS3215_MODE_WHEEL)
    {
        if (BSP_STS3215_SetOperatingMode(state->id,
                                         BSP_STS3215_MODE_WHEEL) != BSP_STS3215_OK ||
            BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
            BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK ||
            mode != BSP_STS3215_MODE_WHEEL)
        {
            return false;
        }
    }

    state->operating_mode_verified = true;
    return true;
}

static bool ServoControl_EnsureWheelMode(WheelAxisState *state)
{
    uint8_t mode;

    if (state->wheel_mode_verified)
    {
        return true;
    }

    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        state->wheel_mode_verified =
            BSP_STS3215_ReadOperatingMode(state->id, &mode) == BSP_STS3215_OK &&
            mode == BSP_STS3215_MODE_WHEEL;
        return state->wheel_mode_verified;
    }

    if (BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
        BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK)
    {
        return false;
    }

    if (mode != BSP_STS3215_MODE_WHEEL)
    {
        if (BSP_STS3215_SetOperatingMode(state->id,
                                         BSP_STS3215_MODE_WHEEL) != BSP_STS3215_OK ||
            BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
            BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK ||
            mode != BSP_STS3215_MODE_WHEEL)
        {
            return false;
        }
    }

    state->level = 0;
    state->wheel_mode_verified = true;
    return true;
}

static void ServoControl_StopPositionAxis(uint8_t axis)
{
    PositionAxisState *state = &position_axes[axis];
    uint16_t position;

    if (ServoControl_IsContinuousAxis(axis))
    {
        if (state->online && ServoControl_EnsureContinuousMode(state) &&
            BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK)
        {
            state->online = false;
            state->operating_mode_verified = false;
        }
        state->jog_direction = 0;
        state->command_pending = false;
        return;
    }

    if (state->online &&
        BSP_STS3215_ReadPresentPosition(state->id, &position) == BSP_STS3215_OK)
    {
        state->current_position = position;
        state->target_position = position;
        state->command_pending = true;
    }
    else
    {
        state->command_pending = false;
    }

    state->jog_direction = 0;
}

static bool ServoControl_HoldForLimitChange(uint8_t axis)
{
    PositionAxisState *state = &position_axes[axis];
    BspSts3215PositionCommand command;
    uint16_t position;

    if (!state->online ||
        BSP_STS3215_ReadPresentPosition(state->id, &position) != BSP_STS3215_OK)
    {
        return false;
    }
    command.id = state->id;
    command.position = position;
    command.speed = EXCAVATOR_CALIBRATION_SPEED_MAX;
    command.acceleration = EXCAVATOR_SERVO_DEFAULT_ACCELERATION;
    if (BSP_STS3215_SyncWritePosition(&command, 1U) != BSP_STS3215_OK)
    {
        return false;
    }

    state->jog_direction = 0;
    state->current_position = position;
    state->target_position = position;
    state->command_pending = false;
    return true;
}

static bool ServoControl_WriteWheelSpeed(WheelAxisState *state, int16_t signed_speed)
{
    if (!state->online || !ServoControl_EnsureWheelMode(state))
    {
        return false;
    }

    if (BSP_STS3215_WriteGoalSpeed(state->id,
                                   ServoControl_EncodeWheelSpeed(signed_speed)) != BSP_STS3215_OK)
    {
        state->online = false;
        state->wheel_mode_verified = false;
        return false;
    }

    if (EXCAVATOR_CHASSIS_TEST_MODE) g_chassis_goal_speeds[state->id - 4U] = ServoControl_EncodeWheelSpeed(signed_speed);

    return true;
}

static bool ServoControl_WriteSwingSpeed(PositionAxisState *state, int16_t signed_speed)
{
    if (!state->online || !ServoControl_EnsureContinuousMode(state))
    {
        return false;
    }

    if (BSP_STS3215_WriteGoalSpeed(state->id,
                                   ServoControl_EncodeWheelSpeed(signed_speed)) != BSP_STS3215_OK)
    {
        state->online = false;
        state->operating_mode_verified = false;
        state->jog_direction = 0;
        return false;
    }

    if (EXCAVATOR_CHASSIS_TEST_MODE) g_chassis_goal_speeds[0] = ServoControl_EncodeWheelSpeed(signed_speed);

    return true;
}

static void ServoControl_SetWheelLevel(uint8_t wheel, int8_t level)
{
    WheelAxisState *state = &wheel_axes[wheel];
    uint8_t magnitude;
    uint16_t speed;
    int16_t signed_speed;

    if (state->level != 0 && level != 0 &&
        ((state->level < 0) != (level < 0)) &&
        !ServoControl_WriteWheelSpeed(state, 0))
    {
        return;
    }

    magnitude = (uint8_t)((level < 0) ? -level : level);
    speed = (uint16_t)(((uint32_t)EXCAVATOR_TRACK_SPEED_MAX * magnitude) /
                       EXCAVATOR_TRACK_LEVEL_MAX);
    if (EXCAVATOR_CHASSIS_TEST_MODE) speed = level == 0 ? 0U : EXCAVATOR_CHASSIS_TRACK_SPEED;
    signed_speed = (int16_t)speed;
    if (((level < 0) ? -1 : 1) * state->forward_direction < 0)
    {
        signed_speed = (int16_t)-signed_speed;
    }

    if (state->level == level && (!EXCAVATOR_CHASSIS_TEST_MODE ||
        g_chassis_goal_speeds[wheel + 1U] == ServoControl_EncodeWheelSpeed(signed_speed))) return;

    if (ServoControl_WriteWheelSpeed(state, signed_speed))
    {
        state->level = level;
    }
}

static void ServoControl_ApplyPositionLevel(uint8_t axis,
                                            int8_t level,
                                            TickType_t now)
{
    PositionAxisState *state = &position_axes[axis];
    const ExcavatorPositionAxisParameters *parameters =
        &g_excavator_position_axis_parameters[axis];
    int8_t direction;

    if (level == 0)
    {
        if (state->jog_direction != 0)
        {
            ServoControl_StopPositionAxis(axis);
        }
        return;
    }

    if (!state->online || !state->limits_enabled)
    {
        return;
    }

    direction = (level > 0) ? parameters->positive_direction
                            : (int8_t)-parameters->positive_direction;
    if (state->jog_direction != direction)
    {
        state->target_position = (direction < 0) ? ServoControl_GetLowerLimit(state)
                                                 : ServoControl_GetUpperLimit(state);
        state->command_pending = true;
    }

    if (state->speed != parameters->wireless_speed)
    {
        state->speed = parameters->wireless_speed;
        state->command_pending = true;
    }

    state->jog_direction = direction;
    state->last_jog_command = now;
}

static void ServoControl_ApplySwingLevel(int8_t level, TickType_t now)
{
    PositionAxisState *state = &position_axes[EXCAVATOR_AXIS_SWING];
    const ExcavatorPositionAxisParameters *parameters =
        &g_excavator_position_axis_parameters[EXCAVATOR_AXIS_SWING];
    int8_t direction;
    int16_t signed_speed;

    if (level == 0)
    {
        if (state->jog_direction != 0 || (EXCAVATOR_CHASSIS_TEST_MODE && g_chassis_goal_speeds[0] != 0U))
        {
            ServoControl_StopPositionAxis(EXCAVATOR_AXIS_SWING);
        }
        return;
    }

    direction = (level > 0) ? parameters->positive_direction
                            : (int8_t)-parameters->positive_direction;
    if (state->jog_direction != 0 && state->jog_direction != direction &&
        !ServoControl_WriteSwingSpeed(state, 0))
    {
        return;
    }

    const uint16_t swing_speed = EXCAVATOR_CHASSIS_TEST_MODE
                                   ? EXCAVATOR_CHASSIS_SWING_SPEED : parameters->wireless_speed;
    signed_speed = (int16_t)((int16_t)direction * (int16_t)swing_speed);
    if ((state->jog_direction != direction || (EXCAVATOR_CHASSIS_TEST_MODE &&
        g_chassis_goal_speeds[0] != ServoControl_EncodeWheelSpeed(signed_speed))) &&
        !ServoControl_WriteSwingSpeed(state, signed_speed))
    {
        return;
    }

    state->speed = parameters->wireless_speed;
    state->jog_direction = direction;
    state->last_jog_command = now;
}

static void ServoControl_ApplyWirelessCommand(const ExcavatorVelocityCmd_t *command)
{
    TickType_t now = xTaskGetTickCount();

    ServoControl_SetWheelLevel(EXCAVATOR_WHEEL_LEFT, command->left_track);
    ServoControl_SetWheelLevel(EXCAVATOR_WHEEL_RIGHT, command->right_track);
    ServoControl_ApplySwingLevel(command->swing, now);
    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        if (!position_axes[EXCAVATOR_AXIS_SWING].online ||
            !wheel_axes[0].online || !wheel_axes[1].online)
            ServoControl_StopChassis();
        return;
    }
    ServoControl_ApplyPositionLevel(EXCAVATOR_AXIS_BOOM, command->boom, now);
    ServoControl_ApplyPositionLevel(EXCAVATOR_AXIS_ARM, command->arm, now);
    ServoControl_ApplyPositionLevel(EXCAVATOR_AXIS_PITCH, command->bucket, now);
}

static void ServoControl_StopChassis(void)
{
    g_chassis_armed = 0U;
    chassis_neutral_seen = false;
    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS) return;
    const uint8_t ids[3] = {EXCAVATOR_SERVO_SWING_ID, EXCAVATOR_SERVO_LEFT_WHEEL_ID,
                            EXCAVATOR_SERVO_RIGHT_WHEEL_ID};
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (BSP_STS3215_WriteGoalSpeed(ids[i], 0U) == BSP_STS3215_OK)
        {
            g_chassis_goal_speeds[i] = 0U;
            if (i == 0U) position_axes[EXCAVATOR_AXIS_SWING].jog_direction = 0;
            else wheel_axes[i - 1U].level = 0;
        }
    }
}

static bool ServoControl_PrepareChassis(void)
{
    g_chassis_armed = 0U;
    chassis_neutral_seen = false;
    bool disabled = true;
    for (uint8_t id = 4U; id <= 6U; id++)
        if (BSP_STS3215_SetTorqueEnabled(id, 0U) != BSP_STS3215_OK) disabled = false;
    if (!disabled) goto failed;
    for (uint8_t id = 4U; id <= 6U; id++)
    {
        uint8_t mode;
        if (BSP_STS3215_SetOperatingMode(id, BSP_STS3215_MODE_WHEEL) != BSP_STS3215_OK ||
            BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_WHEEL)
            goto failed;
        g_servo_read_modes[id] = mode;
        if (BSP_STS3215_WriteGoalSpeed(id, 0U) != BSP_STS3215_OK) goto failed;
        g_chassis_goal_speeds[id - 4U] = 0U;
    }
    for (uint8_t id = 4U; id <= 6U; id++)
    {
        uint8_t mode;
        uint16_t goal;
        vTaskDelay(pdMS_TO_TICKS(100U));
        if (BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_WHEEL ||
            BSP_STS3215_ReadGoalSpeed(id, &goal) != BSP_STS3215_OK || goal != 0U) goto failed;
        if (BSP_STS3215_SetTorqueEnabled(id, 1U) != BSP_STS3215_OK) goto failed;
    }
    wheel_axes[0].level = 0;
    wheel_axes[1].level = 0;
    position_axes[EXCAVATOR_AXIS_SWING].jog_direction = 0;
    return true;
failed:
    for (uint8_t id = 4U; id <= 6U; id++) (void)BSP_STS3215_SetTorqueEnabled(id, 0U);
    return false;
}

static void ServoControl_ApplyControlInput(const ExcavatorVelocityCmd_t *command)
{
    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS) return;
    if (EXCAVATOR_POSITION_REMOTE_MODE && !remote_enabled) return;
    const TickType_t now = xTaskGetTickCount();
    last_chassis_command = now;
    if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_HandleRemotePositions(command);
    if (g_chassis_armed == 0U)
    {
        if (command->left_track != 0 || command->right_track != 0 || command->swing != 0 ||
            (EXCAVATOR_POSITION_REMOTE_MODE && (command->boom != 0 || command->arm != 0 ||
                                               command->bucket != 0 || command->gripper != 0)))
        {
            chassis_neutral_seen = false;
            return;
        }
        if (!chassis_neutral_seen)
        {
            chassis_neutral_seen = true;
            chassis_neutral_since = now;
            return;
        }
        if (now - chassis_neutral_since < pdMS_TO_TICKS(EXCAVATOR_CHASSIS_NEUTRAL_MS)) return;
        if (!position_axes[EXCAVATOR_AXIS_SWING].online || !wheel_axes[0].online || !wheel_axes[1].online ||
            g_servo_read_modes[4] != BSP_STS3215_MODE_WHEEL ||
            g_servo_read_modes[5] != BSP_STS3215_MODE_WHEEL || g_servo_read_modes[6] != BSP_STS3215_MODE_WHEEL ||
            g_servo_read_torque[4] != 1U || g_servo_read_torque[5] != 1U || g_servo_read_torque[6] != 1U)
            return;
        if (EXCAVATOR_POSITION_REMOTE_MODE && remote_ready_mask != 23U) return;
        bool stopped = true;
        for (uint8_t id = 4U; id <= 6U; id++)
        {
            if (BSP_STS3215_WriteGoalSpeed(id, 0U) != BSP_STS3215_OK) stopped = false;
            else g_chassis_goal_speeds[id - 4U] = 0U;
        }
        if (stopped)
        {
            wheel_axes[0].level = 0;
            wheel_axes[1].level = 0;
            position_axes[EXCAVATOR_AXIS_SWING].jog_direction = 0;
            g_chassis_armed = 1U;
        }
        return;
    }
    ServoControl_ApplyWirelessCommand(command);
}

static bool ServoControl_RadioActive(void)
{
    return wireless_input_count != 0U && !wireless_timeout_pending &&
           xTaskGetTickCount() - last_wireless_input_time <= pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS) &&
           !ServoControl_RemoteNeutral(&last_wireless_input);
}

static void ServoControl_HandleWireless(const ExcavatorVelocityCmd_t *command)
{
    const TickType_t now = xTaskGetTickCount();
    if (wireless_input_count == 0U || now - last_wireless_input_time > pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS))
    {
        radio_armed = false;
        radio_neutral_seen = false;
    }
    last_wireless_input = *command;
    last_wireless_input_time = now;
    wireless_input_count++;
    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS || (EXCAVATOR_POSITION_REMOTE_MODE && !remote_enabled)) return;
    if (!EXCAVATOR_POSITION_REMOTE_MODE) { ServoControl_ApplyControlInput(command); return; }
    const bool neutral = ServoControl_RemoteNeutral(command);
    if (!radio_armed)
    {
        if (!neutral) radio_neutral_seen = false;
        else if (!radio_neutral_seen) { radio_neutral_seen = true; radio_neutral_since = now; }
        else if (now - radio_neutral_since >= pdMS_TO_TICKS(EXCAVATOR_CHASSIS_NEUTRAL_MS)) radio_armed = true;
    }
    if (host_owned)
    {
        if (neutral) return; /* Radio heartbeat must never overwrite a host action. */
        /* Even an unarmed deflection cancels host input; movement still needs radio neutral arming. */
        const bool was_remote_armed = remote_armed;
        const uint8_t was_chassis_armed = g_chassis_armed;
        ServoControl_EndHost();
        host_preempted = true;
        if (radio_armed) { remote_armed = was_remote_armed; g_chassis_armed = was_chassis_armed; }
        ServoControl_SendText("HOST_PREEMPTED\r\n");
    }
    if (neutral || radio_armed) ServoControl_ApplyControlInput(command);
}

static void ServoControl_ProcessWirelessInput(void)
{
    ExcavatorVelocityCmd_t command;
    if (wireless_timeout_pending)
    {
        wireless_timeout_pending = false;
        radio_armed = false;
        radio_neutral_seen = false;
        if (!host_owned)
        {
            ServoControl_StopChassis();
            if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_StopRemotePositions();
        }
    }
    if (wireless_command_queue != NULL &&
        xQueueReceive(wireless_command_queue, &command, 0U) == pdPASS)
    {
        if (EXCAVATOR_CHASSIS_TEST_MODE) ServoControl_HandleWireless(&command);
        else ServoControl_ApplyWirelessCommand(&command);
    }
}

static void ServoControl_UpdateFeedback(void)
{
    for (uint8_t i = 0U; i < EXCAVATOR_POSITION_AXIS_COUNT; i++)
    {
        PositionAxisState *state = &position_axes[i];
        uint16_t position;

        const bool mode_ready = (EXCAVATOR_READ_ONLY_DIAGNOSTICS || EXCAVATOR_CHASSIS_TEST_MODE)
                                    ? true : ServoControl_IsContinuousAxis(i)
                                    ? ServoControl_EnsureContinuousMode(state)
                                    : ServoControl_EnsurePositionMode(state);

        if (mode_ready &&
            ServoControl_ReadAxisPosition(i, &position) == BSP_STS3215_OK)
        {
            if (!state->online)
            {
                state->target_position = position;
                state->jog_direction = 0;
                state->command_pending = !EXCAVATOR_READ_ONLY_DIAGNOSTICS && !EXCAVATOR_CHASSIS_TEST_MODE &&
                                         !ServoControl_IsContinuousAxis(i);
            }

            state->current_position = position;
            g_servo_read_positions[state->id] = position;
            state->online = true;
            uint8_t mode;
            if (BSP_STS3215_ReadOperatingMode(state->id, &mode) == BSP_STS3215_OK)
                g_servo_read_modes[state->id] = mode;
            else g_servo_read_modes[state->id] = 0xFFU;
            if (EXCAVATOR_CHASSIS_TEST_MODE)
                state->operating_mode_verified = ServoControl_IsContinuousAxis(i) &&
                                                g_servo_read_modes[state->id] == BSP_STS3215_MODE_WHEEL;
        }
        else
        {
            state->online = false;
            state->operating_mode_verified = false;
            state->jog_direction = 0;
            state->command_pending = false;
            g_servo_read_modes[state->id] = 0xFFU;
        }
        if ((calibration_ready_mask & (1U << i)) != 0U)
        {
            uint8_t torque;
            if (!state->online || !state->limits_enabled ||
                g_servo_read_modes[state->id] != BSP_STS3215_MODE_POSITION ||
                BSP_STS3215_ReadTorqueEnabled(state->id, &torque) != BSP_STS3215_OK || torque != 1U ||
                (!EXCAVATOR_POSITION_REMOTE_MODE &&
                 (state->current_position < state->minimum_position || state->current_position > state->maximum_position)))
            {
                calibration_ready_mask &= (uint8_t)~(1U << i);
                (void)BSP_STS3215_SetTorqueEnabled(state->id, 0U);
            }
        }
    }

    for (uint8_t i = 0U; i < EXCAVATOR_WHEEL_AXIS_COUNT; i++)
    {
        WheelAxisState *state = &wheel_axes[i];
        uint16_t position;

        if ((EXCAVATOR_READ_ONLY_DIAGNOSTICS || EXCAVATOR_CHASSIS_TEST_MODE || ServoControl_EnsureWheelMode(state)) &&
            BSP_STS3215_ReadPresentPosition(state->id, &position) == BSP_STS3215_OK)
        {
            state->online = true;
            g_servo_read_positions[state->id] = position;
            uint8_t mode;
            if (BSP_STS3215_ReadOperatingMode(state->id, &mode) == BSP_STS3215_OK)
                g_servo_read_modes[state->id] = mode;
            else g_servo_read_modes[state->id] = 0xFFU;
            if (EXCAVATOR_CHASSIS_TEST_MODE) state->wheel_mode_verified =
                g_servo_read_modes[state->id] == BSP_STS3215_MODE_WHEEL;
        }
        else
        {
            state->online = false;
            state->wheel_mode_verified = false;
            state->level = 0;
            g_servo_read_modes[state->id] = 0xFFU;
        }
    }
    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        for (uint8_t id = 4U; id <= 6U; id++)
        {
            uint16_t speed;
            if (BSP_STS3215_ReadGoalSpeed(id, &speed) == BSP_STS3215_OK)
                g_chassis_goal_speeds[id - 4U] = speed;
            uint8_t torque;
            g_servo_read_torque[id] = BSP_STS3215_ReadTorqueEnabled(id, &torque) == BSP_STS3215_OK ? torque : 0xFFU;
        }
        if (g_chassis_armed && (g_servo_read_modes[4] != BSP_STS3215_MODE_WHEEL ||
            g_servo_read_modes[5] != BSP_STS3215_MODE_WHEEL || g_servo_read_modes[6] != BSP_STS3215_MODE_WHEEL ||
            g_servo_read_torque[4] != 1U || g_servo_read_torque[5] != 1U || g_servo_read_torque[6] != 1U))
            ServoControl_StopChassis();
        if (!g_chassis_armed && (g_chassis_goal_speeds[0] != 0U ||
            g_chassis_goal_speeds[1] != 0U || g_chassis_goal_speeds[2] != 0U))
            ServoControl_StopChassis();
    }
    if (EXCAVATOR_POSITION_REMOTE_MODE && !EXCAVATOR_READ_ONLY_DIAGNOSTICS) ServoControl_UpdateRemoteReadiness();
}

static void ServoControl_UpdateMotion(void)
{
    if (host_streaming && xTaskGetTickCount() - last_host_command > pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS))
        ServoControl_EndHost();
    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS)
    {
        return;
    }
    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        if (g_chassis_armed && xTaskGetTickCount() - last_chassis_command >
            pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS)) ServoControl_StopChassis();
        if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_UpdateRemoteMotion();
        return;
    }
    BspSts3215PositionCommand commands[EXCAVATOR_POSITION_AXIS_COUNT];
    uint8_t command_axes[EXCAVATOR_POSITION_AXIS_COUNT];
    TickType_t now = xTaskGetTickCount();
    uint8_t command_count = 0U;

    for (uint8_t i = 0U; i < EXCAVATOR_POSITION_AXIS_COUNT; i++)
    {
        PositionAxisState *state = &position_axes[i];

        if (state->jog_direction != 0 &&
            (now - state->last_jog_command) > pdMS_TO_TICKS(EXCAVATOR_JOG_TIMEOUT_MS))
        {
            ServoControl_StopPositionAxis(i);
        }

        if (!ServoControl_IsContinuousAxis(i) && state->command_pending && state->online)
        {
            commands[command_count].id = state->id;
            commands[command_count].position = state->target_position;
            commands[command_count].speed = state->speed;
            commands[command_count].acceleration = EXCAVATOR_SERVO_DEFAULT_ACCELERATION;
            command_axes[command_count] = i;
            command_count++;
        }
    }

    if (command_count > 0U &&
        BSP_STS3215_SyncWritePosition(commands, command_count) == BSP_STS3215_OK)
    {
        for (uint8_t i = 0U; i < command_count; i++)
        {
            position_axes[command_axes[i]].command_pending = false;
        }
    }
}

static bool ServoControl_ReadCalibrationPosition(uint8_t axis, uint16_t *current)
{
    PositionAxisState *state = &position_axes[axis];
    return state->limits_enabled && ServoControl_ReadAxisPosition(axis, current) == BSP_STS3215_OK &&
           (EXCAVATOR_POSITION_REMOTE_MODE || (*current >= state->minimum_position && *current <= state->maximum_position));
}

static bool ServoControl_WritePositionTarget(uint8_t axis, uint16_t target, uint16_t speed, bool remote)
{
    PositionAxisState *state = &position_axes[axis];
    int16_t adjustment;
    if (ServoControl_GetReferenceAdjustment(axis, &adjustment) != BSP_STS3215_OK) return false;
    int32_t raw_target = (int32_t)target - adjustment;
    if (raw_target < 0 || raw_target > (int32_t)BSP_STS3215_POSITION_MAX) return false;
    BspSts3215PositionCommand command = {.id = state->id, .position = (uint16_t)raw_target, .speed = speed, .acceleration = 10U};
    BspSts3215Result result = remote ? BSP_STS3215_WriteRemotePosition(&command) :
                                     BSP_STS3215_WriteCalibrationPosition(&command);
    if (result != BSP_STS3215_OK) return false;
    state->target_position = target;
    state->speed = speed;
    return true;
}

static bool ServoControl_WriteCalibrationTarget(uint8_t axis, uint16_t target, uint16_t speed)
{
    return ServoControl_WritePositionTarget(axis, target, speed, false);
}

static bool ServoControl_RemoteNeutral(const ExcavatorVelocityCmd_t *command)
{
    return command->left_track == 0 && command->right_track == 0 && command->swing == 0 &&
           command->boom == 0 && command->arm == 0 && command->bucket == 0 && command->gripper == 0;
}

static void ServoControl_StopRemoteAxis(uint8_t axis)
{
    if ((remote_goal_active_mask & (1U << axis)) != 0U)
    {
        uint16_t current;
        if (ServoControl_ReadAxisPosition(axis, &current) != BSP_STS3215_OK ||
            !ServoControl_WriteCalibrationTarget(axis, current, EXCAVATOR_REMOTE_POSITION_SPEED))
        {
            (void)BSP_STS3215_SetTorqueEnabled(position_axes[axis].id, 0U);
            remote_ready_mask &= (uint8_t)~(1U << axis);
        }
    }
    remote_goal_active_mask &= (uint8_t)~(1U << axis);
    remote_directions[axis] = 0;
}

static void ServoControl_StopRemotePositions(void)
{
    remote_armed = false;
    remote_neutral_seen = false;
    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS) return;
    for (uint8_t axis = 0; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
        if (ServoControl_IsLimitableAxis(axis)) ServoControl_StopRemoteAxis(axis);
}

/* End only UART ownership. A late HOST_END cannot stop a radio operator. */
static void ServoControl_EndHost(void)
{
    if (host_owned)
    {
        ServoControl_StopRemotePositions();
        ServoControl_StopChassis();
    }
    host_owned = false;
    host_streaming = false;
}

static void ServoControl_SuspendRemote(void)
{
    host_owned = false;
    radio_armed = false;
    radio_neutral_seen = false;
    ServoControl_StopRemotePositions();
    ServoControl_StopChassis();
    remote_enabled = false;
    host_streaming = false;
}

static void ServoControl_UpdateRemoteReadiness(void)
{
    for (uint8_t axis = 0; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
    {
        if (!ServoControl_IsLimitableAxis(axis)) continue;
        PositionAxisState *state = &position_axes[axis];
        uint8_t torque;
        bool available = state->online && state->limits_enabled &&
                         g_servo_read_modes[state->id] == BSP_STS3215_MODE_POSITION &&
                         BSP_STS3215_ReadTorqueEnabled(state->id, &torque) == BSP_STS3215_OK && torque == 1U;
        if (!available)
        {
            ServoControl_StopRemoteAxis(axis);
            remote_ready_mask &= (uint8_t)~(1U << axis);
            continue;
        }
        if ((remote_ready_mask & (1U << axis)) == 0U)
        {
            uint16_t goal;
            if (ServoControl_ReadAxisGoal(axis, &goal) == BSP_STS3215_OK &&
                (uint16_t)(goal > state->current_position ? goal - state->current_position : state->current_position - goal) <= EXCAVATOR_REMOTE_HOLD_TOLERANCE &&
                ExcavatorPosition_TargetAllowed(state->current_position, goal, state->minimum_position,
                                                 state->maximum_position, EXCAVATOR_REMOTE_HOLD_TOLERANCE))
            {
                state->target_position = goal;
                remote_ready_mask |= (uint8_t)(1U << axis);
            }
        }
    }
    if (remote_ready_mask != 23U && (remote_armed || g_chassis_armed))
    {
        ServoControl_StopRemotePositions();
        ServoControl_StopChassis();
    }
}

static void ServoControl_HandleRemotePositions(const ExcavatorVelocityCmd_t *command)
{
    if (!remote_enabled) return;
    TickType_t now = xTaskGetTickCount();
    if (!remote_armed)
    {
        if (!ServoControl_RemoteNeutral(command)) { remote_neutral_seen = false; return; }
        if (!remote_neutral_seen) { remote_neutral_seen = true; remote_neutral_since = now; return; }
        if (now - remote_neutral_since >= pdMS_TO_TICKS(EXCAVATOR_CHASSIS_NEUTRAL_MS) && remote_ready_mask == 23U)
            remote_armed = true;
        return;
    }
    const int8_t levels[EXCAVATOR_POSITION_AXIS_COUNT] = {command->boom, command->arm, command->bucket, 0, command->gripper};
    for (uint8_t axis = 0; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
    {
        if (!ServoControl_IsLimitableAxis(axis)) continue;
        int8_t direction = levels[axis] > 0 ? g_excavator_position_axis_parameters[axis].positive_direction :
                           levels[axis] < 0 ? (int8_t)-g_excavator_position_axis_parameters[axis].positive_direction : 0;
        if (direction != remote_directions[axis]) ServoControl_StopRemoteAxis(axis);
        remote_directions[axis] = direction;
    }
}

static void ServoControl_UpdateRemoteMotion(void)
{
    if (!remote_armed || !remote_enabled || EXCAVATOR_READ_ONLY_DIAGNOSTICS) return;
    for (uint8_t axis = 0; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
    {
        if (xTaskGetTickCount() - last_chassis_command > pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS))
        {
            ServoControl_StopRemotePositions();
            ServoControl_StopChassis();
            return;
        }
        if (!ServoControl_IsLimitableAxis(axis) || remote_directions[axis] == 0) continue;
        PositionAxisState *state = &position_axes[axis];
        uint16_t current, goal, target;
        uint8_t torque;
        if ((remote_ready_mask & (1U << axis)) == 0U ||
            ServoControl_ReadAxisPosition(axis, &current) != BSP_STS3215_OK ||
            ServoControl_ReadAxisGoal(axis, &goal) != BSP_STS3215_OK ||
            BSP_STS3215_ReadTorqueEnabled(state->id, &torque) != BSP_STS3215_OK || torque != 1U)
        { ServoControl_SuspendRemote(); return; }
        const uint16_t speed = g_excavator_position_axis_parameters[axis].wireless_speed;
        uint8_t step = (uint8_t)((speed * EXCAVATOR_CONTROL_PERIOD_MS + 999U) / 1000U);
        if (step == 0U) step = 1U;
        if (step > EXCAVATOR_REMOTE_POSITION_STEP_MAX) step = EXCAVATOR_REMOTE_POSITION_STEP_MAX;
        const uint16_t maximum_lead = EXCAVATOR_REMOTE_POSITION_WINDOW_MAX - step - EXCAVATOR_REMOTE_POSITION_SAMPLE_MARGIN;
        if ((uint16_t)(goal > current ? goal - current : current - goal) > maximum_lead) continue;
        const uint16_t base = ((remote_directions[axis] > 0 && goal > current) ||
                               (remote_directions[axis] < 0 && goal < current)) ? goal : current;
        if (!ExcavatorPosition_NextTarget(base, state->minimum_position, state->maximum_position,
                                          remote_directions[axis], step, &target)) continue;
        if (!ExcavatorPosition_TargetAllowed(current, target, state->minimum_position,
                                              state->maximum_position, EXCAVATOR_REMOTE_POSITION_WINDOW_MAX)) continue;
        remote_goal_active_mask |= (uint8_t)(1U << axis);
        if (!ServoControl_WritePositionTarget(axis, target, speed, true))
        { ServoControl_SuspendRemote(); return; }
    }
}

static bool ServoControl_PrepareCalibration(uint8_t axis)
{
    PositionAxisState *state = &position_axes[axis];
    uint16_t current;
    uint8_t mode;
    calibration_ready_mask &= (uint8_t)~(1U << axis);
    if (!ServoControl_ReadCalibrationPosition(axis, &current)) return false;
    const uint16_t reference_before = current;
    ServoControl_StopChassis();
    if (BSP_STS3215_SetTorqueEnabled(state->id, 0U) != BSP_STS3215_OK) return false;
    if (!ServoControl_ReadCalibrationPosition(axis, &current) ||
        BSP_STS3215_SetOperatingMode(state->id, BSP_STS3215_MODE_POSITION) != BSP_STS3215_OK ||
        BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_POSITION)
        goto failed_calibration;
    /* Mode writes acknowledge before feedback necessarily settles. Keep torque off. */
    vTaskDelay(pdMS_TO_TICKS(100U));
    uint16_t previous = 0U;
    uint8_t stable_samples = 0U;
    for (uint8_t sample = 0U; sample < 10U && stable_samples < 3U; sample++)
    {
        if (!ServoControl_ReadCalibrationPosition(axis, &current)) goto failed_calibration;
        if (sample > 0U && (uint16_t)(previous > current ? previous - current : current - previous) <= 2U)
            stable_samples++;
        else stable_samples = 1U;
        previous = current;
        if (stable_samples < 3U) vTaskDelay(pdMS_TO_TICKS(20U));
    }
    if (stable_samples < 3U ||
        (EXCAVATOR_POSITION_REMOTE_MODE &&
         (uint16_t)(reference_before > current ? reference_before - current : current - reference_before) > 2U) ||
        !ServoControl_WriteCalibrationTarget(axis, current, EXCAVATOR_CALIBRATION_SPEED_MAX) ||
        BSP_STS3215_SetTorqueEnabled(state->id, 1U) != BSP_STS3215_OK ||
        !ServoControl_ReadCalibrationPosition(axis, &current))
    {
failed_calibration:
        (void)BSP_STS3215_SetTorqueEnabled(state->id, 0U);
        return false;
    }
    state->current_position = current;
    g_servo_read_modes[state->id] = BSP_STS3215_MODE_POSITION;
    calibration_ready_mask |= (uint8_t)(1U << axis);
    return true;
}

/* Cold servo power clears mode/torque RAM. Recover independently of the Linux host.
 * Latch only fresh current positions; never adopt a distant retained goal or an endpoint. */
static void ServoControl_UpdateStartup(void)
{
    if (!EXCAVATOR_POSITION_REMOTE_MODE || EXCAVATOR_READ_ONLY_DIAGNOSTICS) return;
    if (!remote_enabled || calibration_ready_mask != 0U)
    { startup_state = startup_attempted && startup_error != 0U ? "FAILED" : "MAINTENANCE"; startup_waiting = false; return; }
    bool online = true;
    for (uint8_t axis = 0U; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++) online &= position_axes[axis].online;
    for (uint8_t wheel = 0U; wheel < EXCAVATOR_WHEEL_AXIS_COUNT; wheel++) online &= wheel_axes[wheel].online;
    if (!online)
    {
        startup_state = "WAIT_SERVO"; startup_attempted = startup_waiting = false; startup_error = 0U;
        return;
    }
    bool chassis_ready = true;
    for (uint8_t id = 4U; id <= 6U; id++)
        chassis_ready &= g_servo_read_modes[id] == BSP_STS3215_MODE_WHEEL && g_servo_read_torque[id] == 1U;
    if (remote_ready_mask == 23U && chassis_ready)
    { startup_state = "READY"; startup_error = 0U; startup_attempted = startup_waiting = false; return; }
    if (startup_attempted) { startup_state = "FAILED"; return; }
    bool stable = true;
    for (uint8_t axis = 0U; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
    {
        if (!ServoControl_IsLimitableAxis(axis)) continue;
        const uint16_t current = position_axes[axis].current_position;
        const uint16_t previous = startup_previous[axis];
        if ((uint16_t)(current > previous ? current - previous : previous - current) > 2U) stable = false;
        startup_previous[axis] = current;
    }
    const TickType_t now = xTaskGetTickCount();
    if (ServoControl_RadioActive() || (host_owned && !host_input_neutral) || !stable)
    { startup_state = "WAIT_NEUTRAL"; startup_waiting = false; return; }
    startup_state = "WAIT_NEUTRAL";
    if (!startup_waiting) { startup_waiting = true; startup_neutral_since = now; return; }
    if (now - startup_neutral_since < pdMS_TO_TICKS(EXCAVATOR_CHASSIS_NEUTRAL_MS)) return;

    startup_attempted = true;
    startup_waiting = false;
    startup_state = "PREPARING";
    /* Preflight every axis before changing anything. Never unload a powered axis in a wrong mode. */
    for (uint8_t axis = 0U; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
    {
        if (!ServoControl_IsLimitableAxis(axis)) continue;
        uint8_t torque;
        uint16_t position;
        if (!ServoControl_ReadCalibrationPosition(axis, &position) ||
            BSP_STS3215_ReadTorqueEnabled(position_axes[axis].id, &torque) != BSP_STS3215_OK)
        { startup_error = 1U; goto failed_startup; }
        if (torque != 0U && g_servo_read_modes[position_axes[axis].id] != BSP_STS3215_MODE_POSITION)
        { startup_error = 2U; goto failed_startup; }
    }
    for (uint8_t id = 4U; id <= 6U; id++)
        if (g_servo_read_torque[id] > 1U ||
            (g_servo_read_torque[id] == 1U && g_servo_read_modes[id] != BSP_STS3215_MODE_WHEEL))
        { startup_error = 2U; goto failed_startup; }

    ServoControl_EndHost();
    ServoControl_StopRemotePositions();
    ServoControl_StopChassis();
    for (uint8_t id = 4U; id <= 6U; id++)
    {
        if (g_servo_read_torque[id] == 1U) continue;
        uint8_t mode;
        uint16_t goal;
        if (BSP_STS3215_SetTorqueEnabled(id, 0U) != BSP_STS3215_OK ||
            BSP_STS3215_SetOperatingMode(id, BSP_STS3215_MODE_WHEEL) != BSP_STS3215_OK ||
            BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_WHEEL ||
            BSP_STS3215_WriteGoalSpeed(id, 0U) != BSP_STS3215_OK)
        { startup_error = 3U; goto failed_startup; }
        vTaskDelay(pdMS_TO_TICKS(100U));
        if (BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK || mode != BSP_STS3215_MODE_WHEEL ||
            BSP_STS3215_ReadGoalSpeed(id, &goal) != BSP_STS3215_OK || goal != 0U ||
            BSP_STS3215_SetTorqueEnabled(id, 1U) != BSP_STS3215_OK)
        { startup_error = 3U; goto failed_startup; }
    }
    for (uint8_t axis = 0U; axis < EXCAVATOR_POSITION_AXIS_COUNT; axis++)
    {
        if (!ServoControl_IsLimitableAxis(axis) || (remote_ready_mask & (1U << axis)) != 0U) continue;
        uint8_t torque;
        if (BSP_STS3215_ReadTorqueEnabled(position_axes[axis].id, &torque) != BSP_STS3215_OK)
        { startup_error = 4U; goto failed_startup; }
        if (torque == 1U)
        {
            uint16_t current;
            if (!ServoControl_ReadCalibrationPosition(axis, &current) ||
                !ServoControl_WriteCalibrationTarget(axis, current, EXCAVATOR_CALIBRATION_SPEED_MAX))
            { startup_error = 4U; goto failed_startup; }
        }
        else if (!ServoControl_PrepareCalibration(axis)) { startup_error = 4U; goto failed_startup; }
    }
    calibration_ready_mask = 0U;
    ServoControl_UpdateFeedback();
    if (remote_ready_mask != 23U) { startup_error = 5U; goto failed_startup; }
    radio_armed = radio_neutral_seen = false;
    remote_armed = remote_neutral_seen = false;
    g_chassis_armed = 0U; chassis_neutral_seen = false;
    startup_state = "READY"; startup_error = 0U;
    return;
failed_startup:
    calibration_ready_mask = 0U;
    ServoControl_SuspendRemote();
    startup_state = "FAILED";
}

static void ServoControl_ProcessCommand(char *line)
{
    char *tokens[SERVO_CONTROL_TOKEN_COUNT_MAX] = {0};
    uint8_t token_count = ServoControl_Tokenize(line, tokens);
    uint8_t axis;
    uint16_t first_value;
    uint16_t second_value;
    int16_t signed_value;

    if (token_count == 0U) { ServoControl_SendText("ERR COMMAND\r\n"); return; }

    if (token_count == 1U && strcmp(tokens[0], "CAPS") == 0)
    {
        ServoControl_SendText(EXCAVATOR_POSITION_REMOTE_MODE && !EXCAVATOR_READ_ONLY_DIAGNOSTICS
                                  ? "CAPS HOST_CONTROL_V2\r\n" : "CAPS NONE\r\n");
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "PING") == 0)
    {
        ServoControl_SendText("PONG\r\n");
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "STATUS") == 0)
    {
        ServoControl_SendStatus();
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "READINGS") == 0)
    {
        char message[128];
        ServoControl_SendStatus();
        (void)snprintf(message, sizeof(message), "READINGS %u %u %u %u %u %u %u\r\n",
                       g_servo_read_positions[1], g_servo_read_positions[2],
                       g_servo_read_positions[3], g_servo_read_positions[4],
                       g_servo_read_positions[5], g_servo_read_positions[6],
                       g_servo_read_positions[8]);
        ServoControl_SendText(message);
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "BUS_DIAG") == 0)
    {
        BspSts3215Diagnostics diagnostics;
        char message[128];
        BSP_STS3215_GetDiagnostics(&diagnostics);
        (void)snprintf(message, sizeof(message), "BUS_DIAG %lu %lu %lu %lu %lu\r\n",
                       (unsigned long)diagnostics.rx_bytes, (unsigned long)diagnostics.overruns,
                       (unsigned long)diagnostics.framing_errors, (unsigned long)diagnostics.buffer_drops,
                       (unsigned long)diagnostics.timeouts);
        ServoControl_SendText(message);
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "MODES") == 0)
    {
        char message[96];
        (void)snprintf(message, sizeof(message), "MODES %u %u %u %u %u %u %u\r\n",
                       g_servo_read_modes[1], g_servo_read_modes[2], g_servo_read_modes[3],
                       g_servo_read_modes[4], g_servo_read_modes[5], g_servo_read_modes[6], g_servo_read_modes[8]);
        ServoControl_SendText(message);
        return;
    }
    if (token_count == 1U && strcmp(tokens[0], "CHASSIS") == 0)
    {
        char message[96];
        (void)snprintf(message, sizeof(message), "CHASSIS %u %u %u %u\r\n",
                       g_chassis_armed, g_chassis_goal_speeds[0], g_chassis_goal_speeds[1], g_chassis_goal_speeds[2]);
        ServoControl_SendText(message);
        return;
    }
    if (token_count == 1U && strcmp(tokens[0], "REMOTE_INPUT") == 0)
    {
        char message[128];
        (void)snprintf(message, sizeof(message), "REMOTE_INPUT %d %d %d %d %d %d %d %lu %lu\r\n",
                       last_wireless_input.left_track, last_wireless_input.right_track, last_wireless_input.swing,
                       last_wireless_input.boom, last_wireless_input.arm, last_wireless_input.bucket, last_wireless_input.gripper,
                       (unsigned long)(xTaskGetTickCount() - last_wireless_input_time), (unsigned long)wireless_input_count);
        ServoControl_SendText(message);
        return;
    }
    if (token_count == 1U && strcmp(tokens[0], "FLASH_LIMITS") == 0)
    {
        char message[128];
        (void)snprintf(message, sizeof(message), "FLASH_LIMITS %s %u %u %u %u %u %u %u %u\r\n",
                       g_excavator_machine_profile_id,
                       g_excavator_position_axis_parameters[0].minimum_position, g_excavator_position_axis_parameters[0].maximum_position,
                       g_excavator_position_axis_parameters[1].minimum_position, g_excavator_position_axis_parameters[1].maximum_position,
                       g_excavator_position_axis_parameters[2].minimum_position, g_excavator_position_axis_parameters[2].maximum_position,
                       g_excavator_position_axis_parameters[4].minimum_position, g_excavator_position_axis_parameters[4].maximum_position);
        ServoControl_SendText(message);
        return;
    }
    if (token_count == 2U && strcmp(tokens[0], "POSITION_DIAG") == 0 && ServoControl_ParseAxis(tokens[1], &axis))
    {
        uint8_t mode, torque;
        uint16_t current, goal, speed, offset;
        uint8_t id = position_axes[axis].id;
        if (BSP_STS3215_ReadOperatingMode(id, &mode) != BSP_STS3215_OK ||
            BSP_STS3215_ReadTorqueEnabled(id, &torque) != BSP_STS3215_OK ||
            BSP_STS3215_ReadPresentPosition(id, &current) != BSP_STS3215_OK ||
            BSP_STS3215_ReadGoalPosition(id, &goal) != BSP_STS3215_OK ||
            BSP_STS3215_ReadGoalSpeed(id, &speed) != BSP_STS3215_OK ||
            BSP_STS3215_ReadPositionOffset(id, &offset) != BSP_STS3215_OK)
        { ServoControl_SendText("ERR POSITION_DIAG\r\n"); return; }
        char message[96];
        (void)snprintf(message, sizeof(message), "POSITION_DIAG %u %u %u %u %u %u %u\r\n", id, mode, torque, current, goal, speed, offset);
        ServoControl_SendText(message);
        return;
    }
    if (token_count == 2U && strcmp(tokens[0], "POSITION_DEADBAND") == 0 && ServoControl_ParseAxis(tokens[1], &axis))
    {
        uint8_t positive, negative;
        uint8_t id = position_axes[axis].id;
        if (BSP_STS3215_ReadPositionDeadband(id, &positive, &negative) != BSP_STS3215_OK)
        { ServoControl_SendText("ERR POSITION_DEADBAND\r\n"); return; }
        char message[64];
        (void)snprintf(message, sizeof(message), "POSITION_DEADBAND %u %u %u\r\n", id, positive, negative);
        ServoControl_SendText(message);
        return;
    }
    if (token_count == 3U && strcmp(tokens[0], "SERVO_REG") == 0 && ServoControl_ParseAxis(tokens[1], &axis) &&
        ServoControl_ParseUnsigned(tokens[2], 254U, &first_value))
    {
        uint16_t value;
        uint8_t id = position_axes[axis].id;
        if (BSP_STS3215_ReadRegisterWord(id, (uint8_t)first_value, &value) != BSP_STS3215_OK)
        { ServoControl_SendText("ERR SERVO_REG\r\n"); return; }
        char message[64];
        (void)snprintf(message, sizeof(message), "SERVO_REG %u %u %u\r\n", id, first_value, value);
        ServoControl_SendText(message);
        return;
    }

    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS)
    {
        ServoControl_SendText("ERR READ_ONLY\r\n");
        return;
    }

    if (EXCAVATOR_POSITION_REMOTE_MODE && token_count == 1U && strcmp(tokens[0], "STOP_CONTROL") == 0)
    {
        ServoControl_SuspendRemote();
        remote_enabled = true; /* Operator can re-arm with one second of neutral radio input. */
        ServoControl_SendText("OK STOP_CONTROL\r\n");
        return;
    }

    if (EXCAVATOR_POSITION_REMOTE_MODE && token_count == 1U && strcmp(tokens[0], "HOST_BEGIN") == 0)
    {
        ServoControl_ProcessWirelessInput();
        if (ServoControl_RadioActive()) { ServoControl_SendText("ERR RADIO_ACTIVE\r\n"); return; }
        ServoControl_EndHost();
        ServoControl_StopRemotePositions();
        ServoControl_StopChassis();
        host_preempted = false;
        host_input_neutral = true;
        host_owned = true;
        host_streaming = true;
        last_host_command = xTaskGetTickCount();
        remote_enabled = true;
        calibration_ready_mask = 0U;
        ServoControl_SendText("OK HOST_BEGIN\r\n");
        return;
    }
    if (EXCAVATOR_POSITION_REMOTE_MODE && token_count == 1U && strcmp(tokens[0], "HOST_END") == 0)
    {
        ServoControl_EndHost();
        ServoControl_SendText("OK HOST_END\r\n");
        return;
    }
    if (EXCAVATOR_POSITION_REMOTE_MODE && strcmp(tokens[0], "HOST_CONTROL") == 0)
    {
        int16_t levels[7];
        bool valid = token_count == 8U;
        for (uint8_t i = 0U; valid && i < 7U; i++)
            valid = ServoControl_ParseSigned(tokens[i + 1U], i < 2U ? -6 : -10, i < 2U ? 6 : 10, &levels[i]);
        if (!host_owned || !host_streaming || !remote_enabled || !valid ||
            xTaskGetTickCount() - last_host_command > pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS))
        {
            ServoControl_EndHost();
            ServoControl_SendText(host_preempted ? "ERR HOST_PREEMPTED\r\n" : "ERR HOST_CONTROL\r\n");
            return;
        }
        ExcavatorVelocityCmd_t input = {(int8_t)levels[0], (int8_t)levels[1], (int8_t)levels[2],
                                       (int8_t)levels[3], (int8_t)levels[4], (int8_t)levels[5], (int8_t)levels[6]};
        last_host_command = xTaskGetTickCount();
        host_input_neutral = ServoControl_RemoteNeutral(&input);
        /* Process radio before accepting UART input, including a freshly queued override. */
        ServoControl_ProcessWirelessInput();
        if (!host_owned) { ServoControl_SendText("ERR HOST_PREEMPTED\r\n"); return; }
        ServoControl_ApplyControlInput(&input);
        ServoControl_SendText("OK HOST_CONTROL\r\n");
        return;
    }

    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        if (EXCAVATOR_POSITION_REMOTE_MODE && token_count == 1U && strcmp(tokens[0], "PREPARE_REMOTE") == 0)
        {
            ServoControl_SuspendRemote();
            bool prepared = ServoControl_PrepareChassis();
            for (uint8_t index = 0; prepared && index < EXCAVATOR_POSITION_AXIS_COUNT; index++)
                if (ServoControl_IsLimitableAxis(index)) prepared = ServoControl_PrepareCalibration(index);
            if (prepared)
            {
                calibration_ready_mask = 0U;
                ServoControl_UpdateRemoteReadiness();
                prepared = remote_ready_mask == 23U;
            }
            remote_enabled = prepared;
            ServoControl_SendText(prepared ? "OK PREPARE_REMOTE\r\n" : "ERR PREPARE_REMOTE\r\n");
            return;
        }
        if (EXCAVATOR_POSITION_REMOTE_MODE && token_count == 1U && strcmp(tokens[0], "PAUSE_REMOTE") == 0)
        {
            ServoControl_SuspendRemote();
            ServoControl_StopChassis();
            ServoControl_SendText("OK PAUSE_REMOTE\r\n");
            return;
        }
        if (EXCAVATOR_POSITION_REMOTE_MODE && token_count == 1U && strcmp(tokens[0], "RESUME_REMOTE") == 0)
        {
            ServoControl_UpdateRemoteReadiness();
            if (remote_ready_mask != 23U) { ServoControl_SendText("ERR NOT_READY\r\n"); return; }
            host_owned = false;
            host_streaming = false;
            remote_armed = false;
            remote_neutral_seen = false;
            calibration_ready_mask = 0U;
            remote_enabled = true;
            ServoControl_SendText("OK RESUME_REMOTE\r\n");
            return;
        }
        if (EXCAVATOR_MANUAL_CALIBRATION && token_count == 2U && strcmp(tokens[0], "CAL_PREPARE") == 0 &&
            ServoControl_ParseAxis(tokens[1], &axis))
        {
            if (!ServoControl_IsLimitableAxis(axis)) { ServoControl_SendText("ERR NO_LIMIT\r\n"); return; }
            if (!position_axes[axis].limits_enabled) { ServoControl_SendText("ERR UNCALIBRATED\r\n"); return; }
            if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_SuspendRemote();
            ServoControl_SendText(ServoControl_PrepareCalibration(axis) ? "OK CAL_PREPARE\r\n" : "ERR CAL_PREPARE\r\n");
            return;
        }
        if (EXCAVATOR_MANUAL_CALIBRATION && token_count == 4U && strcmp(tokens[0], "MOVE") == 0 &&
            ServoControl_ParseAxis(tokens[1], &axis) && ServoControl_IsLimitableAxis(axis) &&
            ServoControl_ParseUnsigned(tokens[2], BSP_STS3215_POSITION_MAX, &first_value) &&
            ServoControl_ParseUnsigned(tokens[3], EXCAVATOR_SERVO_SPEED_MAX, &second_value))
        {
            uint16_t current;
            PositionAxisState *state = &position_axes[axis];
            if ((calibration_ready_mask & (1U << axis)) == 0U) { ServoControl_SendText("ERR AXIS_LOCKED\r\n"); return; }
            if (second_value == 0U || second_value > EXCAVATOR_CALIBRATION_SPEED_MAX)
            { ServoControl_SendText("ERR SPEED\r\n"); return; }
            if (!ServoControl_ReadCalibrationPosition(axis, &current)) { ServoControl_SendText("ERR RANGE\r\n"); return; }
            if ((uint16_t)(current > state->target_position ? current - state->target_position : state->target_position - current) > EXCAVATOR_POSITION_SETTLED_TOLERANCE)
            { ServoControl_SendText("ERR BUSY\r\n"); return; }
            if (!ExcavatorPosition_TargetAllowed(current, first_value, state->minimum_position, state->maximum_position,
                                                 EXCAVATOR_CALIBRATION_STEP_MAX))
            { ServoControl_SendText("ERR RANGE\r\n"); return; }
            uint8_t torque;
            if (BSP_STS3215_ReadTorqueEnabled(state->id, &torque) != BSP_STS3215_OK || torque != 1U)
            { ServoControl_SendText("ERR CAL_PREPARE\r\n"); return; }
            ServoControl_SendText(ServoControl_WriteCalibrationTarget(axis, first_value, second_value) ?
                                  "OK MOVE\r\n" : "ERR MOVE\r\n");
            return;
        }
        if (token_count == 1U && strcmp(tokens[0], "PREPARE_CHASSIS") == 0)
        {
            ServoControl_SendText(ServoControl_PrepareChassis() ? "OK PREPARE_CHASSIS\r\n" : "ERR PREPARE_CHASSIS\r\n");
            return;
        }
        if (token_count == 4U && strcmp(tokens[0], "LIMIT") == 0 &&
            ServoControl_ParseAxis(tokens[1], &axis) &&
            ServoControl_ParseUnsigned(tokens[2], BSP_STS3215_POSITION_MAX, &first_value) &&
            ServoControl_ParseUnsigned(tokens[3], BSP_STS3215_POSITION_MAX, &second_value) && first_value < second_value)
        {
            uint16_t current;
            if (!ServoControl_IsLimitableAxis(axis)) { ServoControl_SendText("ERR NO_LIMIT\r\n"); return; }
            if (EXCAVATOR_POSITION_REMOTE_MODE &&
                (first_value < g_excavator_position_axis_parameters[axis].minimum_position ||
                 second_value > g_excavator_position_axis_parameters[axis].maximum_position))
            { ServoControl_SendText("ERR FLASH_RANGE\r\n"); return; }
            if (ServoControl_ReadAxisPosition(axis, &current) != BSP_STS3215_OK)
            { ServoControl_SendText("ERR OFFLINE\r\n"); return; }
            if (current < first_value || current > second_value) { ServoControl_SendText("ERR RANGE\r\n"); return; }
            if ((calibration_ready_mask & (1U << axis)) != 0U &&
                (position_axes[axis].target_position < first_value || position_axes[axis].target_position > second_value))
            { ServoControl_SendText("ERR BUSY\r\n"); return; }
            position_axes[axis].minimum_position = first_value;
            position_axes[axis].maximum_position = second_value;
            position_axes[axis].limits_enabled = true;
            if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_SuspendRemote();
            ServoControl_SendText("OK LIMIT\r\n");
            return;
        }
        if (token_count == 2U && strcmp(tokens[0], "CLEAR") == 0 && ServoControl_ParseAxis(tokens[1], &axis))
        {
            if (!ServoControl_IsLimitableAxis(axis)) { ServoControl_SendText("ERR NO_LIMIT\r\n"); return; }
            position_axes[axis].minimum_position = g_excavator_position_axis_parameters[axis].minimum_position;
            position_axes[axis].maximum_position = g_excavator_position_axis_parameters[axis].maximum_position;
            position_axes[axis].limits_enabled = EXCAVATOR_POSITION_REMOTE_MODE &&
                g_excavator_position_axis_parameters[axis].default_limits_enabled;
            calibration_ready_mask &= (uint8_t)~(1U << axis);
            if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_SuspendRemote();
            ServoControl_SendText("OK CLEAR\r\n");
            return;
        }
        if (token_count == 2U && strcmp(tokens[0], "STOP") == 0 && ServoControl_ParseAxis(tokens[1], &axis))
        {
            if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_SuspendRemote();
            if (axis == EXCAVATOR_AXIS_SWING)
            {
                ServoControl_StopChassis();
                ServoControl_SendText("OK STOP\r\n");
            }
            else if ((calibration_ready_mask & (1U << axis)) != 0U)
            {
                uint16_t current;
                if (ServoControl_ReadCalibrationPosition(axis, &current) &&
                    ServoControl_WriteCalibrationTarget(axis, current, EXCAVATOR_CALIBRATION_SPEED_MAX))
                    ServoControl_SendText("OK STOP\r\n");
                else
                {
                    calibration_ready_mask &= (uint8_t)~(1U << axis);
                    (void)BSP_STS3215_SetTorqueEnabled(position_axes[axis].id, 0U);
                    ServoControl_SendText("ERR STOP\r\n");
                }
            }
            else ServoControl_SendText("OK STOP_LOCKED\r\n");
            return;
        }
        if (token_count == 1U && strcmp(tokens[0], "WHEEL_STOP_ALL") == 0)
        {
            if (EXCAVATOR_POSITION_REMOTE_MODE) ServoControl_SuspendRemote();
            ServoControl_StopChassis();
            ServoControl_SendText("OK WHEEL_STOP_ALL\r\n");
            return;
        }
        if (strcmp(tokens[0], "WHEEL") == 0 ||
            (token_count > 1U && strcmp(tokens[0], "JOG") == 0 && strcmp(tokens[1], "3") == 0))
            ServoControl_SendText("ERR RADIO_ONLY\r\n");
        else ServoControl_SendText("ERR AXIS_LOCKED\r\n");
        return;
    }

    if (token_count == 2U && strcmp(tokens[0], "STOP") == 0 &&
        ServoControl_ParseAxis(tokens[1], &axis))
    {
        ServoControl_StopPositionAxis(axis);
        ServoControl_SendText("OK STOP\r\n");
        return;
    }

    if (token_count == 2U && strcmp(tokens[0], "CLEAR") == 0 &&
        ServoControl_ParseAxis(tokens[1], &axis))
    {
        if (!ServoControl_IsLimitableAxis(axis))
        {
            ServoControl_SendText("ERR NO_LIMIT\r\n");
            return;
        }

        if (!ServoControl_HoldForLimitChange(axis))
        {
            ServoControl_SendText("ERR OFFLINE\r\n");
            return;
        }
        position_axes[axis].minimum_position =
            g_excavator_position_axis_parameters[axis].minimum_position;
        position_axes[axis].maximum_position =
            g_excavator_position_axis_parameters[axis].maximum_position;
        position_axes[axis].limits_enabled =
            g_excavator_position_axis_parameters[axis].default_limits_enabled;
        ServoControl_SendText("OK CLEAR\r\n");
        return;
    }

    if (token_count == 4U && strcmp(tokens[0], "JOG") == 0 &&
        ServoControl_ParseAxis(tokens[1], &axis) &&
        ServoControl_ParseSigned(tokens[2], -1, 1, &signed_value) &&
        signed_value != 0 &&
        ServoControl_ParseUnsigned(tokens[3], EXCAVATOR_SERVO_SPEED_MAX, &first_value) &&
        first_value >= EXCAVATOR_SERVO_SPEED_MIN)
    {
        PositionAxisState *state = &position_axes[axis];

        if (EXCAVATOR_CALIBRATION_MODE && (axis == EXCAVATOR_AXIS_PITCH ||
            axis == EXCAVATOR_AXIS_GRIPPER) && first_value > EXCAVATOR_CALIBRATION_SPEED_MAX)
        {
            ServoControl_SendText("ERR SPEED\r\n");
            return;
        }
        if (!state->online)
        {
            ServoControl_SendText("ERR OFFLINE\r\n");
            return;
        }

        if (ServoControl_IsContinuousAxis(axis))
        {
            if (EXCAVATOR_CALIBRATION_MODE)
            {
                ServoControl_SendText("ERR CALIBRATION\r\n");
                return;
            }
            int16_t speed = (int16_t)((int16_t)signed_value * (int16_t)first_value);

            if (state->jog_direction != 0 && state->jog_direction != (int8_t)signed_value &&
                !ServoControl_WriteSwingSpeed(state, 0))
            {
                ServoControl_SendText("ERR JOG\r\n");
                return;
            }
            if (!ServoControl_WriteSwingSpeed(state, speed))
            {
                ServoControl_SendText("ERR JOG\r\n");
                return;
            }
            state->jog_direction = (int8_t)signed_value;
            state->speed = first_value;
            state->last_jog_command = xTaskGetTickCount();
            ServoControl_SendText("OK JOG\r\n");
            return;
        }

        if (!state->limits_enabled)
        {
            ServoControl_SendText("ERR UNCALIBRATED\r\n");
            return;
        }
        state->jog_direction = (int8_t)signed_value;
        state->target_position = (signed_value < 0)
                                     ? ServoControl_GetLowerLimit(state)
                                     : ServoControl_GetUpperLimit(state);
        state->speed = first_value;
        state->last_jog_command = xTaskGetTickCount();
        state->command_pending = true;
        ServoControl_SendText("OK JOG\r\n");
        return;
    }

    if (token_count == 4U && strcmp(tokens[0], "MOVE") == 0 &&
        ServoControl_ParseAxis(tokens[1], &axis) &&
        ServoControl_ParseUnsigned(tokens[2], BSP_STS3215_POSITION_MAX, &first_value) &&
        ServoControl_ParseUnsigned(tokens[3], EXCAVATOR_SERVO_SPEED_MAX, &second_value) &&
        second_value >= EXCAVATOR_SERVO_SPEED_MIN)
    {
        PositionAxisState *state = &position_axes[axis];

        if (EXCAVATOR_CALIBRATION_MODE && (axis == EXCAVATOR_AXIS_PITCH ||
            axis == EXCAVATOR_AXIS_GRIPPER) && second_value > EXCAVATOR_CALIBRATION_SPEED_MAX)
        {
            ServoControl_SendText("ERR SPEED\r\n");
            return;
        }
        if (ServoControl_IsContinuousAxis(axis))
        {
            ServoControl_SendText("ERR CONTINUOUS\r\n");
            return;
        }

        if (!state->online)
        {
            ServoControl_SendText("ERR OFFLINE\r\n");
            return;
        }
        if (first_value < ServoControl_GetLowerLimit(state) ||
            first_value > ServoControl_GetUpperLimit(state))
        {
            ServoControl_SendText("ERR RANGE\r\n");
            return;
        }

        if (!state->limits_enabled)
        {
            ServoControl_SendText("ERR UNCALIBRATED\r\n");
            return;
        }

        state->jog_direction = 0;
        state->target_position = first_value;
        state->speed = second_value;
        state->command_pending = true;
        ServoControl_SendText("OK MOVE\r\n");
        return;
    }

    if (token_count == 4U && strcmp(tokens[0], "LIMIT") == 0 &&
        ServoControl_ParseAxis(tokens[1], &axis) &&
        ServoControl_ParseUnsigned(tokens[2], BSP_STS3215_POSITION_MAX, &first_value) &&
        ServoControl_ParseUnsigned(tokens[3], BSP_STS3215_POSITION_MAX, &second_value) &&
        first_value < second_value)
    {
        PositionAxisState *state = &position_axes[axis];

        if (!ServoControl_IsLimitableAxis(axis))
        {
            ServoControl_SendText("ERR NO_LIMIT\r\n");
            return;
        }

        if (!ServoControl_HoldForLimitChange(axis))
        {
            ServoControl_SendText("ERR OFFLINE\r\n");
            return;
        }
        if (state->current_position < first_value || state->current_position > second_value)
        {
            ServoControl_SendText("ERR RANGE\r\n");
            return;
        }

        state->minimum_position = first_value;
        state->maximum_position = second_value;
        state->limits_enabled = true;
        ServoControl_SendText("OK LIMIT\r\n");
        return;
    }

    if (token_count == 3U && strcmp(tokens[0], "WHEEL") == 0 &&
        ServoControl_ParseUnsigned(tokens[1], EXCAVATOR_WHEEL_AXIS_COUNT - 1U, &first_value) &&
        ServoControl_ParseSigned(tokens[2],
                                 (int16_t)-(int16_t)EXCAVATOR_SERVO_SPEED_MAX,
                                 (int16_t)EXCAVATOR_SERVO_SPEED_MAX,
                                 &signed_value))
    {
        WheelAxisState *state = &wheel_axes[first_value];

        if (EXCAVATOR_CALIBRATION_MODE && signed_value != 0)
        {
            ServoControl_SendText("ERR CALIBRATION\r\n");
            return;
        }

        if (ServoControl_WriteWheelSpeed(state, signed_value))
        {
            state->level = (signed_value == 0) ? 0 : INT8_MIN;
            ServoControl_SendText("OK WHEEL\r\n");
        }
        else
        {
            ServoControl_SendText("ERR WHEEL\r\n");
        }
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "WHEEL_STOP_ALL") == 0)
    {
        ServoControl_SetWheelLevel(EXCAVATOR_WHEEL_LEFT, 0);
        ServoControl_SetWheelLevel(EXCAVATOR_WHEEL_RIGHT, 0);
        ServoControl_SendText("OK WHEEL_STOP_ALL\r\n");
        return;
    }

    ServoControl_SendText("ERR COMMAND\r\n");
}

static void ServoControl_ProcessSerialInput(void)
{
    static char line[SERVO_CONTROL_LINE_BUFFER_SIZE];
    static uint8_t line_length;
    static bool discard_line;
    uint8_t byte;
    uint16_t processed_bytes = 0U;

    /* Continuous diagnostic traffic must not starve the motion watchdog. */
    if (host_streaming && xTaskGetTickCount() - last_host_command > pdMS_TO_TICKS(EXCAVATOR_CHASSIS_TIMEOUT_MS))
        ServoControl_EndHost();
    while (processed_bytes < SERVO_CONTROL_RX_BUDGET && BSP_UART_DebugTryRead(&byte))
    {
        processed_bytes++;
        if (byte == '\r')
        {
            continue;
        }
        if (byte == '\n')
        {
            if (discard_line) { discard_line = false; line_length = 0U; continue; }
            if (line_length > 0U)
            {
                line[line_length] = '\0';
                ServoControl_ProcessCommand(line);
                line_length = 0U;
            }
            continue;
        }

        if (discard_line) continue;
        if (line_length < (sizeof(line) - 1U))
        {
            line[line_length++] = (char)byte;
        }
        else
        {
            line_length = 0U;
            discard_line = true;
            ServoControl_EndHost();
            ServoControl_SendText("ERR LINE\r\n");
        }
    }
}

static void ServoControl_Task(void *argument)
{
    TickType_t last_control_time;
    TickType_t last_feedback_time;
    TickType_t last_status_time;

    (void)argument;

    ServoControl_LoadParameters();
    BSP_UART_DebugInit();
    if (BSP_STS3215_Init() != BSP_STS3215_OK)
    {
        BSP_LED_On(BSP_LED_STATUS);
        ServoControl_SendText("ERR SERVO_INIT\r\n");
    }

    vTaskDelay(pdMS_TO_TICKS(100U));
    ServoControl_UpdateFeedback();
    ServoControl_SendText(EXCAVATOR_READ_ONLY_DIAGNOSTICS ? "SERVO_READ_ONLY_READY\r\n" :
                            EXCAVATOR_CHASSIS_TEST_MODE ? "SERVO_CHASSIS_ONLY_READY\r\n" : "SERVO_CALIBRATION_READY\r\n");
    ServoControl_SendStatus();

    last_control_time = xTaskGetTickCount();
    last_feedback_time = last_control_time;
    last_status_time = last_control_time;

    for (;;)
    {
        TickType_t now;

        ServoControl_ProcessSerialInput();
        ServoControl_ProcessWirelessInput();
        now = xTaskGetTickCount();

        if ((now - last_control_time) >= pdMS_TO_TICKS(EXCAVATOR_CONTROL_PERIOD_MS))
        {
            ServoControl_UpdateMotion();
            last_control_time = now;
        }
        if ((now - last_feedback_time) >= pdMS_TO_TICKS(EXCAVATOR_FEEDBACK_PERIOD_MS))
        {
            ServoControl_UpdateFeedback();
            ServoControl_UpdateStartup();
            last_feedback_time = xTaskGetTickCount();
        }
        if ((now - last_status_time) >= pdMS_TO_TICKS(EXCAVATOR_STATUS_PERIOD_MS))
        {
            ServoControl_SendStatus();
            last_status_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void ServoControl_TaskCreate(void)
{
    wireless_command_queue = xQueueCreate(1U, sizeof(ExcavatorVelocityCmd_t));
    (void)xTaskCreate(ServoControl_Task,
                      SERVO_CONTROL_TASK_NAME,
                      SERVO_CONTROL_TASK_STACK_WORDS,
                      NULL,
                      SERVO_CONTROL_TASK_PRIORITY,
                      NULL);
}

bool ServoControl_SubmitWirelessCommand(const ExcavatorVelocityCmd_t *command)
{
    if (command == NULL || wireless_command_queue == NULL)
    {
        return false;
    }

    if (EXCAVATOR_READ_ONLY_DIAGNOSTICS || (EXCAVATOR_CALIBRATION_MODE && !EXCAVATOR_CHASSIS_TEST_MODE))
    {
        return true;
    }

    if (EXCAVATOR_CHASSIS_TEST_MODE)
    {
        if (EXCAVATOR_POSITION_REMOTE_MODE) return xQueueOverwrite(wireless_command_queue, command) == pdPASS;
        ExcavatorVelocityCmd_t chassis = {.left_track = command->left_track,
                                         .right_track = command->right_track, .swing = command->swing};
        return xQueueOverwrite(wireless_command_queue, &chassis) == pdPASS;
    }
    return xQueueOverwrite(wireless_command_queue, command) == pdPASS;
}

void ServoControl_NotifyWirelessTimeout(void)
{
    wireless_timeout_pending = true;
}
