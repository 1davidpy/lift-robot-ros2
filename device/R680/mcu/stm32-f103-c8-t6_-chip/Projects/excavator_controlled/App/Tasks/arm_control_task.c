#include "arm_control_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_led.h"
#include "bsp_sts3215.h"
#include "bsp_uart.h"
#include "excavator_arm_config.h"
#include "queue.h"
#include "step_motor_task.h"
#include "step_motor_config.h"

#define ARM_CONTROL_LINE_BUFFER_SIZE  96U
#define ARM_CONTROL_TOKEN_COUNT_MAX   5U
#define ARM_CONTROL_TRACK_STOP_REPEATS 2U
#define ARM_CONTROL_TRACK_STOP_GAP_MS  25U

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
    bool position_mode_verified;
    bool limits_enabled;
    bool online;
} ArmControlAxis;

static ArmControlAxis arm_axes[EXCAVATOR_ARM_AXIS_COUNT];

static QueueHandle_t arm_wireless_command_queue;
static int8_t arm_step_levels[EXCAVATOR_STEP_MOTOR_COUNT];
static uint8_t track_stop_repeats[2];
static TickType_t track_stop_next_time[2];

static uint16_t ArmControl_GetLowerLimit(const ArmControlAxis *state);
static uint16_t ArmControl_GetUpperLimit(const ArmControlAxis *state);

static void ArmControl_ScheduleTrackStop(uint8_t motor_index)
{
    if (motor_index < 2U)
    {
        track_stop_repeats[motor_index] = ARM_CONTROL_TRACK_STOP_REPEATS;
        track_stop_next_time[motor_index] =
            xTaskGetTickCount() + pdMS_TO_TICKS(ARM_CONTROL_TRACK_STOP_GAP_MS);
    }
}

static void ArmControl_ReinforceTrackStops(TickType_t now)
{
    static const uint8_t addresses[2] = {
        EXCAVATOR_LEFT_TRACK_MOTOR_ADDRESS,
        EXCAVATOR_RIGHT_TRACK_MOTOR_ADDRESS,
    };

    for (uint8_t motor = 0U; motor < 2U; motor++)
    {
        if (track_stop_repeats[motor] == 0U ||
            (int32_t)(now - track_stop_next_time[motor]) < 0)
        {
            continue;
        }

        if (StepMotor_Stop(addresses[motor]) == STEP_MOTOR_CONTROL_OK)
        {
            track_stop_repeats[motor]--;
        }
        track_stop_next_time[motor] =
            now + pdMS_TO_TICKS(ARM_CONTROL_TRACK_STOP_GAP_MS);
    }
}

static void ArmControl_LoadAxisParameters(void)
{
    for (uint8_t i = 0U; i < EXCAVATOR_ARM_AXIS_COUNT; i++)
    {
        arm_axes[i].id = g_excavator_arm_axis_parameters[i].servo_id;
        arm_axes[i].minimum_position =
            g_excavator_arm_axis_parameters[i].minimum_position;
        arm_axes[i].maximum_position =
            g_excavator_arm_axis_parameters[i].maximum_position;
        arm_axes[i].speed = EXCAVATOR_ARM_DEFAULT_SPEED;
        arm_axes[i].limits_enabled = true;
    }
}

static void ArmControl_SendText(const char *text)
{
    if (text != NULL)
    {
        (void)BSP_UART_DebugWrite((const uint8_t *)text, (uint16_t)strlen(text));
    }
}

static void ArmControl_SendStatus(void)
{
    char message[160];
    uint32_t online_mask = 0U;
    uint32_t limit_mask = 0U;
    int length;

    for (uint8_t i = 0U; i < EXCAVATOR_ARM_AXIS_COUNT; i++)
    {
        if (arm_axes[i].online)
        {
            online_mask |= (uint32_t)(1UL << i);
        }

        if (arm_axes[i].limits_enabled)
        {
            limit_mask |= (uint32_t)(1UL << i);
        }
    }

    length = snprintf(message,
                      sizeof(message),
                      "STATUS %lu %lu %u %u %u %u %u %u %u %u %u\r\n",
                      (unsigned long)online_mask,
                      (unsigned long)limit_mask,
                      (unsigned int)arm_axes[0].current_position,
                      (unsigned int)arm_axes[1].current_position,
                      (unsigned int)arm_axes[2].current_position,
                      (unsigned int)arm_axes[0].minimum_position,
                      (unsigned int)arm_axes[0].maximum_position,
                      (unsigned int)arm_axes[1].minimum_position,
                      (unsigned int)arm_axes[1].maximum_position,
                      (unsigned int)arm_axes[2].minimum_position,
                      (unsigned int)arm_axes[2].maximum_position);

    if (length > 0 && (size_t)length < sizeof(message))
    {
        (void)BSP_UART_DebugWrite((const uint8_t *)message, (uint16_t)length);
    }
}

static bool ArmControl_ParseUnsigned(const char *text, uint32_t maximum, uint16_t *value)
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

static bool ArmControl_ParseDirection(const char *text, int8_t *direction)
{
    char *end;
    long parsed;

    if (text == NULL || direction == NULL)
    {
        return false;
    }

    parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || (parsed != -1L && parsed != 1L))
    {
        return false;
    }

    *direction = (int8_t)parsed;
    return true;
}

static bool ArmControl_ParseSigned(const char *text,
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

static uint8_t ArmControl_Tokenize(char *line, char *tokens[ARM_CONTROL_TOKEN_COUNT_MAX])
{
    uint8_t count = 0U;
    char *token = strtok(line, " ");

    while (token != NULL && count < ARM_CONTROL_TOKEN_COUNT_MAX)
    {
        tokens[count++] = token;
        token = strtok(NULL, " ");
    }

    return count;
}

static bool ArmControl_ParseAxis(const char *text, uint8_t *axis)
{
    uint16_t parsed_axis;

    if (!ArmControl_ParseUnsigned(text, EXCAVATOR_ARM_AXIS_COUNT - 1U, &parsed_axis))
    {
        return false;
    }

    *axis = (uint8_t)parsed_axis;
    return true;
}

static void ArmControl_StopAxis(uint8_t axis)
{
    ArmControlAxis *state = &arm_axes[axis];
    uint16_t position;

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

static void ArmControl_CancelAxisCommand(uint8_t axis)
{
    ArmControlAxis *state = &arm_axes[axis];

    state->jog_direction = 0;
    state->target_position = state->current_position;
    state->command_pending = false;
}

static bool ArmControl_EnsurePositionMode(ArmControlAxis *state)
{
    uint8_t mode;

    if (state->position_mode_verified)
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
        if (BSP_STS3215_SetOperatingMode(state->id, BSP_STS3215_MODE_POSITION) != BSP_STS3215_OK ||
            BSP_STS3215_WriteGoalSpeed(state->id, 0U) != BSP_STS3215_OK ||
            BSP_STS3215_ReadOperatingMode(state->id, &mode) != BSP_STS3215_OK ||
            mode != BSP_STS3215_MODE_POSITION)
        {
            return false;
        }
    }

    state->position_mode_verified = true;
    return true;
}

static void ArmControl_ProcessCommand(char *line)
{
    char *tokens[ARM_CONTROL_TOKEN_COUNT_MAX] = {0};
    uint8_t token_count = ArmControl_Tokenize(line, tokens);
    uint8_t axis;
    uint16_t first_value;
    uint16_t second_value;
    uint16_t third_value;
    int32_t step_position;
    int16_t step_speed;
    int16_t signed_value;
    int8_t direction;

    if (token_count == 1U && strcmp(tokens[0], "PING") == 0)
    {
        ArmControl_SendText("PONG\r\n");
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "STATUS") == 0)
    {
        ArmControl_SendStatus();
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "STEP_POSITION") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value))
    {
        char message[48];
        StepMotorControlResult result =
            StepMotor_GetPosition((uint8_t)first_value, &step_position);

        if (result == STEP_MOTOR_CONTROL_OK)
        {
            int length = snprintf(message,
                                  sizeof(message),
                                  "STEP_POSITION %u %ld\r\n",
                                  first_value,
                                  (long)step_position);
            if (length > 0 && (size_t)length < sizeof(message))
            {
                ArmControl_SendText(message);
            }
        }
        else if (result == STEP_MOTOR_CONTROL_NOT_READY)
        {
            ArmControl_SendText("ERR STEP_NOT_READY\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_INVALID_ARGUMENT)
        {
            ArmControl_SendText("ERR STEP_ARGUMENT\r\n");
        }
        else
        {
            ArmControl_SendText("ERR STEP_CAN\r\n");
        }
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "STEP_CENTER_STATUS") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value))
    {
        if (first_value != EXCAVATOR_SWING_MOTOR_ADDRESS)
        {
            ArmControl_SendText("ERR STEP_ARGUMENT\r\n");
        }
        else
        {
            ArmControl_SendText(StepMotor_IsCenterSet((uint8_t)first_value)
                                    ? "STEP_CENTER_STATUS 3 1\r\n"
                                    : "STEP_CENTER_STATUS 3 0\r\n");
        }
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "STEP_SPEED") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value))
    {
        char message[40];
        StepMotorControlResult result =
            StepMotor_GetSpeed((uint8_t)first_value, &step_speed);

        if (result == STEP_MOTOR_CONTROL_OK)
        {
            int length = snprintf(message,
                                  sizeof(message),
                                  "STEP_SPEED %u %d\r\n",
                                  first_value,
                                  (int)step_speed);
            if (length > 0 && (size_t)length < sizeof(message))
            {
                ArmControl_SendText(message);
            }
        }
        else if (result == STEP_MOTOR_CONTROL_NOT_READY)
        {
            ArmControl_SendText("ERR STEP_NOT_READY\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_INVALID_ARGUMENT)
        {
            ArmControl_SendText("ERR STEP_ARGUMENT\r\n");
        }
        else
        {
            ArmControl_SendText("ERR STEP_CAN\r\n");
        }
        return;
    }

    if (token_count == 4U &&
        strcmp(tokens[0], "STEP_RUN") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value) &&
        ArmControl_ParseUnsigned(tokens[2], 1U, &second_value) &&
        ArmControl_ParseUnsigned(tokens[3], STEP_MOTOR_SPEED_RPM_MAX, &third_value) &&
        third_value > 0U)
    {
        StepMotorControlResult result = StepMotor_Run((uint8_t)first_value,
                                                       (uint8_t)second_value,
                                                       third_value);

        if (result == STEP_MOTOR_CONTROL_OK)
        {
            ArmControl_SendText("OK STEP_RUN\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_NOT_READY)
        {
            ArmControl_SendText("ERR STEP_NOT_READY\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_INVALID_ARGUMENT)
        {
            ArmControl_SendText("ERR STEP_ARGUMENT\r\n");
        }
        else
        {
            ArmControl_SendText("ERR STEP_CAN\r\n");
        }
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "STEP_STOP") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value))
    {
        StepMotorControlResult result = StepMotor_Stop((uint8_t)first_value);

        if (result == STEP_MOTOR_CONTROL_OK &&
            first_value >= EXCAVATOR_STEP_MOTOR_1_ADDRESS &&
            first_value <= EXCAVATOR_STEP_MOTOR_3_ADDRESS)
        {
            arm_step_levels[first_value - EXCAVATOR_STEP_MOTOR_1_ADDRESS] = 0;
            ArmControl_ScheduleTrackStop(
                (uint8_t)(first_value - EXCAVATOR_STEP_MOTOR_1_ADDRESS));
        }

        ArmControl_SendText((result == STEP_MOTOR_CONTROL_OK) ? "OK STEP_STOP\r\n"
                                                               : "ERR STEP_STOP\r\n");
        return;
    }

    if (token_count == 1U && strcmp(tokens[0], "STEP_STOP_ALL") == 0)
    {
        StepMotorControlResult result = StepMotor_StopAll();

        if (result == STEP_MOTOR_CONTROL_OK)
        {
            memset(arm_step_levels, 0, sizeof(arm_step_levels));
            ArmControl_ScheduleTrackStop(0U);
            ArmControl_ScheduleTrackStop(1U);
        }

        ArmControl_SendText((result == STEP_MOTOR_CONTROL_OK) ? "OK STEP_STOP_ALL\r\n"
                                                               : "ERR STEP_STOP_ALL\r\n");
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "STEP_CENTER") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value))
    {
        StepMotorControlResult result = StepMotor_SetCenter((uint8_t)first_value);

        if (result == STEP_MOTOR_CONTROL_OK)
        {
            arm_step_levels[2] = 0;
            ArmControl_SendText("OK STEP_CENTER\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_NOT_READY)
        {
            ArmControl_SendText("ERR STEP_NOT_READY\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_INVALID_ARGUMENT)
        {
            ArmControl_SendText("ERR STEP_ARGUMENT\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_STORAGE_ERROR)
        {
            ArmControl_SendText("ERR STEP_STORAGE\r\n");
        }
        else
        {
            ArmControl_SendText("ERR STEP_CAN\r\n");
        }
        return;
    }

    if (token_count == 4U &&
        strcmp(tokens[0], "STEP_ANGLE") == 0 &&
        ArmControl_ParseUnsigned(tokens[1], 255U, &first_value) &&
        ArmControl_ParseSigned(tokens[2],
                               -STEP_MOTOR_CENTER_ANGLE_MAX,
                               STEP_MOTOR_CENTER_ANGLE_MAX,
                               &signed_value) &&
        ArmControl_ParseUnsigned(tokens[3], STEP_MOTOR_SPEED_RPM_MAX, &second_value) &&
        second_value > 0U)
    {
        StepMotorControlResult result = StepMotor_MoveCentered((uint8_t)first_value,
                                                                signed_value);

        if (result == STEP_MOTOR_CONTROL_OK)
        {
            arm_step_levels[2] = 0;
            ArmControl_SendText("OK STEP_ANGLE\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_CENTER_REQUIRED)
        {
            ArmControl_SendText("ERR STEP_CENTER_REQUIRED\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_NOT_READY)
        {
            ArmControl_SendText("ERR STEP_NOT_READY\r\n");
        }
        else if (result == STEP_MOTOR_CONTROL_INVALID_ARGUMENT)
        {
            ArmControl_SendText("ERR STEP_ARGUMENT\r\n");
        }
        else
        {
            ArmControl_SendText("ERR STEP_CAN\r\n");
        }
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "STOP") == 0 &&
        ArmControl_ParseAxis(tokens[1], &axis))
    {
        ArmControl_StopAxis(axis);
        ArmControl_SendText("OK STOP\r\n");
        return;
    }

    if (token_count == 2U &&
        strcmp(tokens[0], "CLEAR") == 0 &&
        ArmControl_ParseAxis(tokens[1], &axis))
    {
        ArmControl_CancelAxisCommand(axis);
        arm_axes[axis].limits_enabled = true;
        arm_axes[axis].minimum_position =
            g_excavator_arm_axis_parameters[axis].minimum_position;
        arm_axes[axis].maximum_position =
            g_excavator_arm_axis_parameters[axis].maximum_position;
        ArmControl_SendText("OK CLEAR\r\n");
        return;
    }

    if (token_count == 4U &&
        strcmp(tokens[0], "JOG") == 0 &&
        ArmControl_ParseAxis(tokens[1], &axis) &&
        ArmControl_ParseDirection(tokens[2], &direction) &&
        ArmControl_ParseUnsigned(tokens[3], EXCAVATOR_ARM_SPEED_MAX, &first_value) &&
        first_value >= EXCAVATOR_ARM_SPEED_MIN)
    {
        ArmControlAxis *state = &arm_axes[axis];
        uint16_t lower_limit;
        uint16_t upper_limit;

        if (!state->online)
        {
            ArmControl_SendText("ERR OFFLINE\r\n");
            return;
        }

        lower_limit = ArmControl_GetLowerLimit(state);
        upper_limit = ArmControl_GetUpperLimit(state);
        if (state->jog_direction != direction)
        {
            if ((direction < 0 && state->current_position <= lower_limit) ||
                (direction > 0 && state->current_position >= upper_limit))
            {
                state->jog_direction = 0;
                ArmControl_SendText("ERR RANGE\r\n");
                return;
            }

            state->target_position = (direction < 0) ? lower_limit : upper_limit;
            state->command_pending = true;
        }

        if (state->speed != first_value)
        {
            state->command_pending = true;
        }
        state->jog_direction = direction;
        state->speed = first_value;
        state->last_jog_command = xTaskGetTickCount();
        ArmControl_SendText("OK JOG\r\n");
        return;
    }

    if (token_count == 4U &&
        strcmp(tokens[0], "MOVE") == 0 &&
        ArmControl_ParseAxis(tokens[1], &axis) &&
        ArmControl_ParseUnsigned(tokens[2], BSP_STS3215_POSITION_MAX, &first_value) &&
        ArmControl_ParseUnsigned(tokens[3], EXCAVATOR_ARM_SPEED_MAX, &second_value) &&
        second_value >= EXCAVATOR_ARM_SPEED_MIN)
    {
        ArmControlAxis *state = &arm_axes[axis];

        if (!state->online)
        {
            ArmControl_SendText("ERR OFFLINE\r\n");
            return;
        }

        if (first_value < ArmControl_GetLowerLimit(state) ||
            first_value > ArmControl_GetUpperLimit(state))
        {
            ArmControl_SendText("ERR RANGE\r\n");
            return;
        }

        state->jog_direction = 0;
        state->target_position = first_value;
        state->speed = second_value;
        state->command_pending = true;
        ArmControl_SendText("OK MOVE\r\n");
        return;
    }

    if (token_count == 4U &&
        strcmp(tokens[0], "LIMIT") == 0 &&
        ArmControl_ParseAxis(tokens[1], &axis) &&
        ArmControl_ParseUnsigned(tokens[2], BSP_STS3215_POSITION_MAX, &first_value) &&
        ArmControl_ParseUnsigned(tokens[3], BSP_STS3215_POSITION_MAX, &second_value) &&
        first_value < second_value)
    {
        ArmControlAxis *state = &arm_axes[axis];

        if (!state->online)
        {
            ArmControl_SendText("ERR OFFLINE\r\n");
            return;
        }

        if (first_value < g_excavator_arm_axis_parameters[axis].minimum_position ||
            second_value > g_excavator_arm_axis_parameters[axis].maximum_position)
        {
            ArmControl_SendText("ERR RANGE\r\n");
            return;
        }

        if (state->current_position < first_value || state->current_position > second_value)
        {
            ArmControl_SendText("ERR RANGE\r\n");
            return;
        }

        ArmControl_CancelAxisCommand(axis);
        state->minimum_position = first_value;
        state->maximum_position = second_value;
        state->limits_enabled = true;
        ArmControl_SendText("OK LIMIT\r\n");
        return;
    }

    ArmControl_SendText("ERR COMMAND\r\n");
}

static void ArmControl_UpdateFeedback(void)
{
    for (uint8_t i = 0U; i < EXCAVATOR_ARM_AXIS_COUNT; i++)
    {
        ArmControlAxis *state = &arm_axes[i];
        uint16_t position;

        if (ArmControl_EnsurePositionMode(state) &&
            BSP_STS3215_ReadPresentPosition(state->id, &position) == BSP_STS3215_OK)
        {
            if (!state->online)
            {
                state->target_position = position;
                state->jog_direction = 0;
                state->command_pending = false;
            }

            state->current_position = position;
            state->online = true;
        }
        else
        {
            state->online = false;
            state->position_mode_verified = false;
            state->jog_direction = 0;
            state->command_pending = false;
        }
    }
}

static uint16_t ArmControl_GetLowerLimit(const ArmControlAxis *state)
{
    return state->limits_enabled ? state->minimum_position : BSP_STS3215_POSITION_MIN;
}

static uint16_t ArmControl_GetUpperLimit(const ArmControlAxis *state)
{
    return state->limits_enabled ? state->maximum_position : BSP_STS3215_POSITION_MAX;
}

static uint8_t ArmControl_OppositeMotorDirection(uint8_t direction)
{
    return (direction == BSP_PD42S1_DIRECTION_CW) ? BSP_PD42S1_DIRECTION_CCW
                                                  : BSP_PD42S1_DIRECTION_CW;
}

static void ArmControl_SendStepState(void)
{
    char message[48];
    int length = snprintf(message,
                          sizeof(message),
                          "STEP_STATE %d %d %d\r\n",
                          (int)arm_step_levels[0],
                          (int)arm_step_levels[1],
                          (int)arm_step_levels[2]);

    if (length > 0 && (size_t)length < sizeof(message))
    {
        (void)BSP_UART_DebugWrite((const uint8_t *)message, (uint16_t)length);
    }
}

static void ArmControl_ApplyStepLevel(uint8_t motor_index,
                                      uint8_t address,
                                      uint8_t positive_direction,
                                      uint16_t maximum_speed_rpm,
                                      uint8_t maximum_level,
                                      int8_t level)
{
    StepMotorControlResult result;
    uint8_t magnitude;
    uint8_t direction;
    uint16_t speed_rpm;

    if (arm_step_levels[motor_index] == level)
    {
        return;
    }

    if (motor_index < 2U && level != 0)
    {
        track_stop_repeats[motor_index] = 0U;
    }

    if (level == 0)
    {
        result = StepMotor_Stop(address);
    }
    else
    {
        magnitude = (uint8_t)((level < 0) ? -level : level);
        direction = (level > 0) ? positive_direction
                                : ArmControl_OppositeMotorDirection(positive_direction);
        speed_rpm = (uint16_t)(((uint32_t)maximum_speed_rpm * magnitude) / maximum_level);
        result = StepMotor_Run(address, direction, speed_rpm);
    }

    if (result == STEP_MOTOR_CONTROL_OK)
    {
        arm_step_levels[motor_index] = level;
        if (motor_index < 2U && level == 0)
        {
            ArmControl_ScheduleTrackStop(motor_index);
        }
        ArmControl_SendStepState();
    }
}

static uint16_t ArmControl_GetWirelessServoSpeed(int8_t level)
{
    (void)level;
    return EXCAVATOR_ARM_WIRELESS_SPEED;
}

static void ArmControl_ApplySwingLevel(int8_t level)
{
    StepMotorControlResult result;
    int16_t target_angle;

    if (arm_step_levels[2] == level)
    {
        return;
    }

    if (level == 0)
    {
        result = StepMotor_Stop(EXCAVATOR_SWING_MOTOR_ADDRESS);
    }
    else
    {
        target_angle = (level > 0) ? STEP_MOTOR_CENTER_ANGLE_MAX
                                   : -STEP_MOTOR_CENTER_ANGLE_MAX;
        result = StepMotor_MoveCentered(EXCAVATOR_SWING_MOTOR_ADDRESS,
                                        target_angle);
    }

    if (result == STEP_MOTOR_CONTROL_OK)
    {
        arm_step_levels[2] = level;
        ArmControl_SendStepState();
    }
}

static void ArmControl_ApplyServoLevel(uint8_t axis,
                                       int8_t positive_direction,
                                       int8_t level,
                                       TickType_t now)
{
    ArmControlAxis *state = &arm_axes[axis];
    int8_t direction;

    if (level == 0)
    {
        if (state->jog_direction != 0)
        {
            ArmControl_StopAxis(axis);
        }
        return;
    }

    if (!state->online)
    {
        return;
    }

    direction = (level > 0) ? positive_direction : (int8_t)-positive_direction;
    if (state->jog_direction != direction)
    {
        state->target_position = (direction < 0) ? ArmControl_GetLowerLimit(state)
                                                 : ArmControl_GetUpperLimit(state);
        state->command_pending = true;
    }

    if (state->speed != ArmControl_GetWirelessServoSpeed(level))
    {
        state->command_pending = true;
    }
    state->jog_direction = direction;
    state->speed = ArmControl_GetWirelessServoSpeed(level);
    state->last_jog_command = now;
}

static void ArmControl_ApplyWirelessCommand(const ExcavatorVelocityCmd_t *command)
{
    TickType_t now = xTaskGetTickCount();

    ArmControl_ApplyStepLevel(0U,
                              EXCAVATOR_LEFT_TRACK_MOTOR_ADDRESS,
                              EXCAVATOR_LEFT_TRACK_FORWARD_DIR,
                              EXCAVATOR_TRACK_SPEED_RPM_MAX,
                              EXCAVATOR_TRACK_LEVEL_MAX,
                              command->left_track);
    ArmControl_ApplyStepLevel(1U,
                              EXCAVATOR_RIGHT_TRACK_MOTOR_ADDRESS,
                              EXCAVATOR_RIGHT_TRACK_FORWARD_DIR,
                              EXCAVATOR_TRACK_SPEED_RPM_MAX,
                              EXCAVATOR_TRACK_LEVEL_MAX,
                              command->right_track);
    ArmControl_ApplySwingLevel(command->swing);

    ArmControl_ApplyServoLevel(EXCAVATOR_ARM_AXIS_BOOM,
                               g_excavator_arm_axis_parameters[EXCAVATOR_ARM_AXIS_BOOM]
                                   .positive_direction,
                               command->boom,
                               now);
    ArmControl_ApplyServoLevel(EXCAVATOR_ARM_AXIS_ARM,
                               g_excavator_arm_axis_parameters[EXCAVATOR_ARM_AXIS_ARM]
                                   .positive_direction,
                               command->arm,
                               now);
    ArmControl_ApplyServoLevel(EXCAVATOR_ARM_AXIS_BUCKET,
                               g_excavator_arm_axis_parameters[EXCAVATOR_ARM_AXIS_BUCKET]
                                   .positive_direction,
                               command->bucket,
                               now);
}

static void ArmControl_ProcessWirelessInput(void)
{
    ExcavatorVelocityCmd_t command;

    if (arm_wireless_command_queue != NULL &&
        xQueueReceive(arm_wireless_command_queue, &command, 0U) == pdPASS)
    {
        ArmControl_ApplyWirelessCommand(&command);
    }
}

static void ArmControl_UpdateMotion(void)
{
    BspSts3215PositionCommand commands[EXCAVATOR_ARM_AXIS_COUNT];
    uint8_t command_axes[EXCAVATOR_ARM_AXIS_COUNT];
    TickType_t now = xTaskGetTickCount();
    uint8_t command_count = 0U;

    for (uint8_t i = 0U; i < EXCAVATOR_ARM_AXIS_COUNT; i++)
    {
        ArmControlAxis *state = &arm_axes[i];

        if (state->jog_direction != 0 &&
            (now - state->last_jog_command) > pdMS_TO_TICKS(EXCAVATOR_ARM_JOG_TIMEOUT_MS))
        {
            ArmControl_StopAxis(i);
        }
        if (state->command_pending && state->online)
        {
            commands[command_count].id = state->id;
            commands[command_count].position = state->target_position;
            commands[command_count].speed = state->speed;
            commands[command_count].acceleration = EXCAVATOR_ARM_DEFAULT_ACCELERATION;
            command_axes[command_count] = i;
            command_count++;
        }
    }

    if (command_count > 0U)
    {
        (void)BSP_STS3215_SyncWritePosition(commands, command_count);
    }

    for (uint8_t i = 0U; i < command_count; i++)
    {
        arm_axes[command_axes[i]].command_pending = false;
    }
}

static void ArmControl_ProcessSerialInput(void)
{
    static char line[ARM_CONTROL_LINE_BUFFER_SIZE];
    static uint8_t line_length;
    uint8_t byte;

    while (BSP_UART_DebugTryRead(&byte))
    {
        if (byte == '\r')
        {
            continue;
        }

        if (byte == '\n')
        {
            if (line_length > 0U)
            {
                line[line_length] = '\0';
                ArmControl_ProcessCommand(line);
                line_length = 0U;
            }
            continue;
        }

        if (line_length < (sizeof(line) - 1U))
        {
            line[line_length++] = (char)byte;
        }
        else
        {
            line_length = 0U;
            ArmControl_SendText("ERR LINE\r\n");
        }
    }
}

static void ArmControl_Task(void *argument)
{
    TickType_t last_control_time;
    TickType_t last_feedback_time;
    TickType_t last_status_time;

    (void)argument;

    ArmControl_LoadAxisParameters();
    BSP_UART_DebugInit();
    if (BSP_STS3215_Init() != BSP_STS3215_OK)
    {
        BSP_LED_On(BSP_LED_STATUS);
        ArmControl_SendText("ERR SERVO_INIT\r\n");
    }

    vTaskDelay(pdMS_TO_TICKS(100U));
    ArmControl_UpdateFeedback();
    ArmControl_SendText("ARM_READY\r\n");
    ArmControl_SendStatus();

    last_control_time = xTaskGetTickCount();
    last_feedback_time = last_control_time;
    last_status_time = last_control_time;

    for (;;)
    {
        TickType_t now;

        ArmControl_ProcessSerialInput();
        ArmControl_ProcessWirelessInput();
        StepMotor_ProcessCenteredMotion();
        now = xTaskGetTickCount();
        ArmControl_ReinforceTrackStops(now);

        if ((now - last_control_time) >= pdMS_TO_TICKS(EXCAVATOR_ARM_CONTROL_PERIOD_MS))
        {
            ArmControl_UpdateMotion();
            last_control_time = now;
        }

        if ((now - last_feedback_time) >= pdMS_TO_TICKS(EXCAVATOR_ARM_FEEDBACK_PERIOD_MS))
        {
            ArmControl_UpdateFeedback();
            last_feedback_time = now;
        }

        if ((now - last_status_time) >= pdMS_TO_TICKS(EXCAVATOR_ARM_STATUS_PERIOD_MS))
        {
            ArmControl_SendStatus();
            last_status_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void ArmControl_TaskCreate(void)
{
    arm_wireless_command_queue = xQueueCreate(1U, sizeof(ExcavatorVelocityCmd_t));
    (void)xTaskCreate(ArmControl_Task,
                      ARM_CONTROL_TASK_NAME,
                      ARM_CONTROL_TASK_STACK_WORDS,
                      NULL,
                      ARM_CONTROL_TASK_PRIORITY,
                      NULL);
}

bool ArmControl_SubmitWirelessCommand(const ExcavatorVelocityCmd_t *command)
{
    if (command == NULL || arm_wireless_command_queue == NULL)
    {
        return false;
    }

    return xQueueOverwrite(arm_wireless_command_queue, command) == pdPASS;
}
