#include "step_motor_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_pd42s1.h"
#include "bsp_uart.h"
#include "step_motor_config.h"
#include "swing_center_storage.h"

static const uint8_t step_motor_addresses[EXCAVATOR_STEP_MOTOR_COUNT] = {
    EXCAVATOR_STEP_MOTOR_1_ADDRESS,
    EXCAVATOR_STEP_MOTOR_2_ADDRESS,
    EXCAVATOR_STEP_MOTOR_3_ADDRESS,
};
static volatile bool step_motor_ready;
static volatile bool swing_motor_center_set;
static int32_t swing_motor_center_position;

typedef struct
{
    int32_t target_position;
    int32_t previous_error;
    int32_t best_absolute_error;
    TickType_t last_poll_time;
    uint8_t read_failures;
    bool active;
} StepMotorCenteredMotion;

static StepMotorCenteredMotion swing_motion;

static bool StepMotor_IsConfiguredAddress(uint8_t address)
{
    for (uint8_t i = 0U; i < EXCAVATOR_STEP_MOTOR_COUNT; i++)
    {
        if (step_motor_addresses[i] == address)
        {
            return true;
        }
    }

    return false;
}

static void StepMotor_SendText(const char *text)
{
    (void)BSP_UART_DebugWrite((const uint8_t *)text, (uint16_t)strlen(text));
}

static BspPd42s1Result StepMotor_HaltSwing(void)
{
    return BSP_PD42S1_SetSpeed(EXCAVATOR_SWING_MOTOR_ADDRESS,
                               BSP_PD42S1_DIRECTION_CW,
                               0U,
                               0.0F);
}

static void StepMotor_MarkUnavailable(void)
{
    swing_motion.active = false;
    swing_motor_center_set = false;
    step_motor_ready = false;
}

static int32_t StepMotor_Absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

static bool StepMotor_CaptureSwingCenter(bool persist)
{
    int32_t position;

    if (BSP_PD42S1_ReadPosition(EXCAVATOR_SWING_MOTOR_ADDRESS,
                                &position) != BSP_PD42S1_OK)
    {
        return false;
    }

    swing_motor_center_position = position;
    swing_motor_center_set = true;
    swing_motion.active = false;
    return !persist || SwingCenterStorage_Save(position);
}

static bool StepMotor_LoadSwingCenter(void)
{
    if (SwingCenterStorage_Load(&swing_motor_center_position))
    {
        swing_motor_center_set = true;
        swing_motion.active = false;
        return true;
    }

    return StepMotor_CaptureSwingCenter(true);
}

static StepMotorControlResult StepMotor_FromDriverResult(BspPd42s1Result result)
{
    if (result == BSP_PD42S1_OK)
    {
        return STEP_MOTOR_CONTROL_OK;
    }

    StepMotor_MarkUnavailable();
    return STEP_MOTOR_CONTROL_DRIVER_ERROR;
}

static bool StepMotor_TryInitialize(void)
{
    if (BSP_PD42S1_Init() != BSP_PD42S1_OK ||
        BSP_PD42S1_SetSpeed(BSP_PD42S1_BROADCAST_ADDRESS,
                            EXCAVATOR_STEP_MOTOR_DIRECTION,
                            EXCAVATOR_STEP_MOTOR_ACCELERATION,
                            0.0F) != BSP_PD42S1_OK)
    {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100U));
    for (uint8_t i = 0U; i < EXCAVATOR_STEP_MOTOR_COUNT; i++)
    {
        if (BSP_PD42S1_ClearState(step_motor_addresses[i]) != BSP_PD42S1_OK)
        {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }

    if (!StepMotor_LoadSwingCenter())
    {
        return false;
    }

    return true;
}

static void StepMotor_Task(void *argument)
{
    bool failure_reported = false;

    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(100U));
    for (;;)
    {
        if (!step_motor_ready)
        {
            if (StepMotor_TryInitialize())
            {
                step_motor_ready = true;
                failure_reported = false;
                StepMotor_SendText("STEP_MOTORS_READY\r\n");
                StepMotor_SendText("STEP_SWING_CENTER_STORED\r\n");
                if (StepMotor_MoveCentered(EXCAVATOR_SWING_MOTOR_ADDRESS, 0) ==
                    STEP_MOTOR_CONTROL_OK)
                {
                    StepMotor_SendText("STEP_SWING_HOMING\r\n");
                }
            }
            else if (!failure_reported)
            {
                failure_reported = true;
                StepMotor_SendText("STEP_MOTORS_WAITING\r\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

StepMotorControlResult StepMotor_Run(uint8_t address, uint8_t direction, uint16_t speed_rpm)
{
    float commanded_speed_rpm;

    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    if (!StepMotor_IsConfiguredAddress(address) || direction > BSP_PD42S1_DIRECTION_CCW ||
        speed_rpm == 0U || speed_rpm > STEP_MOTOR_SPEED_RPM_MAX)
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }

    if (address == EXCAVATOR_SWING_MOTOR_ADDRESS)
    {
        swing_motion.active = false;
        commanded_speed_rpm = EXCAVATOR_SWING_SPEED_RPM;
    }
    else
    {
        commanded_speed_rpm = (float)speed_rpm;
    }

    return StepMotor_FromDriverResult(
        BSP_PD42S1_SetSpeed(address,
                            direction,
                            (address == EXCAVATOR_SWING_MOTOR_ADDRESS)
                                ? EXCAVATOR_SWING_ACCELERATION
                                : EXCAVATOR_STEP_MOTOR_ACCELERATION,
                            commanded_speed_rpm));
}

StepMotorControlResult StepMotor_Stop(uint8_t address)
{
    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    if (!StepMotor_IsConfiguredAddress(address))
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }

    if (address == EXCAVATOR_SWING_MOTOR_ADDRESS)
    {
        swing_motion.active = false;
        return StepMotor_FromDriverResult(StepMotor_HaltSwing());
    }

    return StepMotor_FromDriverResult(
        BSP_PD42S1_SetSpeed(address,
                            BSP_PD42S1_DIRECTION_CW,
                            EXCAVATOR_STEP_MOTOR_ACCELERATION,
                            0.0F));
}

StepMotorControlResult StepMotor_StopAll(void)
{
    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    swing_motion.active = false;
    return StepMotor_FromDriverResult(
        BSP_PD42S1_SetSpeed(BSP_PD42S1_BROADCAST_ADDRESS,
                            BSP_PD42S1_DIRECTION_CW,
                            EXCAVATOR_STEP_MOTOR_ACCELERATION,
                            0.0F));
}

StepMotorControlResult StepMotor_SetCenter(uint8_t address)
{
    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    if (address != EXCAVATOR_SWING_MOTOR_ADDRESS)
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }

    swing_motion.active = false;
    if (StepMotor_Stop(address) != STEP_MOTOR_CONTROL_OK)
    {
        StepMotor_MarkUnavailable();
        return STEP_MOTOR_CONTROL_DRIVER_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(100U));
    if (!StepMotor_CaptureSwingCenter(true))
    {
        return STEP_MOTOR_CONTROL_STORAGE_ERROR;
    }

    return STEP_MOTOR_CONTROL_OK;
}

StepMotorControlResult StepMotor_MoveCentered(uint8_t address,
                                              int16_t angle_degrees)
{
    uint8_t direction;
    int32_t current_position;
    int32_t target_position;
    int32_t error;
    int64_t target_position_wide;

    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    if (address != EXCAVATOR_SWING_MOTOR_ADDRESS ||
        angle_degrees < -STEP_MOTOR_CENTER_ANGLE_MAX ||
        angle_degrees > STEP_MOTOR_CENTER_ANGLE_MAX)
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }

    if (!swing_motor_center_set)
    {
        return STEP_MOTOR_CONTROL_CENTER_REQUIRED;
    }

    target_position_wide =
        (int64_t)swing_motor_center_position +
        ((int64_t)angle_degrees * (int64_t)BSP_PD42S1_PULSES_PER_TURN) / 360;
    if (EXCAVATOR_SWING_POSITIVE_DIR == BSP_PD42S1_DIRECTION_CCW)
    {
        target_position_wide =
            (int64_t)swing_motor_center_position -
            ((int64_t)angle_degrees * (int64_t)BSP_PD42S1_PULSES_PER_TURN) / 360;
    }
    if (target_position_wide < INT32_MIN || target_position_wide > INT32_MAX)
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }
    target_position = (int32_t)target_position_wide;

    if (BSP_PD42S1_ReadPosition(address, &current_position) != BSP_PD42S1_OK)
    {
        StepMotor_MarkUnavailable();
        return STEP_MOTOR_CONTROL_DRIVER_ERROR;
    }

    error = target_position - current_position;
    if ((int64_t)error <= (int64_t)EXCAVATOR_SWING_POSITION_TOLERANCE &&
        (int64_t)error >= -(int64_t)EXCAVATOR_SWING_POSITION_TOLERANCE)
    {
        swing_motion.active = false;
        return StepMotor_FromDriverResult(StepMotor_HaltSwing());
    }

    direction = (error > 0) ? BSP_PD42S1_DIRECTION_CW : BSP_PD42S1_DIRECTION_CCW;
    if (BSP_PD42S1_SetSpeed(address,
                            direction,
                            EXCAVATOR_SWING_ACCELERATION,
                            EXCAVATOR_SWING_SPEED_RPM) != BSP_PD42S1_OK)
    {
        StepMotor_MarkUnavailable();
        return STEP_MOTOR_CONTROL_DRIVER_ERROR;
    }

    swing_motion.target_position = target_position;
    swing_motion.previous_error = error;
    swing_motion.best_absolute_error = StepMotor_Absolute(error);
    swing_motion.last_poll_time = xTaskGetTickCount();
    swing_motion.read_failures = 0U;
    swing_motion.active = true;
    return STEP_MOTOR_CONTROL_OK;
}

void StepMotor_ProcessCenteredMotion(void)
{
    TickType_t now;
    int32_t current_position;
    int32_t error;
    bool target_reached;
    int32_t absolute_error;

    if (!step_motor_ready || !swing_motion.active)
    {
        return;
    }

    now = xTaskGetTickCount();
    if ((now - swing_motion.last_poll_time) <
        pdMS_TO_TICKS(EXCAVATOR_SWING_POSITION_POLL_MS))
    {
        return;
    }
    swing_motion.last_poll_time = now;

    if (BSP_PD42S1_ReadPosition(EXCAVATOR_SWING_MOTOR_ADDRESS,
                                &current_position) != BSP_PD42S1_OK)
    {
        swing_motion.read_failures++;
        if (swing_motion.read_failures >= EXCAVATOR_SWING_READ_FAILURE_MAX)
        {
            (void)StepMotor_HaltSwing();
            StepMotor_MarkUnavailable();
            StepMotor_SendText("STEP_SWING_POSITION_ERROR\r\n");
        }
        return;
    }

    swing_motion.read_failures = 0U;
    error = swing_motion.target_position - current_position;
    absolute_error = StepMotor_Absolute(error);
    target_reached = ((int64_t)error <= (int64_t)EXCAVATOR_SWING_POSITION_TOLERANCE &&
                      (int64_t)error >= -(int64_t)EXCAVATOR_SWING_POSITION_TOLERANCE) ||
                     ((swing_motion.previous_error > 0 && error <= 0) ||
                      (swing_motion.previous_error < 0 && error >= 0));

    if (target_reached)
    {
        swing_motion.active = false;
        (void)StepMotor_HaltSwing();
        return;
    }

    if (absolute_error < swing_motion.best_absolute_error)
    {
        swing_motion.best_absolute_error = absolute_error;
    }
    else if (absolute_error >
             swing_motion.best_absolute_error +
                 (int32_t)EXCAVATOR_SWING_MOVING_AWAY_MARGIN)
    {
        swing_motion.active = false;
        (void)StepMotor_HaltSwing();
        StepMotor_SendText("STEP_SWING_DIRECTION_ERROR\r\n");
        return;
    }

    swing_motion.previous_error = error;
}

StepMotorControlResult StepMotor_GetPosition(uint8_t address,
                                             int32_t *position_pulses)
{
    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    if (!StepMotor_IsConfiguredAddress(address) || position_pulses == NULL)
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }

    return StepMotor_FromDriverResult(
        BSP_PD42S1_ReadPosition(address, position_pulses));
}

StepMotorControlResult StepMotor_GetSpeed(uint8_t address,
                                         int16_t *speed_rpm)
{
    if (!step_motor_ready)
    {
        return STEP_MOTOR_CONTROL_NOT_READY;
    }

    if (!StepMotor_IsConfiguredAddress(address) || speed_rpm == NULL)
    {
        return STEP_MOTOR_CONTROL_INVALID_ARGUMENT;
    }

    return StepMotor_FromDriverResult(BSP_PD42S1_ReadSpeed(address, speed_rpm));
}

bool StepMotor_IsCenterSet(uint8_t address)
{
    return address == EXCAVATOR_SWING_MOTOR_ADDRESS &&
           step_motor_ready && swing_motor_center_set;
}

void StepMotor_TaskCreate(void)
{
    (void)xTaskCreate(StepMotor_Task,
                      STEP_MOTOR_TASK_NAME,
                      STEP_MOTOR_TASK_STACK_WORDS,
                      NULL,
                      STEP_MOTOR_TASK_PRIORITY,
                      NULL);
}
