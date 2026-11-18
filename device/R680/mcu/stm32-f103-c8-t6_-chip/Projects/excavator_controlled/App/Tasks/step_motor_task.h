#ifndef STEP_MOTOR_TASK_H
#define STEP_MOTOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

#define STEP_MOTOR_TASK_NAME          "step_motor"
#define STEP_MOTOR_TASK_STACK_WORDS   256U
#define STEP_MOTOR_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
#define STEP_MOTOR_SPEED_RPM_MAX       3000U
#define STEP_MOTOR_CENTER_ANGLE_MAX    360

/**
 * @brief Step-motor control operation result.
 */
typedef enum
{
    STEP_MOTOR_CONTROL_OK = 0,
    STEP_MOTOR_CONTROL_NOT_READY,
    STEP_MOTOR_CONTROL_INVALID_ARGUMENT,
    STEP_MOTOR_CONTROL_CENTER_REQUIRED,
    STEP_MOTOR_CONTROL_STORAGE_ERROR,
    STEP_MOTOR_CONTROL_DRIVER_ERROR,
} StepMotorControlResult;

/**
 * @brief Create the PD42S1 control initialization task.
 */
void StepMotor_TaskCreate(void);

/**
 * @brief Run one configured step motor continuously.
 * @param address Motor address.
 * @param direction Zero for CW or one for CCW.
 * @param speed_rpm Target speed in RPM.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_Run(uint8_t address, uint8_t direction, uint16_t speed_rpm);

/**
 * @brief Stop one configured step motor with the configured deceleration.
 * @param address Motor address.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_Stop(uint8_t address);

/**
 * @brief Stop all configured step motors with one broadcast command.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_StopAll(void);

/**
 * @brief Use the current position of the configured swing motor as its center.
 * @param address Swing motor address.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_SetCenter(uint8_t address);

/**
 * @brief Move the configured swing motor within 360 degrees of its center.
 * @param address Swing motor address.
 * @param angle_degrees Signed motor target angle from -360 to 360 degrees.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_MoveCentered(uint8_t address,
                                              int16_t angle_degrees);

/**
 * @brief Read the signed encoder position of one configured motor.
 * @param address Motor address.
 * @param position_pulses Destination for the encoder position.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_GetPosition(uint8_t address,
                                             int32_t *position_pulses);

/**
 * @brief Read the signed real-time speed of one configured motor.
 * @param address Motor address.
 * @param speed_rpm Destination for signed RPM.
 * @return Control operation result.
 */
StepMotorControlResult StepMotor_GetSpeed(uint8_t address,
                                         int16_t *speed_rpm);

/**
 * @brief Check whether the swing motor has a usable software center.
 * @param address Swing motor address.
 * @return True when centered positioning is available.
 */
bool StepMotor_IsCenterSet(uint8_t address);

/**
 * @brief Supervise the active centered move and stop at or across the target.
 */
void StepMotor_ProcessCenteredMotion(void);

#endif /* STEP_MOTOR_TASK_H */
