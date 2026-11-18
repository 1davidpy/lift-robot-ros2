#ifndef SERVO_CONTROL_TASK_H
#define SERVO_CONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

#include "excavator_protocol.h"

#define SERVO_CONTROL_TASK_NAME          "servo_control"
#define SERVO_CONTROL_TASK_STACK_WORDS   512U
#define SERVO_CONTROL_TASK_PRIORITY      (tskIDLE_PRIORITY + 2U)

/**
 * @brief Create the seven-servo excavator control task.
 */
void ServoControl_TaskCreate(void);

/**
 * @brief Submit the newest validated wireless velocity command.
 * @param command Seven-channel excavator velocity command.
 * @return True when the latest command is accepted; read-only mode never queues motion.
 */
bool ServoControl_SubmitWirelessCommand(const ExcavatorVelocityCmd_t *command);

/** @brief Request a chassis stop and neutral rearming after radio loss. */
void ServoControl_NotifyWirelessTimeout(void);

#endif /* SERVO_CONTROL_TASK_H */
