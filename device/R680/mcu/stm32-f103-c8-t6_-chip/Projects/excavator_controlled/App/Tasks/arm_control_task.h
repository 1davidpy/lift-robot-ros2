#ifndef ARM_CONTROL_TASK_H
#define ARM_CONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

#include "excavator_protocol.h"

#define ARM_CONTROL_TASK_NAME          "arm_control"
#define ARM_CONTROL_TASK_STACK_WORDS   512U
#define ARM_CONTROL_TASK_PRIORITY      (tskIDLE_PRIORITY + 2U)

/**
 * @brief Create the serial-controlled excavator arm task.
 */
void ArmControl_TaskCreate(void);

/**
 * @brief Submit the newest validated wireless velocity command.
 * @param command Six-channel excavator velocity command.
 * @return True when the command was queued for the arm-control task.
 */
bool ArmControl_SubmitWirelessCommand(const ExcavatorVelocityCmd_t *command);

#endif /* ARM_CONTROL_TASK_H */
