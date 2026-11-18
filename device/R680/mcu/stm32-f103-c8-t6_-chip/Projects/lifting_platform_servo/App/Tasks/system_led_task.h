#ifndef SYSTEM_LED_TASK_H
#define SYSTEM_LED_TASK_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#define SYSTEM_LED_TASK_NAME              "sys_led"
#define SYSTEM_LED_TASK_STACK_WORDS       128U
#define SYSTEM_LED_TASK_PRIORITY          (tskIDLE_PRIORITY + 1U)
#define SYSTEM_LED_HALF_PERIOD_MS         500U
#define SYSTEM_LED_REFRESH_MS             20U
#define SYSTEM_LED_COMM_TIMEOUT_MS        1500U

/** @brief Record a completed motor operation without performing LED or bus IO. */
void SystemLED_ReportMotorCommunication(bool successful);

/** @brief Create the independent MCU heartbeat and motor communication LED task. */
void SystemLED_TaskCreate(void);

#endif /* SYSTEM_LED_TASK_H */
