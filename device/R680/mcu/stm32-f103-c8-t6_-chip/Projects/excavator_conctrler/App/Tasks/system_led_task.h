#ifndef SYSTEM_LED_TASK_H
#define SYSTEM_LED_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define SYSTEM_LED_TASK_NAME          "sys_led"
#define SYSTEM_LED_TASK_STACK_WORDS   128U
#define SYSTEM_LED_TASK_PRIORITY      tskIDLE_PRIORITY

void SystemLED_TaskCreate(void);

#endif /* SYSTEM_LED_TASK_H */
