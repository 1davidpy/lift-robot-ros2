#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define DISPLAY_TASK_NAME          "display"
#define DISPLAY_TASK_STACK_WORDS   256U
#define DISPLAY_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)

void Display_TaskCreate(void);

#endif /* DISPLAY_TASK_H */
