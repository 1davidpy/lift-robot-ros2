#ifndef LIFT_UART_TASK_H
#define LIFT_UART_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define LIFT_UART_TASK_NAME "lift_uart"
#define LIFT_UART_TASK_STACK_WORDS 384U
#define LIFT_UART_TASK_PRIORITY (tskIDLE_PRIORITY + 3U)

void LiftUart_TaskCreate(void);

#endif /* LIFT_UART_TASK_H */
