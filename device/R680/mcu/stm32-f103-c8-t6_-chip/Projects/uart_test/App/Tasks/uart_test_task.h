#ifndef UART_TEST_TASK_H
#define UART_TEST_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define UART_TEST_TASK_NAME          "uart_test"
#define UART_TEST_TASK_STACK_WORDS   256U
#define UART_TEST_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)

/**
 * @brief Create the USB-to-UART communication test task.
 */
void UartTest_TaskCreate(void);

#endif /* UART_TEST_TASK_H */
