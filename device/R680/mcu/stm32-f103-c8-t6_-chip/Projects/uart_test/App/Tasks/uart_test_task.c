#include "uart_test_task.h"

#include <stdint.h>

#include "bsp_uart.h"

#define UART_TEST_ALIVE_PERIOD_MS  1000U

static const uint8_t uart_test_ready_message[] = "uart_test ready\r\n";
static const uint8_t uart_test_alive_message[] = "uart_test alive\r\n";

static void UartTest_Task(void *argument)
{
    TickType_t last_alive_time;
    uint8_t byte;

    (void)argument;

    BSP_UART_DebugInit();
    (void)BSP_UART_DebugWrite(uart_test_ready_message, sizeof(uart_test_ready_message) - 1U);
    last_alive_time = xTaskGetTickCount();

    for (;;)
    {
        while (BSP_UART_DebugTryRead(&byte))
        {
            (void)BSP_UART_DebugWrite(&byte, 1U);
        }

        if ((xTaskGetTickCount() - last_alive_time) >= pdMS_TO_TICKS(UART_TEST_ALIVE_PERIOD_MS))
        {
            (void)BSP_UART_DebugWrite(uart_test_alive_message, sizeof(uart_test_alive_message) - 1U);
            last_alive_time = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void UartTest_TaskCreate(void)
{
    (void)xTaskCreate(UartTest_Task,
                      UART_TEST_TASK_NAME,
                      UART_TEST_TASK_STACK_WORDS,
                      NULL,
                      UART_TEST_TASK_PRIORITY,
                      NULL);
}
