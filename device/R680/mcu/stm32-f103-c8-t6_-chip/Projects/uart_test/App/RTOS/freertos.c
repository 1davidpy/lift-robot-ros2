#include "freertos_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart_test_task.h"

void FreeRTOS_AppInit(void)
{
    UartTest_TaskCreate();
}

void FreeRTOS_AppStart(void)
{
    vTaskStartScheduler();

    for (;;)
    {
    }
}
