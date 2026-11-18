#include "freertos_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "nrf_control_rx_task.h"
#include "servo_control_task.h"

void FreeRTOS_AppInit(void)
{
    ServoControl_TaskCreate();
    NrfControlRx_TaskCreate();
}

void FreeRTOS_AppStart(void)
{
    vTaskStartScheduler();

    for (;;)
    {
    }
}
