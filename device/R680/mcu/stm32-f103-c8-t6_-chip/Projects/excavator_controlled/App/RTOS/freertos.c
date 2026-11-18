#include "freertos_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "arm_control_task.h"
#include "nrf_control_rx_task.h"
#include "step_motor_task.h"

void FreeRTOS_AppInit(void)
{
    StepMotor_TaskCreate();
    ArmControl_TaskCreate();
    NrfControlRx_TaskCreate();
}

void FreeRTOS_AppStart(void)
{
    vTaskStartScheduler();

    for (;;)
    {
    }
}
