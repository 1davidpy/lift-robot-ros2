#include "freertos_app.h"

#include "FreeRTOS.h"
#include "position_monitor_task.h"
#include "servo_jog_task.h"
#include "lift_uart_task.h"
#include "system_led_task.h"
#include "task.h"

void FreeRTOS_AppInit(void)
{
    PositionMonitor_TaskCreate();
    ServoJog_TaskCreate();
    LiftUart_TaskCreate();
    SystemLED_TaskCreate();
}

void FreeRTOS_AppStart(void)
{
    vTaskStartScheduler();

    for (;;)
    {
    }
}
