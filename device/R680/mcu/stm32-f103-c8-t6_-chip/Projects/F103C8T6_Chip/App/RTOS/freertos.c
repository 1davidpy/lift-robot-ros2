#include "freertos_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "system_led_task.h"

void FreeRTOS_AppInit(void)
{
    SystemLED_TaskCreate();
}

void FreeRTOS_AppStart(void)
{
    vTaskStartScheduler();

    for (;;)
    {
    }
}
