#include "system_led_task.h"

#include "bsp_led.h"

static void SystemLED_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        BSP_LED_Toggle(BSP_LED_STATUS);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void SystemLED_TaskCreate(void)
{
    (void)xTaskCreate(SystemLED_Task,
                      SYSTEM_LED_TASK_NAME,
                      SYSTEM_LED_TASK_STACK_WORDS,
                      NULL,
                      SYSTEM_LED_TASK_PRIORITY,
                      NULL);
}
