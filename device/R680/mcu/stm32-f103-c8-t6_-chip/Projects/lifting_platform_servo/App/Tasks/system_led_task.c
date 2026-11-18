#include "system_led_task.h"

#include "bsp_led.h"

static volatile bool motor_communication_ok;
static volatile TickType_t motor_communication_tick;

void SystemLED_ReportMotorCommunication(bool successful)
{
    taskENTER_CRITICAL();
    motor_communication_tick = xTaskGetTickCount();
    motor_communication_ok = successful;
    taskEXIT_CRITICAL();
}

static void SystemLED_Task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    TickType_t phase_tick = wake;
    const TickType_t half_period = pdMS_TO_TICKS(SYSTEM_LED_HALF_PERIOD_MS);
    bool illuminated = false;

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        TickType_t phases = (now - phase_tick) / half_period;
        phase_tick += phases * half_period;
        if (phases & 1U) illuminated = !illuminated;

        taskENTER_CRITICAL();
        now = xTaskGetTickCount();
        bool communication_ok = motor_communication_ok &&
            now - motor_communication_tick <= pdMS_TO_TICKS(SYSTEM_LED_COMM_TIMEOUT_MS);
        taskEXIT_CRITICAL();

        BSP_LED_Write(BSP_LED_STATUS, illuminated && communication_ok);
        BSP_LED_Write(BSP_LED_AUX, illuminated);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SYSTEM_LED_REFRESH_MS));
    }
}

void SystemLED_TaskCreate(void)
{
    BaseType_t created = xTaskCreate(SystemLED_Task,
                                   SYSTEM_LED_TASK_NAME,
                                   SYSTEM_LED_TASK_STACK_WORDS,
                                   NULL,
                                   SYSTEM_LED_TASK_PRIORITY,
                                   NULL);
    configASSERT(created == pdPASS);
    (void)created;
}
