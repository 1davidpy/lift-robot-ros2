#include "freertos_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "display_task.h"
#include "input_adc_task.h"
#include "key_task.h"
#include "nrf_control_tx_task.h"
#include "system_led_task.h"
#include "wireless_task.h"

void FreeRTOS_AppInit(void)
{
    SystemLED_TaskCreate();
    InputADC_TaskCreate();
    Key_TaskCreate();
    Wireless_TaskCreate();
    NrfControlTx_TaskCreate();
    Display_TaskCreate();
}

void FreeRTOS_AppStart(void)
{
    vTaskStartScheduler();

    for (;;)
    {
    }
}
