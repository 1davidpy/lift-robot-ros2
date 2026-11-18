#include "input_adc_task.h"

#include "bsp_analog.h"

static QueueHandle_t input_adc_queue;

static void InputADC_Task(void *argument)
{
    InputAdcSample sample;

    (void)argument;
    BSP_Analog_Init();

    for (;;)
    {
        sample.left_y = BSP_Analog_Read(BSP_ANALOG_LEFT_Y);
        sample.left_x = BSP_Analog_Read(BSP_ANALOG_LEFT_X);
        sample.right_y = BSP_Analog_Read(BSP_ANALOG_RIGHT_Y);
        sample.right_x = BSP_Analog_Read(BSP_ANALOG_RIGHT_X);
        sample.wheel_left = BSP_Analog_Read(BSP_ANALOG_WHEEL_LEFT);
        sample.wheel_right = BSP_Analog_Read(BSP_ANALOG_WHEEL_RIGHT);

        if (input_adc_queue != NULL)
        {
            (void)xQueueOverwrite(input_adc_queue, &sample);
        }

        vTaskDelay(pdMS_TO_TICKS(INPUT_ADC_UPDATE_PERIOD_MS));
    }
}

void InputADC_TaskCreate(void)
{
    input_adc_queue = xQueueCreate(1U, sizeof(InputAdcSample));

    (void)xTaskCreate(InputADC_Task,
                      INPUT_ADC_TASK_NAME,
                      INPUT_ADC_TASK_STACK_WORDS,
                      NULL,
                      INPUT_ADC_TASK_PRIORITY,
                      NULL);
}

QueueHandle_t InputADC_GetQueue(void)
{
    return input_adc_queue;
}
