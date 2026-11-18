#ifndef INPUT_ADC_TASK_H
#define INPUT_ADC_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define INPUT_ADC_TASK_NAME          "input_adc"
#define INPUT_ADC_TASK_STACK_WORDS   192U
#define INPUT_ADC_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
#define INPUT_ADC_UPDATE_PERIOD_MS   20U

typedef struct
{
    uint16_t left_y;
    uint16_t left_x;
    uint16_t right_y;
    uint16_t right_x;
    uint16_t wheel_left;
    uint16_t wheel_right;
} InputAdcSample;

/**
 * @brief Create the ADC input sampling task and data queue.
 */
void InputADC_TaskCreate(void);

/**
 * @brief Get the queue used to publish ADC samples.
 * @return Queue handle, or null if the queue was not created.
 */
QueueHandle_t InputADC_GetQueue(void);

#endif /* INPUT_ADC_TASK_H */
