#ifndef KEY_TASK_H
#define KEY_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define KEY_TASK_NAME             "key"
#define KEY_TASK_STACK_WORDS      192U
#define KEY_TASK_PRIORITY         (tskIDLE_PRIORITY + 1U)
#define KEY_SCAN_PERIOD_MS        10U
#define KEY_DEBOUNCE_MS           30U
#define KEY_LONG_PRESS_MS         800U

#define KEY_MASK_TOP_LEFT         (1U << 0U)
#define KEY_MASK_TOP_RIGHT        (1U << 1U)
#define KEY_MASK_BOTTOM_LEFT      (1U << 2U)
#define KEY_MASK_BOTTOM_RIGHT     (1U << 3U)

/**
 * @brief Debounced key state published by the key task.
 */
typedef struct
{
    uint8_t pressed_mask;
    uint8_t long_press_mask;
    uint8_t changed_mask;
} KeyState;

/**
 * @brief Create the key scan task and state queue.
 */
void Key_TaskCreate(void);

/**
 * @brief Get the queue used to publish debounced key states.
 * @return Queue handle, or null if the queue was not created.
 */
QueueHandle_t Key_GetStateQueue(void);

#endif /* KEY_TASK_H */
