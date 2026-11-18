#include "key_task.h"

#include "bsp_key.h"

typedef struct
{
    bool stable_pressed;
    bool last_raw_pressed;
    uint16_t debounce_ms;
    uint16_t held_ms;
} KeyRuntimeState;

static QueueHandle_t key_state_queue;
static KeyRuntimeState key_runtime[BSP_KEY_COUNT];

static uint8_t Key_Bit(BspKeyId key)
{
    return (uint8_t)(1U << (uint8_t)key);
}

static void Key_PublishState(uint8_t pressed_mask, uint8_t long_press_mask, uint8_t changed_mask)
{
    KeyState state;

    state.pressed_mask = pressed_mask;
    state.long_press_mask = long_press_mask;
    state.changed_mask = changed_mask;

    if (key_state_queue != NULL)
    {
        (void)xQueueOverwrite(key_state_queue, &state);
    }
}

static void Key_Task(void *argument)
{
    uint8_t pressed_mask = 0U;
    uint8_t long_press_mask = 0U;
    uint8_t last_pressed_mask = 0xFFU;
    uint8_t last_long_press_mask = 0xFFU;

    (void)argument;
    BSP_Key_Init();

    for (;;)
    {
        uint8_t changed_mask = 0U;

        for (uint8_t i = 0U; i < (uint8_t)BSP_KEY_COUNT; i++)
        {
            BspKeyId key = (BspKeyId)i;
            KeyRuntimeState *runtime = &key_runtime[i];
            bool raw_pressed = BSP_Key_IsPressed(key);

            if (raw_pressed != runtime->last_raw_pressed)
            {
                runtime->last_raw_pressed = raw_pressed;
                runtime->debounce_ms = 0U;
            }
            else if (runtime->debounce_ms < KEY_DEBOUNCE_MS)
            {
                runtime->debounce_ms = (uint16_t)(runtime->debounce_ms + KEY_SCAN_PERIOD_MS);
            }
            else if (runtime->stable_pressed != raw_pressed)
            {
                runtime->stable_pressed = raw_pressed;
                changed_mask |= Key_Bit(key);

                if (!runtime->stable_pressed)
                {
                    runtime->held_ms = 0U;
                }
            }

            if (runtime->stable_pressed)
            {
                if (runtime->held_ms < KEY_LONG_PRESS_MS)
                {
                    runtime->held_ms = (uint16_t)(runtime->held_ms + KEY_SCAN_PERIOD_MS);
                }

                pressed_mask |= Key_Bit(key);
                if (runtime->held_ms >= KEY_LONG_PRESS_MS)
                {
                    long_press_mask |= Key_Bit(key);
                }
                else
                {
                    long_press_mask &= (uint8_t)~Key_Bit(key);
                }
            }
            else
            {
                pressed_mask &= (uint8_t)~Key_Bit(key);
                long_press_mask &= (uint8_t)~Key_Bit(key);
            }
        }

        if (pressed_mask != last_pressed_mask ||
            long_press_mask != last_long_press_mask ||
            changed_mask != 0U)
        {
            Key_PublishState(pressed_mask, long_press_mask, changed_mask);
            last_pressed_mask = pressed_mask;
            last_long_press_mask = long_press_mask;
        }

        vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS));
    }
}

void Key_TaskCreate(void)
{
    key_state_queue = xQueueCreate(1U, sizeof(KeyState));

    (void)xTaskCreate(Key_Task,
                      KEY_TASK_NAME,
                      KEY_TASK_STACK_WORDS,
                      NULL,
                      KEY_TASK_PRIORITY,
                      NULL);
}

QueueHandle_t Key_GetStateQueue(void)
{
    return key_state_queue;
}
