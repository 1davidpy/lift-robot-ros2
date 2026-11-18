#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
#define tskIDLE_PRIORITY 0U
#define pdMS_TO_TICKS(x) (x)
#define portTICK_PERIOD_MS 1U
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
