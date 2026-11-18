#ifndef TEST_QUEUE_H
#define TEST_QUEUE_H
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned count, unsigned size);
BaseType_t xQueueReceive(QueueHandle_t queue, void *value, TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *value);
#endif
