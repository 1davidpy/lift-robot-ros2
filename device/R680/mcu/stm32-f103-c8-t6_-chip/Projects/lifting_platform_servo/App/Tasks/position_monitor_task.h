#ifndef POSITION_MONITOR_TASK_H
#define POSITION_MONITOR_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define POSITION_MONITOR_TASK_NAME          "position"
#define POSITION_MONITOR_TASK_STACK_WORDS   128U
#define POSITION_MONITOR_TASK_PRIORITY      (tskIDLE_PRIORITY + 3U)

/**
 * @brief Create the debounced discrete position monitor task.
 */
void PositionMonitor_TaskCreate(void);

#endif /* POSITION_MONITOR_TASK_H */
