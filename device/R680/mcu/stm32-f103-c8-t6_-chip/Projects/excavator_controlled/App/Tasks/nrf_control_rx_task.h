#ifndef NRF_CONTROL_RX_TASK_H
#define NRF_CONTROL_RX_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define NRF_CONTROL_RX_TASK_NAME          "nrf_rx"
#define NRF_CONTROL_RX_TASK_STACK_WORDS   320U
#define NRF_CONTROL_RX_TASK_PRIORITY      (tskIDLE_PRIORITY + 3U)

/**
 * @brief Create the automatic NRF24L01 control receiver task.
 */
void NrfControlRx_TaskCreate(void);

#endif /* NRF_CONTROL_RX_TASK_H */
