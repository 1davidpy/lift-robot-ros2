#ifndef NRF_CONTROL_TX_TASK_H
#define NRF_CONTROL_TX_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define NRF_CONTROL_TX_TASK_NAME          "nrf_ctrl"
#define NRF_CONTROL_TX_TASK_STACK_WORDS   320U
#define NRF_CONTROL_TX_TASK_PRIORITY      (tskIDLE_PRIORITY + 4U)
#define NRF_CONTROL_TX_PERIOD_MS          33U
#define NRF_CONTROL_TX_TIMEOUT_MS         100U
#define NRF_CONTROL_TX_REINIT_PERIOD_MS   1000U

/**
 * @brief Create the NRF24L01 30Hz control packet TX task.
 */
void NrfControlTx_TaskCreate(void);

/**
 * @brief Get the selected chassis speed level.
 * @return Speed level from 1 through 6.
 */
uint8_t NrfControlTx_GetTrackSpeedLevel(void);

#endif /* NRF_CONTROL_TX_TASK_H */
