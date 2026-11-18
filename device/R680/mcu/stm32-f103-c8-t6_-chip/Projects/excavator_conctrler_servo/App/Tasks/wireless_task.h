#ifndef WIRELESS_TASK_H
#define WIRELESS_TASK_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define WIRELESS_TASK_NAME          "wireless"
#define WIRELESS_TASK_STACK_WORDS   256U
#define WIRELESS_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
#define WIRELESS_CONTROL_RATE_HZ    30U
#define WIRELESS_RF_CHANNEL         76U

/**
 * @brief Wireless module runtime state.
 */
typedef enum
{
    WIRELESS_STATUS_BOOT = 0,
    WIRELESS_STATUS_OK,
    WIRELESS_STATUS_NOT_FOUND,
    WIRELESS_STATUS_CONFIG_ERROR,
    WIRELESS_STATUS_TX_FAILED,
    WIRELESS_STATUS_CONNECTED,
    WIRELESS_STATUS_CONNECTION_ERROR,
} WirelessStatusCode;

/**
 * @brief Wireless status message published to the display task.
 */
typedef struct
{
    WirelessStatusCode code;
    uint8_t channel;
    uint8_t status_reg;
} WirelessStatus;

/**
 * @brief Last wireless status, exposed for non-intrusive SWD diagnostics.
 */
extern volatile WirelessStatusCode g_wireless_last_status;

/**
 * @brief Create the wireless module task and status queue.
 */
void Wireless_TaskCreate(void);

/**
 * @brief Publish wireless status to the display queue.
 * @param code Wireless status code.
 */
void Wireless_PublishStatus(WirelessStatusCode code);

/**
 * @brief Get the queue used to publish wireless status.
 * @return Queue handle, or null if the queue was not created.
 */
QueueHandle_t Wireless_GetStatusQueue(void);

#endif /* WIRELESS_TASK_H */
