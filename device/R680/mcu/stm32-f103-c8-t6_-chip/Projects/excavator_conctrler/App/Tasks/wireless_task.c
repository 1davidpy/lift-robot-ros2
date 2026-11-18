#include "wireless_task.h"

#include "bsp_nrf24l01.h"

static QueueHandle_t wireless_status_queue;

void Wireless_PublishStatus(WirelessStatusCode code)
{
    WirelessStatus status;

    status.code = code;
    status.channel = WIRELESS_RF_CHANNEL;
    status.status_reg = (code == WIRELESS_STATUS_BOOT) ? 0U : BSP_NRF24L01_ReadStatus();

    if (wireless_status_queue != NULL)
    {
        (void)xQueueOverwrite(wireless_status_queue, &status);
    }
}

void Wireless_TaskCreate(void)
{
    wireless_status_queue = xQueueCreate(1U, sizeof(WirelessStatus));
}

QueueHandle_t Wireless_GetStatusQueue(void)
{
    return wireless_status_queue;
}
