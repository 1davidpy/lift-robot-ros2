#include "nrf_control_tx_task.h"

#include <stdint.h>

#include "bsp_nrf24l01.h"
#include "excavator_protocol.h"
#include "input_adc_task.h"
#include "key_task.h"
#include "wireless_task.h"

#define NRF_CONTROL_TX_RF_CHANNEL      76U

#ifdef EXCAVATOR_CONTROL_KEY_SELF_TEST
#define NRF_CONTROL_SELF_TEST_START_MS       2000U
#define NRF_CONTROL_SELF_TEST_PRESS_MS       400U
#define NRF_CONTROL_SELF_TEST_RELEASE_MS     600U
#define NRF_CONTROL_SELF_TEST_L1_MASK        (1U << 0U)
#define NRF_CONTROL_SELF_TEST_R1_MASK        (1U << 1U)
#endif

static const uint8_t nrf_control_tx_rx_address[BSP_NRF24L01_ADDRESS_WIDTH] = {
    0x34U, 0x43U, 0x10U, 0x10U, 0x01U,
};

static void NrfControlTx_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void NrfControlTx_DelayUs(uint32_t us)
{
    const uint32_t loop_count = us * 8U;

    for (volatile uint32_t i = 0; i < loop_count; i++)
    {
    }
}

static BspNrf24l01Result NrfControlTx_RadioInit(void)
{
    BspNrf24l01Result result;

    BSP_NRF24L01_SetDelayCallbacks(NrfControlTx_DelayMs, NrfControlTx_DelayUs);
    result = BSP_NRF24L01_Init();
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetAddress(nrf_control_tx_rx_address, BSP_NRF24L01_ADDRESS_WIDTH);
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetChannel(NRF_CONTROL_TX_RF_CHANNEL);
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetRfConfig(BSP_NRF24L01_DATA_RATE_250KBPS, BSP_NRF24L01_POWER_0DBM);
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetPayloadLength(sizeof(ControlPacket_t));
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    return BSP_NRF24L01_EnableAckPayload();
}

static void NrfControlTx_FillAdc(const InputAdcSample *sample, ExcavatorControlAdc_t *adc)
{
    adc->left_y = sample->left_y;
    adc->left_x = sample->left_x;
    adc->right_y = sample->right_y;
    adc->right_x = sample->right_x;
    adc->wheel_left = sample->wheel_left;
    adc->wheel_right = sample->wheel_right;
}

#ifdef EXCAVATOR_CONTROL_KEY_SELF_TEST
static uint8_t NrfControlTx_GetSimulatedKeys(TickType_t elapsed)
{
    TickType_t l1_start = pdMS_TO_TICKS(NRF_CONTROL_SELF_TEST_START_MS);
    TickType_t l1_end = l1_start + pdMS_TO_TICKS(NRF_CONTROL_SELF_TEST_PRESS_MS);
    TickType_t r1_start = l1_end + pdMS_TO_TICKS(NRF_CONTROL_SELF_TEST_RELEASE_MS);
    TickType_t r1_end = r1_start + pdMS_TO_TICKS(NRF_CONTROL_SELF_TEST_PRESS_MS);

    if (elapsed >= l1_start && elapsed < l1_end)
    {
        return NRF_CONTROL_SELF_TEST_L1_MASK;
    }

    if (elapsed >= r1_start && elapsed < r1_end)
    {
        return NRF_CONTROL_SELF_TEST_R1_MASK;
    }

    return 0U;
}
#endif

static void NrfControlTx_Task(void *argument)
{
    QueueHandle_t input_queue;
    QueueHandle_t key_queue;
    InputAdcSample sample = {
        .left_y = 1967U,
        .left_x = 1967U,
        .right_y = 2086U,
        .right_x = 1975U,
        .wheel_left = 750U,
        .wheel_right = 750U,
    };
    KeyState key_state = {0};
    ExcavatorControlAdc_t adc = {0};
    ExcavatorVelocityCmd_t command = {0};
    ControlPacket_t packet;
    uint8_t ack_payload[BSP_NRF24L01_MAX_PAYLOAD_LENGTH] = {0U};
    uint8_t ack_length = 0U;
    BspNrf24l01Result result;
    WirelessStatusCode last_status = WIRELESS_STATUS_BOOT;
    uint8_t active_actuator_axis = EXCAVATOR_ACTUATOR_AXIS_NONE;
    TickType_t last_wake_time;
    TickType_t last_init_time;
#ifdef EXCAVATOR_CONTROL_KEY_SELF_TEST
    TickType_t self_test_start_time;
#endif
    uint8_t seq = 0U;

    (void)argument;
    Wireless_PublishStatus(WIRELESS_STATUS_BOOT);

    result = NrfControlTx_RadioInit();
    last_init_time = xTaskGetTickCount();
    if (result != BSP_NRF24L01_OK)
    {
        Wireless_PublishStatus(WIRELESS_STATUS_NOT_FOUND);
        last_status = WIRELESS_STATUS_NOT_FOUND;
    }
    else
    {
        Wireless_PublishStatus(WIRELESS_STATUS_OK);
        last_status = WIRELESS_STATUS_OK;
    }

    input_queue = InputADC_GetQueue();
    key_queue = Key_GetStateQueue();
    last_wake_time = xTaskGetTickCount();
#ifdef EXCAVATOR_CONTROL_KEY_SELF_TEST
    self_test_start_time = last_wake_time;
#endif

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        if (result != BSP_NRF24L01_OK &&
            (now - last_init_time) >= pdMS_TO_TICKS(NRF_CONTROL_TX_REINIT_PERIOD_MS))
        {
            result = NrfControlTx_RadioInit();
            last_init_time = xTaskGetTickCount();

            WirelessStatusCode init_status = (result == BSP_NRF24L01_OK)
                                                 ? WIRELESS_STATUS_OK
                                                 : WIRELESS_STATUS_NOT_FOUND;
            if (init_status != last_status)
            {
                Wireless_PublishStatus(init_status);
                last_status = init_status;
            }
        }

        if (result == BSP_NRF24L01_OK)
        {
            WirelessStatusCode current_status;
            BspNrf24l01Result send_result;

            if (input_queue != NULL)
            {
                (void)xQueuePeek(input_queue, &sample, 0U);
            }
            if (key_queue != NULL)
            {
                (void)xQueuePeek(key_queue, &key_state, 0U);
            }
#ifdef EXCAVATOR_CONTROL_KEY_SELF_TEST
            key_state.pressed_mask =
                NrfControlTx_GetSimulatedKeys(now - self_test_start_time);
#endif

            NrfControlTx_FillAdc(&sample, &adc);
            ExcavatorProtocol_BuildVelocityFromAdc(&adc,
                                                    &active_actuator_axis,
                                                    &command);
            ExcavatorProtocol_ApplyDriveButtons(key_state.pressed_mask, &command);
            ExcavatorProtocol_BuildControlPacket(&packet, seq++, &command);

            ack_length = 0U;
            send_result = BSP_NRF24L01_SendWithAckPayload((const uint8_t *)&packet,
                                                          sizeof(packet),
                                                          ack_payload,
                                                          sizeof(ack_payload),
                                                          &ack_length,
                                                          NRF_CONTROL_TX_TIMEOUT_MS);
            current_status = (send_result == BSP_NRF24L01_OK &&
                              ack_length >= EXCAVATOR_PROTOCOL_ACK_LEN &&
                              ack_payload[0] == EXCAVATOR_PROTOCOL_ACK_OK_1 &&
                              ack_payload[1] == EXCAVATOR_PROTOCOL_ACK_OK_2)
                                 ? WIRELESS_STATUS_CONNECTED
                                 : WIRELESS_STATUS_CONNECTION_ERROR;

            if (current_status != last_status)
            {
                Wireless_PublishStatus(current_status);
                last_status = current_status;
            }

            now = xTaskGetTickCount();
            if (current_status == WIRELESS_STATUS_CONNECTION_ERROR &&
                (now - last_init_time) >= pdMS_TO_TICKS(NRF_CONTROL_TX_REINIT_PERIOD_MS))
            {
                result = NrfControlTx_RadioInit();
                last_init_time = xTaskGetTickCount();

                if (result != BSP_NRF24L01_OK &&
                    last_status != WIRELESS_STATUS_NOT_FOUND)
                {
                    Wireless_PublishStatus(WIRELESS_STATUS_NOT_FOUND);
                    last_status = WIRELESS_STATUS_NOT_FOUND;
                }
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(NRF_CONTROL_TX_PERIOD_MS));
    }
}

void NrfControlTx_TaskCreate(void)
{
    (void)xTaskCreate(NrfControlTx_Task,
                      NRF_CONTROL_TX_TASK_NAME,
                      NRF_CONTROL_TX_TASK_STACK_WORDS,
                      NULL,
                      NRF_CONTROL_TX_TASK_PRIORITY,
                      NULL);
}
