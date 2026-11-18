#include "nrf_control_rx_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_nrf24l01.h"
#include "excavator_protocol.h"
#include "servo_control_task.h"

#define NRF_CONTROL_RX_RF_CHANNEL           76U
#define NRF_CONTROL_RX_PIPE                 0U
#define NRF_CONTROL_RX_POLL_PERIOD_MS       2U
#define NRF_CONTROL_RX_COMMAND_TIMEOUT_MS   200U
#define NRF_CONTROL_RX_REINIT_PERIOD_MS     2000U

static const uint8_t nrf_control_rx_address[BSP_NRF24L01_ADDRESS_WIDTH] = {
    0x34U, 0x43U, 0x10U, 0x10U, 0x02U,
};

static const uint8_t nrf_control_rx_ack[EXCAVATOR_PROTOCOL_ACK_LEN] = {
    EXCAVATOR_PROTOCOL_ACK_OK_1,
    EXCAVATOR_PROTOCOL_ACK_OK_2,
};

static void NrfControlRx_DelayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void NrfControlRx_DelayUs(uint32_t us)
{
    const uint32_t loop_count = us * 8U;

    for (volatile uint32_t i = 0U; i < loop_count; i++)
    {
    }
}

static bool NrfControlRx_QueueAck(void)
{
    if (BSP_NRF24L01_WriteAckPayload(NRF_CONTROL_RX_PIPE,
                                     nrf_control_rx_ack,
                                     sizeof(nrf_control_rx_ack)) == BSP_NRF24L01_OK)
    {
        return true;
    }

    BSP_NRF24L01_FlushTx();
    return BSP_NRF24L01_WriteAckPayload(NRF_CONTROL_RX_PIPE,
                                        nrf_control_rx_ack,
                                        sizeof(nrf_control_rx_ack)) == BSP_NRF24L01_OK;
}

static BspNrf24l01Result NrfControlRx_RadioInit(void)
{
    BspNrf24l01Result result;

    BSP_NRF24L01_SetDelayCallbacks(NrfControlRx_DelayMs, NrfControlRx_DelayUs);
    result = BSP_NRF24L01_Init();
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetAddress(nrf_control_rx_address,
                                     BSP_NRF24L01_ADDRESS_WIDTH);
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetChannel(NRF_CONTROL_RX_RF_CHANNEL);
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetRfConfig(BSP_NRF24L01_DATA_RATE_250KBPS,
                                      BSP_NRF24L01_POWER_0DBM);
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_SetPayloadLength(sizeof(ControlPacket_t));
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_EnableAckPayload();
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    result = BSP_NRF24L01_StartRx();
    if (result != BSP_NRF24L01_OK)
    {
        return result;
    }

    return NrfControlRx_QueueAck() ? BSP_NRF24L01_OK : BSP_NRF24L01_ERROR;
}

static int8_t NrfControlRx_Clamp(int8_t value, int8_t limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return (int8_t)-limit;
    }

    return value;
}

static void NrfControlRx_CopyCommand(const ControlPacket_t *packet,
                                     ExcavatorVelocityCmd_t *command)
{
    command->left_track = NrfControlRx_Clamp(packet->left_track,
                                             EXCAVATOR_TRACK_LEVEL_MAX);
    command->right_track = NrfControlRx_Clamp(packet->right_track,
                                              EXCAVATOR_TRACK_LEVEL_MAX);
    command->swing = NrfControlRx_Clamp(packet->swing,
                                        EXCAVATOR_JOYSTICK_LEVEL_MAX);
    command->boom = NrfControlRx_Clamp(packet->boom,
                                       EXCAVATOR_JOYSTICK_LEVEL_MAX);
    command->arm = NrfControlRx_Clamp(packet->arm,
                                      EXCAVATOR_JOYSTICK_LEVEL_MAX);
    command->bucket = NrfControlRx_Clamp(packet->bucket,
                                         EXCAVATOR_JOYSTICK_LEVEL_MAX);
    command->gripper = NrfControlRx_Clamp(packet->gripper,
                                          EXCAVATOR_JOYSTICK_LEVEL_MAX);
}

static void NrfControlRx_Task(void *argument)
{
    ControlPacket_t packet;
    ExcavatorVelocityCmd_t command;
    TickType_t last_valid_time = 0U;
    TickType_t last_init_time = 0U;
    bool radio_ready = false;
    bool command_active = false;
    bool have_sequence = false;
    uint8_t last_sequence = 0U;

    (void)argument;

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();

        if (!radio_ready ||
            (!command_active &&
             (now - last_init_time) >= pdMS_TO_TICKS(NRF_CONTROL_RX_REINIT_PERIOD_MS)))
        {
            radio_ready = NrfControlRx_RadioInit() == BSP_NRF24L01_OK;
            last_init_time = xTaskGetTickCount();
            have_sequence = false;

            if (!radio_ready)
            {
                vTaskDelay(pdMS_TO_TICKS(500U));
                continue;
            }
        }

        uint8_t payload_length = 0U;
        BspNrf24l01Result result =
            BSP_NRF24L01_ReadPayload((uint8_t *)&packet,
                                     sizeof(packet),
                                     &payload_length);

        now = xTaskGetTickCount();
        if (result == BSP_NRF24L01_OK)
        {
            if (payload_length == sizeof(packet) &&
                ExcavatorProtocol_ValidateControlPacket(&packet))
            {
                bool accepted = true;

                if (!have_sequence || packet.seq != last_sequence)
                {
                    NrfControlRx_CopyCommand(&packet, &command);
                    accepted = ServoControl_SubmitWirelessCommand(&command);
                }

                if (accepted)
                {
                    last_sequence = packet.seq;
                    have_sequence = true;
                    last_valid_time = now;
                    command_active = true;
                    (void)NrfControlRx_QueueAck();
                }
            }
            else
            {
                BSP_NRF24L01_FlushTx();
            }
        }

        if (command_active &&
            (now - last_valid_time) >= pdMS_TO_TICKS(NRF_CONTROL_RX_COMMAND_TIMEOUT_MS))
        {
            ServoControl_NotifyWirelessTimeout();
            command_active = false;
            have_sequence = false;
        }

        vTaskDelay(pdMS_TO_TICKS(NRF_CONTROL_RX_POLL_PERIOD_MS));
    }
}

void NrfControlRx_TaskCreate(void)
{
    (void)xTaskCreate(NrfControlRx_Task,
                      NRF_CONTROL_RX_TASK_NAME,
                      NRF_CONTROL_RX_TASK_STACK_WORDS,
                      NULL,
                      NRF_CONTROL_RX_TASK_PRIORITY,
                      NULL);
}
