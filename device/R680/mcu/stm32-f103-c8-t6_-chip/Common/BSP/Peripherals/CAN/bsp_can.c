#include "bsp_can.h"

#include <stddef.h>
#include <string.h>

#include "bsp_pin.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define BSP_CAN_EXTENDED_ID_MAX       0x1FFFFFFFU
#define BSP_CAN_TX_TIMEOUT_LOOPS      1000000U
#define BSP_CAN_STANDARD_ID_MAX       0x7FFU
#define BSP_CAN_RX_QUEUE_LENGTH       16U

#ifndef BSP_CAN_RX_IRQ_ENABLED
#define BSP_CAN_RX_IRQ_ENABLED 0
#endif
#ifndef BSP_CAN_NO_AUTO_RETRY
#define BSP_CAN_NO_AUTO_RETRY 0
#endif
#ifndef BSP_CAN_TX_TIMEOUT_MS
#define BSP_CAN_TX_TIMEOUT_MS 100U
#endif
#if BSP_CAN_TX_TIMEOUT_MS < 1U || BSP_CAN_TX_TIMEOUT_MS > 0x7FFFFFFFU
#error "BSP_CAN_TX_TIMEOUT_MS must fit a positive signed 32-bit interval."
#endif
#if defined(BSP_CAN_RX_STANDARD_ID) && BSP_CAN_RX_STANDARD_ID > BSP_CAN_STANDARD_ID_MAX
#error "BSP_CAN_RX_STANDARD_ID must be an 11-bit identifier."
#endif

#if BSP_CAN_RX_IRQ_ENABLED
#ifndef BSP_CAN_RX_IRQN
#define BSP_CAN_RX_IRQN USB_LP_CAN1_RX0_IRQn
#endif
#ifndef BSP_CAN_RX_IRQ_HANDLER
#define BSP_CAN_RX_IRQ_HANDLER USB_LP_CAN_RX0_IRQHandler
#endif
static volatile BspCanFrame can_rx_queue[BSP_CAN_RX_QUEUE_LENGTH];
static volatile uint8_t can_rx_head;
static volatile uint8_t can_rx_tail;
#endif

typedef struct
{
    uint32_t started_ms;
    bool pending;
    bool canceled;
} BspCanTxMailbox;

static BspCanTxMailbox can_tx_mailboxes[3];
static uint32_t can_now_ms;
static volatile BspCanDiagnostics can_diagnostics;

/**
 * @brief Count and clear one hardware FIFO overrun indication.
 */
static void CAN_RecordRxOverflow(void)
{
    if (CAN_GetFlagStatus(BSP_CAN_INSTANCE, CAN_FLAG_FOV0) != RESET)
    {
        ++can_diagnostics.rx_overflows;
        CAN_ClearFlag(BSP_CAN_INSTANCE, CAN_FLAG_FOV0);
    }
}

/**
 * @brief Validate a received data frame before copying its bounded payload.
 */
static bool CAN_CopyReceived(const CanRxMsg *message, BspCanFrame *frame)
{
    if (message->RTR != CAN_RTR_Data || message->DLC > BSP_CAN_FRAME_DATA_MAX)
    {
        ++can_diagnostics.rx_rejected;
        return false;
    }
#ifdef BSP_CAN_RX_STANDARD_ID
    if (message->IDE != CAN_Id_Standard || message->StdId != BSP_CAN_RX_STANDARD_ID)
    {
        ++can_diagnostics.rx_rejected;
        return false;
    }
#endif
    frame->extended = message->IDE == CAN_Id_Extended;
    frame->id = frame->extended ? message->ExtId : message->StdId;
    frame->length = message->DLC;
    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, message->Data, message->DLC);
    return true;
}

BspCanResult BSP_CAN_Init(void)
{
    GPIO_InitTypeDef gpio_init;
    CAN_InitTypeDef can_init;
    CAN_FilterInitTypeDef filter_init;

#if BSP_CAN_RX_IRQ_ENABLED
    NVIC_DisableIRQ(BSP_CAN_RX_IRQN);
    can_rx_head = 0U;
    can_rx_tail = 0U;
#endif
    memset(can_tx_mailboxes, 0, sizeof(can_tx_mailboxes));
    can_now_ms = 0U;
    can_diagnostics.rx_overflows = 0U;
    can_diagnostics.rx_rejected = 0U;
    can_diagnostics.tx_failures = 0U;
    can_diagnostics.bus_off = 0U;
    RCC_APB2PeriphClockCmd(BSP_CAN_RX_CLOCK | BSP_CAN_TX_CLOCK, ENABLE);
    RCC_APB1PeriphClockCmd(BSP_CAN_CLOCK, ENABLE);
#ifdef BSP_CAN_REMAP
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(BSP_CAN_REMAP, ENABLE);
#endif
#ifdef BSP_CAN_SWJ_REMAP
    /** The SPL remap helper cannot preserve the write-only SWJ field. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(BSP_CAN_SWJ_REMAP, ENABLE);
#endif

    gpio_init.GPIO_Pin = BSP_CAN_TX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BSP_CAN_TX_PORT, &gpio_init);

    gpio_init.GPIO_Pin = BSP_CAN_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(BSP_CAN_RX_PORT, &gpio_init);

    CAN_DeInit(BSP_CAN_INSTANCE);
    CAN_StructInit(&can_init);
    can_init.CAN_TTCM = DISABLE;
    can_init.CAN_ABOM = ENABLE;
    can_init.CAN_AWUM = DISABLE;
    can_init.CAN_NART = BSP_CAN_NO_AUTO_RETRY ? ENABLE : DISABLE;
    can_init.CAN_RFLM = BSP_CAN_RX_IRQ_ENABLED ? ENABLE : DISABLE;
    can_init.CAN_TXFP = ENABLE;
    can_init.CAN_Mode = CAN_Mode_Normal;
    can_init.CAN_SJW = CAN_SJW_1tq;
    can_init.CAN_BS1 = BSP_CAN_BS1;
    can_init.CAN_BS2 = BSP_CAN_BS2;
    can_init.CAN_Prescaler = BSP_CAN_PRESCALER;

    if (CAN_Init(BSP_CAN_INSTANCE, &can_init) != CAN_InitStatus_Success)
    {
        return BSP_CAN_INIT_ERROR;
    }

    filter_init.CAN_FilterNumber = 0U;
    filter_init.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter_init.CAN_FilterScale = CAN_FilterScale_32bit;
    filter_init.CAN_FilterIdHigh = 0U;
    filter_init.CAN_FilterIdLow = 0U;
    filter_init.CAN_FilterMaskIdHigh = 0U;
    filter_init.CAN_FilterMaskIdLow = 0U;
#ifdef BSP_CAN_RX_STANDARD_ID
    filter_init.CAN_FilterIdHigh = (uint16_t)(BSP_CAN_RX_STANDARD_ID << 5U);
    filter_init.CAN_FilterMaskIdHigh = 0xFFE0U;
    filter_init.CAN_FilterMaskIdLow = 0x0006U;
#endif
    filter_init.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter_init.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter_init);

#if BSP_CAN_RX_IRQ_ENABLED
    CAN_ClearFlag(BSP_CAN_INSTANCE, CAN_FLAG_FOV0);
    CAN_ITConfig(BSP_CAN_INSTANCE, CAN_IT_FMP0 | CAN_IT_FOV0, ENABLE);
    NVIC_ClearPendingIRQ(BSP_CAN_RX_IRQN);
    NVIC_SetPriority(BSP_CAN_RX_IRQN, 6U);
    NVIC_EnableIRQ(BSP_CAN_RX_IRQN);
#endif
    return BSP_CAN_OK;
}

BspCanResult BSP_CAN_TransmitExtended(uint32_t id, const uint8_t *data, uint8_t length)
{
    CanTxMsg message = {0};
    uint8_t mailbox;
    uint32_t timeout = BSP_CAN_TX_TIMEOUT_LOOPS;
    uint8_t status;

    if (data == NULL || length == 0U || length > BSP_CAN_FRAME_DATA_MAX ||
        id > BSP_CAN_EXTENDED_ID_MAX)
    {
        return BSP_CAN_INVALID_ARGUMENT;
    }

    BSP_CAN_Service(can_now_ms);
    message.ExtId = id;
    message.IDE = CAN_Id_Extended;
    message.RTR = CAN_RTR_Data;
    message.DLC = length;
    memcpy(message.Data, data, length);

    mailbox = CAN_Transmit(BSP_CAN_INSTANCE, &message);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        ++can_diagnostics.tx_failures;
        return BSP_CAN_TIMEOUT;
    }

    do
    {
        status = CAN_TransmitStatus(BSP_CAN_INSTANCE, mailbox);
        if (status == CAN_TxStatus_Ok)
        {
            return BSP_CAN_OK;
        }
        if (status == CAN_TxStatus_Failed)
        {
            ++can_diagnostics.tx_failures;
            return BSP_CAN_TIMEOUT;
        }
    } while (timeout-- > 0U);

    CAN_CancelTransmit(BSP_CAN_INSTANCE, mailbox);
    ++can_diagnostics.tx_failures;
    return BSP_CAN_TIMEOUT;
}

void BSP_CAN_Service(uint32_t now_ms)
{
    can_now_ms = now_ms;
    can_diagnostics.bus_off =
        CAN_GetFlagStatus(BSP_CAN_INSTANCE, CAN_FLAG_BOF) != RESET;
#if !BSP_CAN_RX_IRQ_ENABLED
    CAN_RecordRxOverflow();
#endif
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        BspCanTxMailbox *mailbox = &can_tx_mailboxes[i];
        if (!mailbox->pending) continue;
        uint8_t status = CAN_TransmitStatus(BSP_CAN_INSTANCE, i);
        if (status != CAN_TxStatus_Pending)
        {
            if (status != CAN_TxStatus_Ok && !mailbox->canceled)
                ++can_diagnostics.tx_failures;
            mailbox->pending = false;
        }
        else if (!mailbox->canceled && (can_diagnostics.bus_off ||
                 now_ms - mailbox->started_ms >= BSP_CAN_TX_TIMEOUT_MS))
        {
            CAN_CancelTransmit(BSP_CAN_INSTANCE, i);
            mailbox->canceled = true;
            ++can_diagnostics.tx_failures;
        }
    }
}

BspCanResult BSP_CAN_TryTransmitStandard(uint16_t id, const uint8_t *data, uint8_t length)
{
    if (id > BSP_CAN_STANDARD_ID_MAX || length > BSP_CAN_FRAME_DATA_MAX ||
        (data == NULL && length != 0U)) return BSP_CAN_INVALID_ARGUMENT;
    BSP_CAN_Service(can_now_ms);
    if (can_diagnostics.bus_off) return BSP_CAN_BUS_OFF;

    CanTxMsg message = {0};
    message.StdId = id;
    message.IDE = CAN_Id_Standard;
    message.RTR = CAN_RTR_Data;
    message.DLC = length;
    if (length != 0U) memcpy(message.Data, data, length);
    uint8_t mailbox = CAN_Transmit(BSP_CAN_INSTANCE, &message);
    if (mailbox == CAN_TxStatus_NoMailBox) return BSP_CAN_BUSY;
    can_tx_mailboxes[mailbox].started_ms = can_now_ms;
    can_tx_mailboxes[mailbox].canceled = false;
    can_tx_mailboxes[mailbox].pending = true;
    return BSP_CAN_OK;
}

void BSP_CAN_GetDiagnostics(BspCanDiagnostics *diagnostics)
{
    if (diagnostics == NULL) return;
    diagnostics->rx_overflows = can_diagnostics.rx_overflows;
    diagnostics->rx_rejected = can_diagnostics.rx_rejected;
    diagnostics->tx_failures = can_diagnostics.tx_failures;
    diagnostics->bus_off = CAN_GetFlagStatus(BSP_CAN_INSTANCE, CAN_FLAG_BOF) != RESET;
}

bool BSP_CAN_TryReceive(BspCanFrame *frame)
{
    if (frame == NULL) return false;
#if BSP_CAN_RX_IRQ_ENABLED
    uint8_t tail = can_rx_tail;
    if (tail == can_rx_head) return false;
    __DMB();
    *frame = can_rx_queue[tail & (BSP_CAN_RX_QUEUE_LENGTH - 1U)];
    __DMB();
    can_rx_tail = (uint8_t)(tail + 1U);
    return true;
#else
    CAN_RecordRxOverflow();
    uint8_t pending = CAN_MessagePending(BSP_CAN_INSTANCE, CAN_FIFO0);
    while (pending-- > 0U)
    {
        CanRxMsg message;
        CAN_Receive(BSP_CAN_INSTANCE, CAN_FIFO0, &message);
        if (CAN_CopyReceived(&message, frame)) return true;
    }
    return false;
#endif
}

#if BSP_CAN_RX_IRQ_ENABLED
/**
 * @brief Drain one FIFO snapshot into the single-producer receive queue.
 */
void BSP_CAN_RX_IRQ_HANDLER(void)
{
    CAN_RecordRxOverflow();
    uint8_t pending = CAN_MessagePending(BSP_CAN_INSTANCE, CAN_FIFO0);
    while (pending-- > 0U)
    {
        CanRxMsg message;
        BspCanFrame frame;
        CAN_Receive(BSP_CAN_INSTANCE, CAN_FIFO0, &message);
        if (!CAN_CopyReceived(&message, &frame)) continue;
        uint8_t head = can_rx_head;
        if ((uint8_t)(head - can_rx_tail) >= BSP_CAN_RX_QUEUE_LENGTH)
        {
            ++can_diagnostics.rx_overflows;
            continue;
        }
        can_rx_queue[head & (BSP_CAN_RX_QUEUE_LENGTH - 1U)] = frame;
        __DMB();
        can_rx_head = (uint8_t)(head + 1U);
    }
    CAN_RecordRxOverflow();
}
#endif
