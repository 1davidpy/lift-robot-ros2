#ifndef DRIVER_TEST_USART_H
#define DRIVER_TEST_USART_H
#include <stdint.h>
typedef struct { volatile uint32_t SR, DR; } USART_TypeDef;
extern USART_TypeDef test_uart;
typedef struct {
    uint32_t USART_BaudRate, USART_WordLength, USART_StopBits;
    uint32_t USART_Parity, USART_Mode, USART_HardwareFlowControl;
} USART_InitTypeDef;
#define USART_FLAG_PE 0x01U
#define USART_FLAG_FE 0x02U
#define USART_FLAG_NE 0x04U
#define USART_FLAG_ORE 0x08U
#define USART_FLAG_RXNE 0x20U
#define USART_FLAG_TC 0x40U
#define USART_FLAG_TXE 0x80U
#define USART_IT_RXNE USART_FLAG_RXNE
#define USART_WordLength_8b 0U
#define USART_StopBits_1 0U
#define USART_Parity_No 0U
#define USART_Mode_Rx 4U
#define USART_Mode_Tx 8U
#define USART_HardwareFlowControl_None 0U
#define RESET 0
#define SET 1
#define ENABLE 1
int USART_GetFlagStatus(USART_TypeDef *uart, uint16_t flag);
void USART_SendData(USART_TypeDef *uart, uint16_t data);
uint16_t USART_ReceiveData(USART_TypeDef *uart);
void USART_StructInit(USART_InitTypeDef *init);
void USART_Init(USART_TypeDef *uart, USART_InitTypeDef *init);
void USART_ITConfig(USART_TypeDef *uart, uint16_t interrupt, int enable);
void USART_Cmd(USART_TypeDef *uart, int enable);
void NVIC_SetPriority(unsigned irq, unsigned priority);
void NVIC_EnableIRQ(unsigned irq);
#endif
