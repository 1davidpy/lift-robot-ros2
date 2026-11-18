"""Compile the production UART TX/IRQ functions against a host USART stub."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
source = (root / 'Common/BSP/Peripherals/UART/bsp_uart.c').read_text()
write = source[source.index('BspUartResult BSP_UART_DebugWrite('):source.index('bool BSP_UART_DebugTryRead(')]
irq = source[source.index('void BSP_DEBUG_UART_IRQ_HANDLER(void)'):]
preamble = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define BSP_UART_ASYNC_TX 1
#define BSP_UART_TX_BUFFER_SIZE 256U
#define BSP_UART_RX_BUFFER_SIZE 128U
#define BSP_DEBUG_UART 0
#define BSP_DEBUG_UART_IRQ_HANDLER test_irq
#define USART_IT_RXNE 1
#define USART_IT_TXE 2
#define RESET 0
#define ENABLE 1
#define DISABLE 0
typedef enum { BSP_UART_OK, BSP_UART_INVALID_ARGUMENT, BSP_UART_TIMEOUT } BspUartResult;
static uint8_t uart_tx_buffer[256], uart_rx_buffer[128];
static volatile uint16_t uart_tx_head, uart_tx_tail;
static volatile uint8_t uart_rx_head, uart_rx_tail;
static volatile uint32_t uart_rx_overflow_count;
static int tx_irq, rx_ready, sent;
static uint8_t wire[2048];
static void USART_ITConfig(int uart, int bit, int value) { (void)uart; assert(bit==USART_IT_TXE); tx_irq=value; }
static int USART_GetITStatus(int uart,int bit) { (void)uart;return bit==USART_IT_TXE ? tx_irq : rx_ready; }
static void USART_SendData(int uart, uint16_t value) { (void)uart; wire[sent++]=(uint8_t)value; }
static uint16_t USART_ReceiveData(int uart) { (void)uart;rx_ready=0;return 0x42; }
'''
main = r'''
int main(void) {
 uint8_t data[256]; for(int i=0;i<256;i++) data[i]=(uint8_t)i;
 assert(BSP_UART_DebugWrite(NULL,1)==BSP_UART_INVALID_ARGUMENT);
 assert(BSP_UART_DebugWrite(data,0)==BSP_UART_INVALID_ARGUMENT);
 assert(BSP_UART_DebugWrite(data,256)==BSP_UART_TIMEOUT);
 assert(!tx_irq && uart_tx_head==0);
 assert(BSP_UART_DebugWrite(data,255)==BSP_UART_OK);
 assert(sent==0 && tx_irq);
 assert(BSP_UART_DebugWrite(data,1)==BSP_UART_TIMEOUT);
 assert(uart_tx_head==255);
 rx_ready=1;test_irq();assert(uart_rx_head==1 && uart_rx_buffer[0]==0x42 && sent==1);
 while(tx_irq)test_irq(); assert(sent==255 && memcmp(wire,data,255)==0);
 assert(BSP_UART_DebugWrite(data,60)==BSP_UART_OK);
 for(int i=0;i<30;i++)test_irq();
 assert(BSP_UART_DebugWrite(data+60,100)==BSP_UART_OK);
 while(tx_irq)test_irq();
 assert(sent==415 && memcmp(wire+255,data,160)==0);
 test_irq(); assert(sent==415);
 rx_ready=1;test_irq();assert(uart_rx_head==2 && sent==415);
 assert(BSP_UART_DebugWrite(data,14)==BSP_UART_OK);while(tx_irq)test_irq();
 assert(sent==429 && memcmp(wire+415,data,14)==0);
 puts("PASS: nonblocking enqueue, atomic rejection, FIFO/wrap, RX+TX IRQ, idle restart");
 return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='lift-uart-tx-') as directory:
 path=Path(directory)
 (path/'test.c').write_text(preamble+write+irq+main)
 subprocess.run([os.environ.get('TCC','tcc'),str(path/'test.c'),'-o',str(path/'test.exe')],check=True)
 subprocess.run([str(path/'test.exe')],check=True)
