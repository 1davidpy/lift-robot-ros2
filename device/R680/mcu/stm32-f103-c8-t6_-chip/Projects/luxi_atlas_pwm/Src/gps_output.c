#include "gps_output.h"

#include <stdint.h>

#include "bsp_pin.h"

#define GPS_USART_BAUD_RATE        9600U
#define GPS_SECONDS_PER_DAY        86400U

static volatile uint32_t gps_utc_seconds = 0U;
static volatile uint8_t gps_sentence_pending = 0U;

static char HexDigit(uint8_t value)
{
    return (value < 10U) ? (char)('0' + value) : (char)('A' + value - 10U);
}

static void UsartWrite(const char *text)
{
    while (*text != '\0')
    {
        while ((BSP_GPS_USART->SR & USART_SR_TXE) == 0U)
        {
        }

        BSP_GPS_USART->DR = (uint8_t)*text;
        text++;
    }

    while ((BSP_GPS_USART->SR & USART_SR_TC) == 0U)
    {
    }
}

static void SendGprmc(uint32_t utc_seconds)
{
    char sentence[] =
        "$GPRMC,000000.00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,W,A*00\r\n";
    const uint32_t hours = utc_seconds / 3600U;
    const uint32_t minutes = (utc_seconds / 60U) % 60U;
    const uint32_t seconds = utc_seconds % 60U;
    uint8_t checksum = 0U;
    uint32_t index;

    sentence[7] = (char)('0' + (hours / 10U));
    sentence[8] = (char)('0' + (hours % 10U));
    sentence[9] = (char)('0' + (minutes / 10U));
    sentence[10] = (char)('0' + (minutes % 10U));
    sentence[11] = (char)('0' + (seconds / 10U));
    sentence[12] = (char)('0' + (seconds % 10U));

    for (index = 1U; sentence[index] != '*'; index++)
    {
        checksum ^= (uint8_t)sentence[index];
    }

    sentence[index + 1U] = HexDigit((uint8_t)(checksum >> 4U));
    sentence[index + 2U] = HexDigit((uint8_t)(checksum & 0x0FU));
    UsartWrite(sentence);
}

void GPS_OutputInit(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    BSP_GPS_USART->CR1 = 0U;
    BSP_GPS_USART->CR2 = 0U;
    BSP_GPS_USART->CR3 = 0U;
    BSP_GPS_USART->BRR = (SystemCoreClock + (GPS_USART_BAUD_RATE / 2U)) /
                         GPS_USART_BAUD_RATE;
    BSP_GPS_USART->CR1 = USART_CR1_TE | USART_CR1_UE;
}

void GPS_OutputOnSecondTick(void)
{
    gps_utc_seconds++;
    if (gps_utc_seconds >= GPS_SECONDS_PER_DAY)
    {
        gps_utc_seconds = 0U;
    }

    gps_sentence_pending = 1U;
}

void GPS_OutputProcess(void)
{
    uint32_t utc_seconds;

    if (gps_sentence_pending == 0U)
    {
        return;
    }

    __disable_irq();
    utc_seconds = gps_utc_seconds;
    gps_sentence_pending = 0U;
    __enable_irq();

    SendGprmc(utc_seconds);
}
