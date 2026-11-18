#ifndef GPS_OUTPUT_H
#define GPS_OUTPUT_H

#include <stdint.h>

#define GPS_USART_RECEIVE_LENGTH 200U

extern uint8_t GPS_USART_RECEIVE_BUFFER[GPS_USART_RECEIVE_LENGTH];
extern volatile uint16_t GPS_USART_RECEIVE_STATE;

/**
 * @brief Initialize the USART used to transmit simulated GPS data.
 */
void GPS_OutputInit(void);

/**
 * @brief Advance simulated UTC time and request one GPRMC transmission.
 */
void GPS_OutputOnSecondTick(void);

/**
 * @brief Transmit a pending GPRMC sentence from thread context.
 */
void GPS_OutputProcess(void);

#endif

