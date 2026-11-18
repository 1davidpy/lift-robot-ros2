#ifndef GPS_OUTPUT_H
#define GPS_OUTPUT_H

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
