#ifndef BSP_PD42S1_H
#define BSP_PD42S1_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_PD42S1_SPEED_RPM_MAX      3000.0F
#define BSP_PD42S1_ACCELERATION_MAX   200U
#define BSP_PD42S1_DIRECTION_CW       0U
#define BSP_PD42S1_DIRECTION_CCW      1U
#define BSP_PD42S1_BROADCAST_ADDRESS  0U
#define BSP_PD42S1_PULSES_PER_TURN    51200UL

/**
 * @brief PD42S1 operation result.
 */
typedef enum
{
    BSP_PD42S1_OK = 0,
    BSP_PD42S1_INVALID_ARGUMENT,
    BSP_PD42S1_CAN_ERROR,
    BSP_PD42S1_TIMEOUT,
    BSP_PD42S1_PROTOCOL_ERROR,
} BspPd42s1Result;

/**
 * @brief Initialize the CAN bus used by PD42S1 drivers.
 * @return Driver operation result.
 */
BspPd42s1Result BSP_PD42S1_Init(void);

/**
 * @brief Run one motor continuously in closed-loop speed mode.
 * @param address Motor address, or BSP_PD42S1_BROADCAST_ADDRESS for all motors.
 * @param direction BSP_PD42S1_DIRECTION_CW or BSP_PD42S1_DIRECTION_CCW.
 * @param acceleration Acceleration from 0 to 200 RPM/s^2; zero starts directly.
 * @param speed_rpm Target speed from 0 to 3000 RPM.
 * @return Driver operation result.
 * @warning A broadcast command does not produce a device response.
 */
BspPd42s1Result BSP_PD42S1_SetSpeed(uint8_t address,
                                    uint8_t direction,
                                    uint8_t acceleration,
                                    float speed_rpm);

/**
 * @brief Move one motor to an absolute encoder position.
 * @param address Motor address.
 * @param direction BSP_PD42S1_DIRECTION_CW or BSP_PD42S1_DIRECTION_CCW.
 * @param acceleration Acceleration from 0 to 200 RPM/s^2; zero starts directly.
 * @param speed_rpm Maximum target speed from 1 to 3000 RPM.
 * @param position_pulses Position magnitude; 51200 pulses represent one turn.
 * @return Driver operation result.
 */
BspPd42s1Result BSP_PD42S1_SetAbsolutePosition(uint8_t address,
                                               uint8_t direction,
                                               uint8_t acceleration,
                                               uint16_t speed_rpm,
                                               uint32_t position_pulses);

/**
 * @brief Make the motor's current encoder position the zero position.
 * @param address Motor address.
 * @return Driver operation result.
 */
BspPd42s1Result BSP_PD42S1_ZeroPosition(uint8_t address);

/**
 * @brief Clear stall, disable, and braking states for one motor.
 * @param address Motor address.
 * @return Driver operation result.
 */
BspPd42s1Result BSP_PD42S1_ClearState(uint8_t address);

/**
 * @brief Stop one motor immediately.
 * @param address Motor address.
 * @return Driver operation result.
 * @warning Clear the braking state after the motor has stopped to avoid heating.
 */
BspPd42s1Result BSP_PD42S1_Stop(uint8_t address);

/**
 * @brief Read the signed real-time motor speed.
 * @param address Motor address.
 * @param speed_rpm Destination for the signed RPM value.
 * @return Driver operation result.
 */
BspPd42s1Result BSP_PD42S1_ReadSpeed(uint8_t address, int16_t *speed_rpm);

/**
 * @brief Read the signed real-time encoder position.
 * @param address Motor address.
 * @param position_pulses Destination for the signed encoder position.
 * @return Driver operation result.
 */
BspPd42s1Result BSP_PD42S1_ReadPosition(uint8_t address, int32_t *position_pulses);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PD42S1_H */
