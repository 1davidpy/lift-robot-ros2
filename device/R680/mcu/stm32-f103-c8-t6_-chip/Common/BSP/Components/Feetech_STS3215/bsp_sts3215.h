#ifndef BSP_STS3215_H
#define BSP_STS3215_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_STS3215_MAX_SYNC_SERVOS  16U
#define BSP_STS3215_POSITION_MIN     0U
#define BSP_STS3215_POSITION_MAX     4095U
#define BSP_STS3215_MODE_POSITION    0U
#define BSP_STS3215_MODE_WHEEL       1U

/**
 * @brief STS3215 driver operation result.
 */
typedef enum
{
    BSP_STS3215_OK = 0,
    BSP_STS3215_INVALID_ARGUMENT,
    BSP_STS3215_TIMEOUT,
    BSP_STS3215_PROTOCOL_ERROR,
} BspSts3215Result;

/**
 * @brief One STS3215 position command used in a synchronous write.
 */
typedef struct
{
    uint8_t id;
    uint16_t position;
    uint16_t speed;
    uint8_t acceleration;
} BspSts3215PositionCommand;

/** @brief Receive diagnostics; counters are cumulative since UART init. */
typedef struct
{
    uint32_t rx_bytes;
    uint32_t overruns;
    uint32_t framing_errors;
    uint32_t buffer_drops;
    uint32_t timeouts;
} BspSts3215Diagnostics;

/**
 * @brief Read a snapshot of UART receive diagnostics.
 * @param diagnostics Destination for cumulative receive counters.
 */
void BSP_STS3215_GetDiagnostics(BspSts3215Diagnostics *diagnostics);

/**
 * @brief Initialize the UART used by the STS3215 servo bus.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_Init(void);

/**
 * @brief Send simultaneous position commands to multiple STS3215 servos.
 * @param commands Position command array.
 * @param count Number of servos in the command array.
 * @return Driver operation result.
 * @warning Each servo on the bus must have a unique ID.
 */
BspSts3215Result BSP_STS3215_SyncWritePosition(const BspSts3215PositionCommand *commands,
                                                uint8_t count);

/**
 * @brief Read the current position reported by one STS3215 servo.
 * @param id Servo ID.
 * @param position Destination for the current position.
 * @return Driver operation result.
 * @warning The servo bus return signal must be connected to the configured RX pin.
 */
/**
 * @brief Write one bounded calibration position and verify the goal registers.
 * @param command Position command; allowed IDs and bounds are supplied by the board.
 * @return Transfer result; unavailable without a board calibration policy.
 */
BspSts3215Result BSP_STS3215_WriteCalibrationPosition(const BspSts3215PositionCommand *command);

/**
 * @brief Write and verify a position within the board's separate remote follow window.
 * @param command Position command subject to the board ID, speed and fresh-feedback guards.
 * @return Transfer result; unavailable without an explicit remote step policy.
 */
BspSts3215Result BSP_STS3215_WriteRemotePosition(const BspSts3215PositionCommand *command);

/**
 * @brief Read the position goal register.
 * @param id Servo address.
 * @param position Destination for the unsigned position goal.
 * @return Transfer result.
 */
BspSts3215Result BSP_STS3215_ReadGoalPosition(uint8_t id, uint16_t *position);

/**
 * @brief Read the raw encoder offset stored by the servo; does not recalibrate it.
 * @param id Servo address.
 * @param offset Destination for the raw offset word.
 * @return Transfer result.
 */
BspSts3215Result BSP_STS3215_ReadPositionOffset(uint8_t id, uint16_t *offset);

/**
 * @brief Read the two position-loop deadband registers without changing servo tuning.
 * @param id Servo address.
 * @param positive Destination for register 26.
 * @param negative Destination for register 27.
 * @return Transfer result.
 */
BspSts3215Result BSP_STS3215_ReadPositionDeadband(uint8_t id, uint8_t *positive, uint8_t *negative);

/**
 * @brief Read two adjacent bytes as a little-endian word without changing servo registers.
 * @param id Servo address.
 * @param address First register address, 0 through 254.
 * @param value Destination for the raw word.
 * @return Transfer result.
 */
BspSts3215Result BSP_STS3215_ReadRegisterWord(uint8_t id, uint8_t address, uint16_t *value);

BspSts3215Result BSP_STS3215_ReadPresentPosition(uint8_t id, uint16_t *position);

/**
 * @brief Read the configured operating mode.
 * @param id Servo ID.
 * @param mode Destination for the mode value.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_ReadOperatingMode(uint8_t id, uint8_t *mode);

/**
 * @brief Read the raw goal-speed register including the direction bit.
 * @param id Servo ID.
 * @param speed Destination for the commanded speed value.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_ReadGoalSpeed(uint8_t id, uint16_t *speed);

/**
 * @brief Read the torque enable register (0 or 1).
 * @param id Servo ID.
 * @param enabled Destination for torque enable state.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_ReadTorqueEnabled(uint8_t id, uint8_t *enabled);

/**
 * @brief Enable or disable torque without changing any position goal.
 * @param id Servo ID.
 * @param enabled Torque state, restricted to 0 or 1.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_SetTorqueEnabled(uint8_t id, uint8_t enabled);

/**
 * @brief Set the operating mode.
 * @param id Servo ID.
 * @param mode Operating mode value.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_SetOperatingMode(uint8_t id, uint8_t mode);

/**
 * @brief Write only the goal-speed register without changing goal position.
 * @param id Servo ID.
 * @param speed Unsigned goal speed.
 * @return Driver operation result.
 */
BspSts3215Result BSP_STS3215_WriteGoalSpeed(uint8_t id, uint16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* BSP_STS3215_H */
