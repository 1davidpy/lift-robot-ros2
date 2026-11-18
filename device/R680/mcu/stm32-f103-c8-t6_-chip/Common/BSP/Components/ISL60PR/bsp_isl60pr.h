#ifndef BSP_ISL60PR_H
#define BSP_ISL60PR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ISL60-PR operation result.
 */
typedef enum
{
    BSP_ISL60PR_OK = 0,
    BSP_ISL60PR_INVALID_ARGUMENT,
    BSP_ISL60PR_TIMEOUT,
    BSP_ISL60PR_IO_ERROR,
    BSP_ISL60PR_BAD_RESPONSE,
    BSP_ISL60PR_CRC_ERROR,
    BSP_ISL60PR_EXCEPTION,
} BspIsl60prResult;

/**
 * @brief ISL60-PR jog command values.
 */
typedef enum
{
    BSP_ISL60PR_JOG_STOP = 0x0100,
    BSP_ISL60PR_JOG_POSITIVE = 0x0010,
    BSP_ISL60PR_JOG_NEGATIVE = 0x0020,
} BspIsl60prJogCommand;

/**
 * @brief Configure the project-provided RS485 UART and leave the bus in receive mode.
 */
void BSP_ISL60PR_Init(void);

/**
 * @brief Read one 16-bit Modbus holding register.
 * @param register_address Servo register address.
 * @param value Destination for the register value.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_ReadRegister(uint16_t register_address, uint16_t *value);

/**
 * @brief Read one 32-bit Modbus value using the drive's low-word-first order.
 * @param register_address Address of the low 16-bit register.
 * @param value Destination for the 32-bit register value.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_ReadRegister32(uint16_t register_address, uint32_t *value);

/**
 * @brief Write one 16-bit Modbus holding register.
 * @param register_address Servo register address.
 * @param value Register value.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_WriteRegister(uint16_t register_address, uint16_t value);

/**
 * @brief Write one 32-bit value using the drive's default low-word-first order.
 * @param register_address Address of the low 16-bit register.
 * @param value 32-bit register value.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_WriteRegister32(uint16_t register_address, uint32_t value);

/**
 * @brief Configure Modbus PP mode for relative-position commands.
 * @param speed_pulses_per_second Positive PP speed magnitude.
 * @param acceleration_pulses_per_second2 PP acceleration.
 * @param deceleration_pulses_per_second2 PP deceleration.
 * @param arrival_window_pulses Position-arrival window in command units.
 * @return Operation result.
 * @warning This selects forced DI control and leaves the servo disabled.
 */
BspIsl60prResult BSP_ISL60PR_ConfigurePosition(uint32_t speed_pulses_per_second,
                                              uint32_t acceleration_pulses_per_second2,
                                              uint32_t deceleration_pulses_per_second2,
                                              uint16_t arrival_window_pulses);

/**
 * @brief Enable or disable the servo through the forced DI image over RS485.
 * @param enabled True to assert the default active-low DI3 S-ON input.
 * @return Operation result.
 * @warning Forced DI mode bypasses physical DI inputs, including travel limits.
 */
BspIsl60prResult BSP_ISL60PR_SetServoEnabled(bool enabled);

/**
 * @brief Start one PP relative-position movement.
 * @param position_delta Signed distance in position command units.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_MoveRelative(int32_t position_delta);

/**
 * @brief Start a PP absolute move in the drive's current coordinate system.
 * @param position Signed target in command units, within +/-1073741824.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_MoveAbsolute(int32_t position);

/**
 * @brief Send and reset the bus motion-stop command.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_Stop(void);

/**
 * @brief Send and reset the bus emergency-stop command.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_EmergencyStop(void);

/**
 * @brief Configure Modbus PV jog mode while preserving physical DI handling.
 * @param speed_pulses_per_second Positive PV speed magnitude.
 * @param acceleration_pulses_per_second2 PV acceleration.
 * @param deceleration_pulses_per_second2 PV deceleration.
 * @param configure_brake_output True to assign DO1 as the brake output.
 * @return Operation result.
 * @warning Physical DI3 must be active before the drive can energize the motor.
 */
BspIsl60prResult BSP_ISL60PR_ConfigureJog(uint32_t speed_pulses_per_second,
                                         uint32_t acceleration_pulses_per_second2,
                                         uint32_t deceleration_pulses_per_second2,
                                         bool configure_brake_output);

/**
 * @brief Configure Modbus PV JOG mode with servo enable controlled over RS485.
 * @param speed_pulses_per_second Initial JOG speed magnitude; zero is allowed.
 * @param acceleration_pulses_per_second2 JOG acceleration.
 * @param deceleration_pulses_per_second2 JOG deceleration.
 * @return Operation result.
 * @warning Forced DI mode bypasses physical DI inputs, including travel limits.
 */
BspIsl60prResult BSP_ISL60PR_ConfigureJogRs485(
    uint32_t speed_pulses_per_second,
    uint32_t acceleration_pulses_per_second2,
    uint32_t deceleration_pulses_per_second2);

/**
 * @brief Set the PV JOG speed magnitude, including zero for a safe idle state.
 * @param speed_pulses_per_second Speed magnitude from zero through 6000000.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_SetJogSpeed(uint32_t speed_pulses_per_second);

/**
 * @brief Send a positive, negative, or stop jog command.
 * @param command Jog command.
 * @return Operation result.
 */
BspIsl60prResult BSP_ISL60PR_Jog(BspIsl60prJogCommand command);

/**
 * @brief Return the latest Modbus exception code.
 * @return Exception byte, or zero if the latest response was not an exception.
 */
uint8_t BSP_ISL60PR_GetLastException(void);

/**
 * @brief Return the function code of the latest Modbus request.
 * @return Latest Modbus function code.
 */
uint8_t BSP_ISL60PR_GetLastFunction(void);

/**
 * @brief Return the register address of the latest Modbus request.
 * @return Latest register address.
 */
uint16_t BSP_ISL60PR_GetLastRegisterAddress(void);

/**
 * @brief Return the value supplied by the latest Modbus write request.
 * @return Latest 16-bit or 32-bit write value.
 */
uint32_t BSP_ISL60PR_GetLastWriteValue(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ISL60PR_H */
