#ifndef EXCAVATOR_ARM_CONFIG_H
#define EXCAVATOR_ARM_CONFIG_H

#include <stdint.h>

#define EXCAVATOR_ARM_BOOM_SERVO_ID           1U
#define EXCAVATOR_ARM_ARM_SERVO_ID            2U
#define EXCAVATOR_ARM_BUCKET_SERVO_ID         3U

#define EXCAVATOR_ARM_AXIS_COUNT               3U
#define EXCAVATOR_ARM_CONTROL_PERIOD_MS        20U
#define EXCAVATOR_ARM_FEEDBACK_PERIOD_MS       100U
#define EXCAVATOR_ARM_STATUS_PERIOD_MS         100U
#define EXCAVATOR_ARM_JOG_TIMEOUT_MS           200U
#define EXCAVATOR_ARM_SPEED_MIN                1U
#define EXCAVATOR_ARM_SPEED_MAX                3400U
#define EXCAVATOR_ARM_DEFAULT_SPEED            300U
#define EXCAVATOR_ARM_DEFAULT_ACCELERATION     100U
#define EXCAVATOR_ARM_WIRELESS_SPEED           410U

#define EXCAVATOR_ARM_AXIS_BOOM                 0U
#define EXCAVATOR_ARM_AXIS_ARM                  1U
#define EXCAVATOR_ARM_AXIS_BUCKET               2U

/**
 * @brief Fixed mechanical parameters for one excavator servo axis.
 */
typedef struct
{
    uint8_t servo_id;
    uint16_t minimum_position;
    uint16_t maximum_position;
    int8_t positive_direction;
} ExcavatorArmAxisParameters;

extern const ExcavatorArmAxisParameters
    g_excavator_arm_axis_parameters[EXCAVATOR_ARM_AXIS_COUNT];

#endif /* EXCAVATOR_ARM_CONFIG_H */
