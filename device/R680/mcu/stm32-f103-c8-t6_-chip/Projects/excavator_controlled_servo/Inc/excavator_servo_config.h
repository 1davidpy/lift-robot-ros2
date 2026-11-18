#ifndef EXCAVATOR_SERVO_CONFIG_H
#define EXCAVATOR_SERVO_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define EXCAVATOR_SERVO_BOOM_ID                 1U
#define EXCAVATOR_SERVO_ARM_ID                  2U
#define EXCAVATOR_SERVO_BUCKET_ID               3U
#define EXCAVATOR_SERVO_SWING_ID                4U
#define EXCAVATOR_SERVO_LEFT_WHEEL_ID           5U
#define EXCAVATOR_SERVO_RIGHT_WHEEL_ID          6U
#define EXCAVATOR_SERVO_GRIPPER_ID              8U

#define EXCAVATOR_POSITION_AXIS_COUNT            5U
/* Config2 uses stable neutral startup preparation and bounded position channels. */
#ifndef EXCAVATOR_POSITION_REMOTE_MODE
#define EXCAVATOR_POSITION_REMOTE_MODE           1U
#endif
#define EXCAVATOR_REMOTE_POSITION_SPEED         100U
#define EXCAVATOR_REMOTE_SPEED_MAX              500U
#define EXCAVATOR_REMOTE_POSITION_STEP_MAX        10U
#define EXCAVATOR_REMOTE_POSITION_WINDOW_MAX     96U
#define EXCAVATOR_REMOTE_POSITION_SAMPLE_MARGIN   4U
#define EXCAVATOR_POSITION_SETTLED_TOLERANCE       6U
#define EXCAVATOR_REMOTE_HOLD_TOLERANCE           16U
#define EXCAVATOR_CALIBRATION_MODE               1U
#define EXCAVATOR_CHASSIS_TEST_MODE              1U
#define EXCAVATOR_CHASSIS_TEST_SPEED_MAX         3000U
#define EXCAVATOR_CHASSIS_TRACK_SPEED            3000U
#define EXCAVATOR_CHASSIS_SWING_SPEED            1000U
#define EXCAVATOR_MANUAL_CALIBRATION              1U
#define EXCAVATOR_CALIBRATION_STEP_MAX            10U
#define EXCAVATOR_CHASSIS_NEUTRAL_MS             1000U
#define EXCAVATOR_CHASSIS_TIMEOUT_MS             200U
#ifndef EXCAVATOR_READ_ONLY_DIAGNOSTICS
#define EXCAVATOR_READ_ONLY_DIAGNOSTICS          0U
#endif
#define EXCAVATOR_CALIBRATION_SPEED_MAX          100U
#define EXCAVATOR_WHEEL_AXIS_COUNT               2U
#define EXCAVATOR_CONTROL_PERIOD_MS              20U
#define EXCAVATOR_FEEDBACK_PERIOD_MS             100U
#define EXCAVATOR_STATUS_PERIOD_MS               100U
#define EXCAVATOR_JOG_TIMEOUT_MS                 200U
#define EXCAVATOR_SERVO_SPEED_MIN                1U
#define EXCAVATOR_SERVO_SPEED_MAX                3400U
#define EXCAVATOR_SERVO_DEFAULT_SPEED            500U
#define EXCAVATOR_SERVO_DEFAULT_ACCELERATION     100U
#define EXCAVATOR_BOOM_WIRELESS_SPEED            250U
#define EXCAVATOR_SERVO_WIRELESS_SPEED           500U
#define EXCAVATOR_PITCH_WIRELESS_SPEED           200U
#define EXCAVATOR_GRIPPER_WIRELESS_SPEED         200U
#define EXCAVATOR_SWING_WIRELESS_SPEED           1000U
#define EXCAVATOR_TRACK_SPEED_MAX                6000U
#define EXCAVATOR_WHEEL_DIRECTION_BIT            0x8000U

#define EXCAVATOR_AXIS_BOOM                       0U
#define EXCAVATOR_AXIS_ARM                        1U
#define EXCAVATOR_AXIS_PITCH                     2U
#define EXCAVATOR_AXIS_SWING                      3U
#define EXCAVATOR_AXIS_GRIPPER                    4U

#define EXCAVATOR_WHEEL_LEFT                      0U
#define EXCAVATOR_WHEEL_RIGHT                     1U

/**
 * @brief Fixed mechanical parameters for one position-controlled servo axis.
 */
typedef struct
{
    uint8_t servo_id;
    uint16_t minimum_position;
    uint16_t maximum_position;
    int8_t positive_direction;
    uint16_t wireless_speed;
    bool supports_limits;
    bool default_limits_enabled;
    uint8_t reference_mode;
    uint16_t encoder_offset;
} ExcavatorPositionAxisParameters;

/**
 * @brief Fixed mechanical parameters for one wheel-mode servo axis.
 */
typedef struct
{
    uint8_t servo_id;
    int8_t forward_direction;
} ExcavatorWheelAxisParameters;

extern const ExcavatorPositionAxisParameters
    g_excavator_position_axis_parameters[EXCAVATOR_POSITION_AXIS_COUNT];
extern const ExcavatorWheelAxisParameters
    g_excavator_wheel_axis_parameters[EXCAVATOR_WHEEL_AXIS_COUNT];
extern const char g_excavator_machine_profile_id[];

#endif /* EXCAVATOR_SERVO_CONFIG_H */
