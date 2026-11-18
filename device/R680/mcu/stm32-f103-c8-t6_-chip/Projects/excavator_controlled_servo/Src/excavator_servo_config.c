#include "excavator_servo_config.h"
#include "excavator_machine_limits.h"

const char g_excavator_machine_profile_id[] = EXCAVATOR_MACHINE_PROFILE_ID;

const ExcavatorPositionAxisParameters
    g_excavator_position_axis_parameters[EXCAVATOR_POSITION_AXIS_COUNT] = {
        [EXCAVATOR_AXIS_BOOM] = {
            .servo_id = EXCAVATOR_SERVO_BOOM_ID,
            .minimum_position = EXCAVATOR_BOOM_POSITION_MIN,
            .maximum_position = EXCAVATOR_BOOM_POSITION_MAX,
            .positive_direction = -1,
            .wireless_speed = EXCAVATOR_BOOM_WIRELESS_SPEED,
            .supports_limits = true,
            .default_limits_enabled = EXCAVATOR_BOOM_LIMITS_ENABLED,
            .reference_mode = EXCAVATOR_BOOM_REFERENCE_MODE,
            .encoder_offset = EXCAVATOR_BOOM_ENCODER_OFFSET,
        },
        [EXCAVATOR_AXIS_ARM] = {
            .servo_id = EXCAVATOR_SERVO_ARM_ID,
            .minimum_position = EXCAVATOR_ARM_POSITION_MIN,
            .maximum_position = EXCAVATOR_ARM_POSITION_MAX,
            .positive_direction = 1,
            .wireless_speed = EXCAVATOR_SERVO_WIRELESS_SPEED,
            .supports_limits = true,
            .default_limits_enabled = EXCAVATOR_ARM_LIMITS_ENABLED,
            .reference_mode = EXCAVATOR_ARM_REFERENCE_MODE,
            .encoder_offset = EXCAVATOR_ARM_ENCODER_OFFSET,
        },
        [EXCAVATOR_AXIS_PITCH] = {
            .servo_id = EXCAVATOR_SERVO_BUCKET_ID,
            .minimum_position = EXCAVATOR_PITCH_POSITION_MIN,
            .maximum_position = EXCAVATOR_PITCH_POSITION_MAX,
            .positive_direction = 1,
            .wireless_speed = EXCAVATOR_PITCH_WIRELESS_SPEED,
            .supports_limits = true,
            .default_limits_enabled = EXCAVATOR_PITCH_LIMITS_ENABLED,
            .reference_mode = EXCAVATOR_PITCH_REFERENCE_MODE,
            .encoder_offset = EXCAVATOR_PITCH_ENCODER_OFFSET,
        },
        [EXCAVATOR_AXIS_SWING] = {
            .servo_id = EXCAVATOR_SERVO_SWING_ID,
            .minimum_position = 0U,
            .maximum_position = 4095U,
            .positive_direction = 1,
            .wireless_speed = EXCAVATOR_SWING_WIRELESS_SPEED,
            .supports_limits = false,
            .default_limits_enabled = false,
        },
        [EXCAVATOR_AXIS_GRIPPER] = {
            .servo_id = EXCAVATOR_SERVO_GRIPPER_ID,
            .minimum_position = EXCAVATOR_GRIPPER_POSITION_MIN,
            .maximum_position = EXCAVATOR_GRIPPER_POSITION_MAX,
            .positive_direction = 1,
            .wireless_speed = EXCAVATOR_GRIPPER_WIRELESS_SPEED,
            .supports_limits = true,
            .default_limits_enabled = EXCAVATOR_GRIPPER_LIMITS_ENABLED,
            .reference_mode = EXCAVATOR_GRIPPER_REFERENCE_MODE,
            .encoder_offset = EXCAVATOR_GRIPPER_ENCODER_OFFSET,
        },
};

const ExcavatorWheelAxisParameters
    g_excavator_wheel_axis_parameters[EXCAVATOR_WHEEL_AXIS_COUNT] = {
        [EXCAVATOR_WHEEL_LEFT] = {
            .servo_id = EXCAVATOR_SERVO_LEFT_WHEEL_ID,
            .forward_direction = -1,
        },
        [EXCAVATOR_WHEEL_RIGHT] = {
            .servo_id = EXCAVATOR_SERVO_RIGHT_WHEEL_ID,
            .forward_direction = 1,
        },
};
