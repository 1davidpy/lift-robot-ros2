#include "excavator_arm_config.h"

const ExcavatorArmAxisParameters
    g_excavator_arm_axis_parameters[EXCAVATOR_ARM_AXIS_COUNT] = {
        [EXCAVATOR_ARM_AXIS_BOOM] = {
            .servo_id = EXCAVATOR_ARM_BOOM_SERVO_ID,
            .minimum_position = 949U,
            .maximum_position = 2640U,
            .positive_direction = -1,
        },
        [EXCAVATOR_ARM_AXIS_ARM] = {
            .servo_id = EXCAVATOR_ARM_ARM_SERVO_ID,
            .minimum_position = 1174U,
            .maximum_position = 3123U,
            .positive_direction = -1,
        },
        [EXCAVATOR_ARM_AXIS_BUCKET] = {
            .servo_id = EXCAVATOR_ARM_BUCKET_SERVO_ID,
            .minimum_position = 897U,
            .maximum_position = 3137U,
            .positive_direction = -1,
        },
};
