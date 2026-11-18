#ifndef SERVO_JOG_TASK_H
#define SERVO_JOG_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#define SERVO_JOG_TASK_NAME          "servo_jog"
#define SERVO_JOG_TASK_STACK_WORDS   640U
#define SERVO_JOG_TASK_PRIORITY      (tskIDLE_PRIORITY + 2U)

#define SERVO_JOG_DIAGNOSTIC_SIGNATURE  0x4A4F4760U
#define SERVO_JOG_DIAGNOSTIC_VERSION    31U

typedef enum
{
    SERVO_JOG_STATE_BOOT = 0U,
    SERVO_JOG_STATE_CONFIGURING = 1U,
    SERVO_JOG_STATE_READY = 2U,
    SERVO_JOG_STATE_ENABLING = 3U,
    SERVO_JOG_STATE_CLOCKWISE = 4U,
    SERVO_JOG_STATE_COUNTERCLOCKWISE = 5U,
    SERVO_JOG_STATE_STOPPED = 6U,
    SERVO_JOG_STATE_LOWER_LIMIT = 7U,
    SERVO_JOG_STATE_UPPER_LIMIT = 8U,
    SERVO_JOG_STATE_COMMUNICATION_ERROR = 0xE1U,
    SERVO_JOG_STATE_DRIVE_FAULT = 0xE2U,
} ServoJogState;

typedef enum
{
    SERVO_TARGET_STATUS_IDLE = 0U,
    SERVO_TARGET_STATUS_QUEUED = 1U,
    SERVO_TARGET_STATUS_MOVING = 2U,
    SERVO_TARGET_STATUS_REACHED = 3U,
    SERVO_TARGET_STATUS_REJECTED_NO_POSITION = 0xE1U,
    SERVO_TARGET_STATUS_CANCELED_BY_KEY = 0xE2U,
    SERVO_TARGET_STATUS_DRIVE_ERROR = 0xE3U,
    SERVO_TARGET_STATUS_CANCELED_BY_SERIAL = 0xE4U,
    SERVO_TARGET_STATUS_TIMEOUT = 0xE5U,
    SERVO_TARGET_STATUS_LIMIT_BLOCKED = 0xE6U,
} ServoTargetStatus;

typedef struct
{
    uint32_t signature;
    uint32_t version;
    uint32_t state;
    uint32_t last_result;
    uint32_t modbus_exception;
    uint32_t last_function;
    uint32_t last_register_address;
    uint32_t last_write_value;
    uint32_t direction_selection;
    uint32_t clockwise_command;
    uint32_t counterclockwise_command;
    uint32_t requested_command;
    uint32_t active_command;
    uint32_t rated_speed_rpm;
    uint32_t manual_speed_rpm;
    uint32_t automatic_speed_rpm;
    int32_t speed_rpm;
    uint32_t status_word;
    uint32_t fault_code;
    uint32_t motion_start_count;
    uint32_t communication_errors;
    uint32_t brake_released;
    uint32_t commanded_speed;
    uint32_t position_sensor_raw_mask;
    uint32_t position_sensor_raw_events;
    uint32_t position_sensor_mask;
    uint32_t actual_position;
    uint32_t last_position;
    uint32_t position_change_count;
    uint32_t position_sensor_conflicts;
    uint32_t position_rejected_events;
    uint32_t position_sensor_fault_active;
    uint32_t lower_limit_latched;
    uint32_t upper_limit_latched;
    uint32_t limit_stop_count;
    uint32_t target_position;
    uint32_t target_status;
    int32_t encoder_count;
    int32_t height_um;
    int32_t turns_milli;
    uint32_t position_valid;
    uint32_t saved_mask;
    uint32_t home_calibrated;
    uint32_t home_phase;
    uint32_t command_result;
    uint32_t motor_code;
    uint32_t command_units_per_rev;
    int32_t saved_points[5];
    int32_t target_height_um;
    uint32_t target_height_valid;
    uint32_t height_reply_sequence;
    int32_t height_reply_mm;
    uint32_t height_reply_result;
    int32_t stop_travel_um;
    uint32_t stop_elapsed_ms;
    uint32_t stop_emergency;
    uint32_t absolute_stage;
    uint32_t absolute_read_valid;
    uint32_t absolute_low;
    uint32_t absolute_high;
    uint32_t absolute_system;
    uint32_t absolute_turns;
    uint32_t absolute_single;
    uint32_t mechanical_low;
    uint32_t mechanical_high;
    uint32_t recovery_mode;
    uint32_t motion_inhibited; /* Speed/segment tracking failure; no automatic retry. */
    int32_t operating_min_um;
    int32_t operating_max_um;
    uint32_t profile_flags;
    uint32_t position_sample_tick;
    uint32_t active_source; /* 0 none, 1 legacy local, 2 UART protocol, 3 keys, 4 boot. */
    uint32_t route_active;
    int32_t lower_edge_um;
    int32_t upper_edge_um;
    uint32_t profile_cruise_rpm;
    uint32_t edge_capture_mask;
    int32_t edge2_um;
    int32_t edge5_um;
    uint32_t edge2_age_ms;
    uint32_t edge5_age_ms;
    uint32_t guard_reason; /* 1 invalid, 2 overspeed, 4 overrun, 8 continuation. */
    int32_t guard_speed_rpm;
    uint32_t guard_cap_rpm;
    int32_t guard_error_units;
    int32_t guard_height_um;
    int32_t position_speed_rpm;
    uint32_t position_speed_valid;
    uint32_t ready_for_command;
    uint32_t reference_pending;
    uint32_t reset_pending;
} ServoJogDiagnostics;

typedef enum
{
    LIFT_CMD_GOTO = 1, LIFT_CMD_SET, LIFT_CMD_HEIGHT, LIFT_CMD_HOME,
    LIFT_CMD_LEARN, LIFT_CMD_REFERENCE3, LIFT_CMD_SLOW, LIFT_CMD_FAST,
    LIFT_CMD_HOME_UP, LIFT_CMD_ABS_CHECK, LIFT_CMD_ABS_READ, LIFT_CMD_RECOVER,
    LIFT_CMD_HEIGHT_UM, LIFT_CMD_SPEED, LIFT_CMD_BOOT_HOME,
    LIFT_CMD_LIMIT_LOWER, LIFT_CMD_LIMIT_UPPER, LIFT_CMD_VERIFY_LIMITS,
    LIFT_CMD_PROFILE_SLOW, LIFT_CMD_PROBE_UM
} LiftCommand;

/* Commands run exclusively in the servo task; no other task uses RS485. */
bool ServoJog_QueueCommand(LiftCommand command, int32_t value);
bool ServoJog_QueueRemoteCommand(LiftCommand command, int32_t value, uint16_t token);
bool ServoJog_TakeRemoteReply(uint16_t *token, uint32_t *result);
bool ServoJog_RemoteActive(void);
void ServoJog_CancelRemote(void);

extern volatile ServoJogDiagnostics g_servo_jog_diagnostics;

/**
 * @brief Queue an automatic move to the saved centre point.
 * @param target_position Must be 3 in the encoder-only configuration.
 * @return true when the request is valid and queued.
 */
bool ServoJog_RequestTarget(uint8_t target_position);

/**
 * @brief Queue an immediate stop for automatic serial motion.
 */
void ServoJog_RequestStop(void);

/** @brief Schedule a calibration-preserving MCU reset only while locked/stopped. */
bool ServoJog_RequestReset(void);

/**
 * @brief Create the RS485 hold-to-run JOG task.
 */
void ServoJog_TaskCreate(void);

#endif /* SERVO_JOG_TASK_H */
