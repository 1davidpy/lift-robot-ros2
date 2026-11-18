#ifndef SERVO_APP_CONFIG_H
#define SERVO_APP_CONFIG_H

/* Motor nameplate rating; this is not the commanded test speed. */
#define SERVO_MOTOR_RATED_SPEED_RPM                3000U

/* PV speed conversion with the factory 10000 command units/revolution. */
#define SERVO_COMMAND_UNITS_PER_REVOLUTION         10000U
#define SERVO_MANUAL_SPEED_RPM                     3000U
#define SERVO_AUTOMATIC_SPEED_RPM                  3000U
#define SERVO_HOMING_SPEED_RPM                     3000U
#define SERVO_HOMING_CALIBRATION_MARKER            3U
#define SERVO_HOMING_FINE_SPEED_RPM                60U
#define SERVO_RECOVERY_SPEED_RPM                   1000U
/* A low speed sample alone never proves standstill: position must stay stable. */
#define SERVO_STOP_FEEDBACK_TOLERANCE_RPM          2
#define SERVO_MANUAL_SPEED_PULSES_PER_SECOND       \
    ((SERVO_MANUAL_SPEED_RPM * SERVO_COMMAND_UNITS_PER_REVOLUTION) / 60U)
#define SERVO_AUTOMATIC_SPEED_PULSES_PER_SECOND    \
    ((SERVO_AUTOMATIC_SPEED_RPM * SERVO_COMMAND_UNITS_PER_REVOLUTION) / 60U)

/* Reach or leave the 3000 rpm manual speed in approximately one second. */
#define SERVO_JOG_ACCELERATION_PULSES_PER_SECOND2  \
    SERVO_MANUAL_SPEED_PULSES_PER_SECOND
#define SERVO_JOG_DECELERATION_PULSES_PER_SECOND2  \
    SERVO_MANUAL_SPEED_PULSES_PER_SECOND
/* Native PP final approach: 20 mm/s^2 at the provisional 1 mm/rev scale.
   P10-31 keeps the faster existing stopping ramp for interrupted moves. */
#define SERVO_PP_DECELERATION_PULSES_PER_SECOND2   200000U

#define SERVO_KEY_SCAN_PERIOD_MS                   10U
#define SERVO_KEY_DEBOUNCE_MS                      30U
#define SERVO_ENABLE_SETTLE_TIME_MS                500U
#define SERVO_DIRECTION_CHANGE_DELAY_MS            1100U
#define SERVO_COMMUNICATION_CHECK_PERIOD_MS        500U
#define SERVO_RECONNECT_PERIOD_MS                  1000U
#define SERVO_COMMAND_RETRY_COUNT                  3U
#define SERVO_BRAKE_RELEASE_SETTLE_TIME_MS         300U
#define SERVO_BRAKE_ZERO_CONFIRM_MS                200U
#define SERVO_BRAKE_STABLE_POSITION_UNITS          20L
#define SERVO_AUTOMATIC_MOVE_TIMEOUT_MS            30000U

/* Bench test: K1 moves down and K2 moves up with the drive's reported sense. */
#define SERVO_REVERSE_DIRECTION_MAPPING            1U

#define SERVO_POSITION_SCAN_PERIOD_MS              10U
#define SERVO_POSITION_ASSERT_DEBOUNCE_MS          20U
#define SERVO_POSITION_RELEASE_DEBOUNCE_MS         50U

#define SERVO_SERIAL_STATUS_PERIOD_MS               500U
#define SERVO_SERIAL_COMMAND_POLL_PERIOD_MS          10U

#endif /* SERVO_APP_CONFIG_H */
