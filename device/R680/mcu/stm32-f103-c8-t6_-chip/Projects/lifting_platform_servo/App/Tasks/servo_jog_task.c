#include "servo_jog_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_brake_relay.h"
#include "bsp_isl60pr.h"
#include "bsp_key.h"
#include "system_led_task.h"
#include "servo_app_config.h"
#include "lift_positions.h"
#include "lift_motion_profile.h"
#include "bsp_position_sensor.h"
#include <limits.h>
#include "stm32f10x.h"

#define SERVO_DIRECTION_SELECTION_REGISTER  0x0202U
#define SERVO_MONITOR_SPEED_REGISTER        0x0B00U
#define SERVO_MONITOR_STATUS_REGISTER       0x0B04U
#define SERVO_MONITOR_FAULT_REGISTER        0x0B22U

#define SERVO_DIRECTION_CCW_IS_POSITIVE     0U
#define SERVO_DIRECTION_CW_IS_POSITIVE      1U
#define SERVO_PENDING_STOP                  0xFFU
#define SERVO_CENTER_SENSOR_MASK            1U

typedef struct
{
    bool stable_pressed;
    bool candidate_pressed;
    bool initialized;
    uint16_t candidate_time_ms;
} ServoKeyDebounce;

volatile ServoJogDiagnostics g_servo_jog_diagnostics = {
    .signature = SERVO_JOG_DIAGNOSTIC_SIGNATURE,
    .version = SERVO_JOG_DIAGNOSTIC_VERSION,
    .state = SERVO_JOG_STATE_BOOT,
    .requested_command = BSP_ISL60PR_JOG_STOP,
    .active_command = BSP_ISL60PR_JOG_STOP,
    .rated_speed_rpm = SERVO_MOTOR_RATED_SPEED_RPM,
    .manual_speed_rpm = LIFT_PROFILE_SLOW_RPM,
    .automatic_speed_rpm = LIFT_PROFILE_SLOW_RPM,
    .target_status = SERVO_TARGET_STATUS_IDLE,
};

static ServoKeyDebounce k1_state;
static ServoKeyDebounce k2_state;
static volatile bool stop_pending;
static volatile bool reset_pending;
static TickType_t reset_requested_at;
static volatile LiftCommand pending_command;
static volatile int32_t pending_value;
static volatile uint8_t pending_source;
static volatile uint16_t pending_token;
static volatile uint8_t processing_source;
static uint16_t processing_token;
static volatile bool remote_owned;
static volatile bool remote_cancel_requested;
static volatile bool remote_reply_ready;
static volatile uint16_t remote_reply_token;
static volatile uint32_t remote_reply_result;
static bool RemoteRouteActive(void);
static bool Stationary(void);

/* Caller holds the task critical section. There is at most one remote request. */
static void CancelQueuedRemote(uint32_t result)
{
    if (pending_command && pending_source == 2U)
    {
        remote_reply_token = pending_token;
        remote_reply_result = result;
        remote_reply_ready = true;
        pending_command = 0;
        remote_owned = false;
    }
}

bool ServoJog_QueueCommand(LiftCommand command, int32_t value)
{
    bool accepted = false;
    taskENTER_CRITICAL();
    if (pending_command == 0 && !stop_pending && !reset_pending)
    {
        pending_value = value;
        pending_source = 1U;
        pending_command = command;
        accepted = true;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

bool ServoJog_QueueRemoteCommand(LiftCommand command, int32_t value, uint16_t token)
{
    bool accepted = false;
    taskENTER_CRITICAL();
    if (!pending_command && !stop_pending && !reset_pending && !remote_owned && !remote_reply_ready &&
        processing_source != 2U)
    {
        pending_source = 2U;
        pending_value = value;
        pending_token = token;
        remote_owned = true;
        remote_cancel_requested = false;
        pending_command = command;
        g_servo_jog_diagnostics.ready_for_command = 0;
        accepted = true;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

bool ServoJog_TakeRemoteReply(uint16_t *token, uint32_t *result)
{
    if (!token || !result) return false;
    taskENTER_CRITICAL();
    bool ready = remote_reply_ready;
    if (ready)
    {
        *token = remote_reply_token;
        *result = remote_reply_result;
        remote_reply_ready = false;
    }
    taskEXIT_CRITICAL();
    return ready;
}

bool ServoJog_RemoteActive(void) { return remote_owned; }

void ServoJog_CancelRemote(void)
{
    taskENTER_CRITICAL();
    bool active = remote_owned && RemoteRouteActive();
    if (remote_owned) remote_cancel_requested = true;
    CancelQueuedRemote(9U); /* CANCELED */
    /* A dequeued UART request may only be waiting to return BUSY while a local
       route runs. Cancel that request without stopping the local route. */
    if (active) stop_pending = true;
    taskEXIT_CRITICAL();
}

bool ServoJog_RequestTarget(uint8_t target)
{
    return target == 3U && ServoJog_QueueCommand(LIFT_CMD_GOTO, target);
}

void ServoJog_RequestStop(void)
{
    taskENTER_CRITICAL();
    reset_pending = false;
    g_servo_jog_diagnostics.reset_pending = 0;
    stop_pending = true;
    CancelQueuedRemote(9U);
    pending_command = 0;
    taskEXIT_CRITICAL();
}

static void ServoJog_UpdateKey(ServoKeyDebounce *state, BspKeyId key)
{
    bool raw_pressed = BSP_Key_IsPressed(key);

    if (raw_pressed != state->candidate_pressed)
    {
        state->candidate_pressed = raw_pressed;
        state->candidate_time_ms = 0U;
        return;
    }

    if (state->candidate_time_ms < SERVO_KEY_DEBOUNCE_MS)
    {
        state->candidate_time_ms =
            (uint16_t)(state->candidate_time_ms + SERVO_KEY_SCAN_PERIOD_MS);
        if (state->candidate_time_ms >= SERVO_KEY_DEBOUNCE_MS)
        {
            state->stable_pressed = state->candidate_pressed;
            state->initialized = true;
        }
    }
}

static bool ServoJog_KeysSettled(void)
{
    return k1_state.initialized && k2_state.initialized;
}

static void ServoJog_SetBrakeReleased(bool released)
{
    BSP_BrakeRelay_SetReleased(released);
    g_servo_jog_diagnostics.brake_released = released ? 1U : 0U;
}

static void ServoJog_UpdateKeys(void)
{
    ServoJog_UpdateKey(&k1_state, BSP_KEY_TOP_LEFT);
    ServoJog_UpdateKey(&k2_state, BSP_KEY_TOP_RIGHT);
}

static void ServoJog_RecordResult(BspIsl60prResult result)
{
    /* Communication health is independent of drive alarms or centre pairing. */
    SystemLED_ReportMotorCommunication(result == BSP_ISL60PR_OK);
    g_servo_jog_diagnostics.last_result = (uint32_t)result;
    g_servo_jog_diagnostics.modbus_exception =
        (uint32_t)BSP_ISL60PR_GetLastException();
    g_servo_jog_diagnostics.last_function =
        (uint32_t)BSP_ISL60PR_GetLastFunction();
    g_servo_jog_diagnostics.last_register_address =
        (uint32_t)BSP_ISL60PR_GetLastRegisterAddress();
    g_servo_jog_diagnostics.last_write_value =
        BSP_ISL60PR_GetLastWriteValue();
}

static BspIsl60prJogCommand ServoJog_GetManualCommand(bool armed)
{
    if (!armed || k1_state.stable_pressed == k2_state.stable_pressed)
    {
        return BSP_ISL60PR_JOG_STOP;
    }

    return k1_state.stable_pressed
               ? (BspIsl60prJogCommand)
                     g_servo_jog_diagnostics.counterclockwise_command
               : (BspIsl60prJogCommand)
                     g_servo_jog_diagnostics.clockwise_command;
}

/* All motion uses bounded PP segments; optical-only scans are disabled. */
#define POSITION_TOLERANCE LIFT_POSITION_TOLERANCE_UNITS
#define HOME_TIMEOUT_MS 300000U
#define MONITOR_PERIOD_MS 50U
#define HOME_SPEED ((SERVO_HOMING_SPEED_RPM * SERVO_COMMAND_UNITS_PER_REVOLUTION) / 60U)
#define PP_SPEED SERVO_AUTOMATIC_SPEED_PULSES_PER_SECOND
#define PP_ACCEL SERVO_JOG_ACCELERATION_PULSES_PER_SECOND2
#define HOME_ACCEL SERVO_JOG_ACCELERATION_PULSES_PER_SECOND2
#define RESULT_OK 0U
#define RESULT_BUSY 1U
#define RESULT_NEED_REFERENCE 2U
#define RESULT_SENSOR 3U
#define RESULT_RANGE 4U
#define RESULT_FLASH 5U
#define RESULT_COMM 6U
#define RESULT_LIMIT 7U
#define RESULT_TIMEOUT 8U
#define RESULT_CANCELED 9U
#define RESULT_SCALE 10U
#define RESULT_PROFILE 11U

typedef struct
{
    LiftPositions saved;
    int32_t origin;
    int32_t count;
    int32_t target;
    int32_t up_sign;
    int32_t home_start;
    int32_t candidate_edge;
    uint16_t mode;
    bool valid;
    bool learning;
    bool finish_home;
    bool save_edge;
    bool configured;
    bool manual_control;
    bool boot_home_checked;
    bool lower_latched;
    bool upper_latched;
    bool lower_return_seen;
    bool upper_return_seen;
    bool sensor_fault_active;
    bool sensor_fault_invalidated;
    TickType_t sensor_clear_since;
    int32_t lower_return_count;
    int32_t upper_return_count;
    uint16_t stopping_direction;
    uint8_t home; /* 1 seek down, 2 leave 3 down, 3 seek 3 up, 4 return center. */
    uint16_t moving;
    TickType_t stopped_at;
    TickType_t started_at;
    TickType_t home_started;
    bool arrival_confirming;
    int32_t arrival_anchor;
    TickType_t arrival_since;
    bool zero_confirming;
    bool stop_tracking;
    int32_t zero_anchor;
    int32_t stop_count;
    TickType_t zero_since;
    bool center_scan;
    int32_t center_entry;
    bool home_fine;
    bool absolute_return;
    bool recovery_mode;
    uint16_t recovery_direction;
    bool route_active;
    bool route_manual;
    bool route_continuing;
    int32_t route_final;
    uint16_t route_direction;
    uint32_t profile_speed;
    bool monitor_initialized;
    bool reference_pending;
    TickType_t last_reference_attempt;
} LiftRuntime;

static LiftRuntime lift;

static bool RemoteRouteActive(void)
{
    return lift.route_active && g_servo_jog_diagnostics.active_source == 2U;
}

static bool StoppedSpeedFeedback(void)
{
    int32_t speed = g_servo_jog_diagnostics.speed_rpm;
    return speed >= -SERVO_STOP_FEEDBACK_TOLERANCE_RPM &&
        speed <= SERVO_STOP_FEEDBACK_TOLERANCE_RPM;
}

static void UpdateStopObservation(void)
{
    if (lift.moving != BSP_ISL60PR_JOG_STOP ||
        !StoppedSpeedFeedback())
        lift.zero_confirming = false;
    else
    {
        int64_t drift = (int64_t)lift.count - lift.zero_anchor;
        if (!lift.zero_confirming || drift > SERVO_BRAKE_STABLE_POSITION_UNITS ||
            drift < -SERVO_BRAKE_STABLE_POSITION_UNITS)
        {
            lift.zero_confirming = true;
            lift.zero_anchor = lift.count;
            lift.zero_since = xTaskGetTickCount();
        }
    }
    if (lift.stop_tracking)
    {
        g_servo_jog_diagnostics.stop_travel_um = (int32_t)(
            ((int64_t)lift.count - lift.stop_count) * lift.up_sign / 10);
        g_servo_jog_diagnostics.stop_elapsed_ms =
            (xTaskGetTickCount() - lift.stopped_at) * portTICK_PERIOD_MS;
    }
}

static bool BrakeCanEngage(void)
{
    return lift.moving == BSP_ISL60PR_JOG_STOP && StoppedSpeedFeedback() && lift.zero_confirming &&
        xTaskGetTickCount() - lift.zero_since >= pdMS_TO_TICKS(SERVO_BRAKE_ZERO_CONFIRM_MS);
}

static bool PositionArrivalConfirmed(void)
{
    int64_t error = (int64_t)lift.target - lift.count;
    if (error < -POSITION_TOLERANCE || error > POSITION_TOLERANCE ||
        !StoppedSpeedFeedback() || !(g_servo_jog_diagnostics.status_word & 1U))
    {
        lift.arrival_confirming = false;
        return false;
    }
    int64_t drift = (int64_t)lift.count - lift.arrival_anchor;
    if (!lift.arrival_confirming || drift > SERVO_BRAKE_STABLE_POSITION_UNITS ||
        drift < -SERVO_BRAKE_STABLE_POSITION_UNITS)
    {
        lift.arrival_confirming = true;
        lift.arrival_anchor = lift.count;
        lift.arrival_since = xTaskGetTickCount();
        return false;
    }
    return xTaskGetTickCount() - lift.arrival_since >= pdMS_TO_TICKS(SERVO_BRAKE_ZERO_CONFIRM_MS);
}

static bool ReadMonitors(void)
{
    uint16_t speed = 0, status = 0, fault = 0;
    uint32_t position;
    TickType_t sample_tick = xTaskGetTickCount();
    BspIsl60prResult r = BSP_ISL60PR_ReadRegister32(0x0B07U, &position);
    if (r == BSP_ISL60PR_OK)
        r = BSP_ISL60PR_ReadRegister(SERVO_MONITOR_SPEED_REGISTER, &speed);
    if (r == BSP_ISL60PR_OK)
        r = BSP_ISL60PR_ReadRegister(SERVO_MONITOR_STATUS_REGISTER, &status);
    if (r == BSP_ISL60PR_OK)
        r = BSP_ISL60PR_ReadRegister(SERVO_MONITOR_FAULT_REGISTER, &fault);
    if (r == BSP_ISL60PR_OK)
    {
        TickType_t interval = sample_tick - g_servo_jog_diagnostics.position_sample_tick;
        bool velocity_valid = lift.monitor_initialized && interval > 0U && interval <= pdMS_TO_TICKS(500U);
        int64_t velocity = velocity_valid ?
            ((int64_t)(int32_t)position - lift.count) * 60000LL /
            ((int64_t)interval * portTICK_PERIOD_MS * SERVO_COMMAND_UNITS_PER_REVOLUTION) : 0;
        if (velocity > INT32_MAX || velocity < INT32_MIN) velocity_valid = false;
        /* Publish one completed RS485 scan; no new timestamp with old height. */
        taskENTER_CRITICAL();
        lift.count = (int32_t)position;
        g_servo_jog_diagnostics.encoder_count = lift.count;
        g_servo_jog_diagnostics.speed_rpm = (int16_t)speed;
        g_servo_jog_diagnostics.position_speed_rpm = velocity_valid ? (int32_t)velocity : 0;
        g_servo_jog_diagnostics.position_speed_valid = velocity_valid;
        lift.monitor_initialized = true;
        g_servo_jog_diagnostics.status_word = status;
        g_servo_jog_diagnostics.fault_code = fault;
        g_servo_jog_diagnostics.position_valid = lift.valid;
        g_servo_jog_diagnostics.height_um = lift.valid ?
            (int32_t)(((int64_t)lift.count - lift.origin) * lift.up_sign / 10) : 0;
        g_servo_jog_diagnostics.position_sample_tick = sample_tick;
        taskEXIT_CRITICAL();
        if (!fault) UpdateStopObservation();
    }
    ServoJog_RecordResult(r);
    return r == BSP_ISL60PR_OK && g_servo_jog_diagnostics.fault_code == 0U;
}

/* P0B-77/79 stay zero on this drive. Use raw P0B-70/71 instead; do not
   infer a signed turn count. Differences are bounded within the 16-bit ring. */
static bool ReadAbsolute(uint64_t *position, uint32_t *resolution)
{
    uint32_t single;
    uint16_t system, turns, check, overflow_mask, fault;
    uint32_t kind = (g_servo_jog_diagnostics.motor_code / 10U) % 10U;
    g_servo_jog_diagnostics.absolute_read_valid = 0;
    g_servo_jog_diagnostics.absolute_system = UINT32_MAX;
    if (BSP_ISL60PR_ReadRegister(0x0201U, &system) != BSP_ISL60PR_OK) return false;
    g_servo_jog_diagnostics.absolute_system = system;
    if (system != 1U || (kind != 2U && kind != 3U)) return false;
    if (BSP_ISL60PR_ReadRegister32(0x001FU, resolution) != BSP_ISL60PR_OK ||
        *resolution != (kind == 2U ? 131072U : 8388608U)) return false;
    if (BSP_ISL60PR_ReadRegister(0x0A24U, &overflow_mask) != BSP_ISL60PR_OK || overflow_mask != 0U ||
        BSP_ISL60PR_ReadRegister(SERVO_MONITOR_FAULT_REGISTER, &fault) != BSP_ISL60PR_OK || fault != 0U)
        return false;
    for (unsigned attempt = 0; attempt < 3U; ++attempt)
    {
        if (BSP_ISL60PR_ReadRegister(0x0B46U, &turns) != BSP_ISL60PR_OK ||
            BSP_ISL60PR_ReadRegister32(0x0B47U, &single) != BSP_ISL60PR_OK ||
            BSP_ISL60PR_ReadRegister(0x0B46U, &check) != BSP_ISL60PR_OK) return false;
        if (turns == check && single < *resolution)
        {
            *position = (uint64_t)turns * *resolution + single;
            g_servo_jog_diagnostics.absolute_turns = turns;
            g_servo_jog_diagnostics.absolute_single = single;
            g_servo_jog_diagnostics.absolute_low = (uint32_t)*position;
            g_servo_jog_diagnostics.absolute_high = (uint32_t)(*position >> 32);
            g_servo_jog_diagnostics.absolute_read_valid = 1U;
            return true;
        }
    }
    return false;
}

static bool AbsoluteOffset(int64_t *offset)
{
    uint64_t position;
    uint32_t resolution;
    if (!lift.saved.absolute_stage ||
        lift.saved.absolute_motor != g_servo_jog_diagnostics.motor_code ||
        !ReadAbsolute(&position, &resolution) || resolution != lift.saved.absolute_resolution)
        return false;
    uint64_t zero = ((uint64_t)lift.saved.absolute_zero_high << 32) | lift.saved.absolute_zero_low;
    int64_t period = (int64_t)resolution * 65536;
    if (zero >= (uint64_t)period) return false;
    *offset = (int64_t)position - (int64_t)zero;
    if (*offset > period / 2) *offset -= period;
    if (*offset < -period / 2) *offset += period;
    return *offset >= -(int64_t)resolution * 200 && *offset <= (int64_t)resolution * 200;
}

static void PublishPosition(void)
{
    int64_t relative = ((int64_t)lift.count - lift.origin) * lift.up_sign;
    taskENTER_CRITICAL();
    g_servo_jog_diagnostics.position_valid = lift.valid;
    g_servo_jog_diagnostics.reference_pending = lift.reference_pending;
    g_servo_jog_diagnostics.ready_for_command = lift.configured && lift.valid &&
        !lift.reference_pending && lift.boot_home_checked && !reset_pending &&
        !pending_command && !lift.route_active && !g_servo_jog_diagnostics.motion_inhibited &&
        !g_servo_jog_diagnostics.fault_code && Stationary();
    g_servo_jog_diagnostics.height_um = lift.valid ? (int32_t)(relative * 1000 / LIFT_UNITS_PER_MM) : 0;
    g_servo_jog_diagnostics.turns_milli = lift.valid ? (int32_t)(relative / 10) : 0;
    g_servo_jog_diagnostics.saved_mask = lift.saved.mask;
    g_servo_jog_diagnostics.home_calibrated =
        lift.saved.home_valid == SERVO_HOMING_CALIBRATION_MARKER;
    g_servo_jog_diagnostics.home_phase = lift.home;
    g_servo_jog_diagnostics.absolute_stage = lift.saved.absolute_stage;
    g_servo_jog_diagnostics.recovery_mode = lift.recovery_mode;
    g_servo_jog_diagnostics.lower_limit_latched = lift.lower_latched;
    g_servo_jog_diagnostics.upper_limit_latched = lift.upper_latched;
    g_servo_jog_diagnostics.operating_min_um = LiftMotion_MinTarget(&lift.saved) / 10;
    g_servo_jog_diagnostics.operating_max_um = LiftMotion_MaxTarget(&lift.saved) / 10;
    g_servo_jog_diagnostics.profile_flags = lift.saved.profile_flags;
    g_servo_jog_diagnostics.lower_edge_um = lift.saved.lower_edge_units / 10;
    g_servo_jog_diagnostics.upper_edge_um = lift.saved.upper_edge_units / 10;
    g_servo_jog_diagnostics.profile_cruise_rpm = lift.saved.cruise_rpm;
    g_servo_jog_diagnostics.route_active = lift.route_active;
    uint32_t speed = lift.saved.cruise_rpm;
    g_servo_jog_diagnostics.manual_speed_rpm = speed;
    g_servo_jog_diagnostics.automatic_speed_rpm = speed;
    for (unsigned i = 0; i < 5; ++i)
        g_servo_jog_diagnostics.saved_points[i] = lift.saved.points[i];
    taskEXIT_CRITICAL();
}

static void CommunicationFault(void)
{
    remote_owned = false;
    lift.route_active = false;
    lift.lower_return_seen = lift.upper_return_seen = false;
    lift.stopping_direction = BSP_ISL60PR_JOG_STOP;
    lift.recovery_mode = false;
    (void)BSP_ISL60PR_EmergencyStop();
    ServoJog_SetBrakeReleased(false);
    lift.configured = false;
    lift.monitor_initialized = false;
    lift.valid = false;
    g_servo_jog_diagnostics.position_valid = 0;
    g_servo_jog_diagnostics.ready_for_command = 0;
    g_servo_jog_diagnostics.commanded_speed = 0;
    lift.zero_confirming = false;
    lift.stop_tracking = false;
    g_servo_jog_diagnostics.target_height_valid = 0;
    lift.moving = BSP_ISL60PR_JOG_STOP;
    lift.manual_control = false;
    lift.home = 0;
    lift.save_edge = false;
    lift.finish_home = false;
    g_servo_jog_diagnostics.active_command = BSP_ISL60PR_JOG_STOP;
    g_servo_jog_diagnostics.command_result = RESULT_COMM;
    g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_DRIVE_ERROR;
    g_servo_jog_diagnostics.communication_errors++;
    g_servo_jog_diagnostics.state = SERVO_JOG_STATE_COMMUNICATION_ERROR;
    /* Preserve the boot decision: wait for a slow-starting drive, but never
       restart homing once its first boot attempt has actually begun. */
    taskENTER_CRITICAL();
    if (stop_pending) lift.boot_home_checked = true;
    stop_pending = false;
    CancelQueuedRemote(RESULT_COMM);
    pending_command = 0;
    if (processing_source == 2U)
    {
        remote_reply_token = processing_token;
        remote_reply_result = RESULT_COMM;
        remote_reply_ready = true;
        processing_source = 0;
    }
    taskEXIT_CRITICAL();
}

static bool StopMotion(bool emergency)
{
    remote_owned = false;
    lift.route_active = false;
    if (lift.moving == BSP_ISL60PR_JOG_STOP && lift.stop_tracking &&
        (!emergency || g_servo_jog_diagnostics.stop_emergency)) return true;
    if (emergency)
    {
        lift.lower_return_seen = lift.upper_return_seen = false;
        lift.stopping_direction = BSP_ISL60PR_JOG_STOP;
    }
    else if (lift.moving != BSP_ISL60PR_JOG_STOP)
        lift.stopping_direction = lift.moving;
    lift.stop_count = lift.count; /* Latest sample, not a hardware edge capture. */
    /* Do not use the gentle target-arrival ramp for an interrupted move. */
    bool quick_stop = emergency || lift.moving != BSP_ISL60PR_JOG_STOP;
    BspIsl60prResult r = quick_stop ? BSP_ISL60PR_EmergencyStop() : BSP_ISL60PR_Stop();
    ServoJog_RecordResult(r);
    if (r != BSP_ISL60PR_OK) { CommunicationFault(); return false; }
    lift.moving = BSP_ISL60PR_JOG_STOP;
    lift.manual_control = false;
    lift.stopped_at = xTaskGetTickCount();
    lift.zero_confirming = false;
    lift.stop_tracking = true;
    g_servo_jog_diagnostics.stop_travel_um = 0;
    g_servo_jog_diagnostics.stop_elapsed_ms = 0;
    g_servo_jog_diagnostics.stop_emergency = quick_stop;
    g_servo_jog_diagnostics.active_command = BSP_ISL60PR_JOG_STOP;
    g_servo_jog_diagnostics.commanded_speed = 0;
    g_servo_jog_diagnostics.state = SERVO_JOG_STATE_STOPPED;
    return true;
}

static bool Stationary(void)
{
    return lift.moving == BSP_ISL60PR_JOG_STOP &&
        !g_servo_jog_diagnostics.brake_released &&
        StoppedSpeedFeedback() && BrakeCanEngage() &&
        xTaskGetTickCount() - lift.stopped_at >= pdMS_TO_TICKS(1200U);
}

static bool SaveCalibration(void)
{
    ServoJog_SetBrakeReleased(false);
    vTaskDelay(pdMS_TO_TICKS(100U));
    bool ok = LiftPositions_Save(&lift.saved);
    if (!ok)
    {
        LiftPositions_Load(&lift.saved);
        lift.valid = false;
        g_servo_jog_diagnostics.command_result = RESULT_FLASH;
    }
    return ok;
}

static void UpdateDirectionalLimits(void)
{
    if (!lift.configured || !lift.valid || g_servo_jog_diagnostics.fault_code)
    {
        lift.lower_latched = false;
        lift.upper_latched = false;
        return;
    }
    int64_t relative = ((int64_t)lift.count - lift.origin) * lift.up_sign;
    lift.lower_latched = relative <= LiftMotion_MinTarget(&lift.saved) + POSITION_TOLERANCE;
    lift.upper_latched = relative >= LiftMotion_MaxTarget(&lift.saved) - POSITION_TOLERANCE;
}

bool ServoJog_RequestReset(void)
{
    taskENTER_CRITICAL();
    bool allowed = !reset_pending && !stop_pending && !pending_command && !remote_owned &&
        processing_source != 2U && !lift.route_active && lift.moving == BSP_ISL60PR_JOG_STOP &&
        !g_servo_jog_diagnostics.brake_released && !g_servo_jog_diagnostics.commanded_speed &&
        (!lift.configured || Stationary());
    if (allowed)
    {
        reset_pending = true;
        reset_requested_at = xTaskGetTickCount();
        g_servo_jog_diagnostics.reset_pending = 1;
        g_servo_jog_diagnostics.ready_for_command = 0;
    }
    taskEXIT_CRITICAL();
    return allowed;
}

static bool LimitBlocks(uint16_t direction)
{
    if (!lift.valid) return true;
    UpdateDirectionalLimits();
    bool up = direction == g_servo_jog_diagnostics.clockwise_command;
    return up ? lift.upper_latched : lift.lower_latched;
}

static bool StartMotion(uint16_t direction, bool pp, int32_t target, uint32_t speed)
{
    if (processing_source == 2U && remote_cancel_requested)
    { g_servo_jog_diagnostics.command_result = RESULT_CANCELED; return false; }
    if (g_servo_jog_diagnostics.motion_inhibited)
    { g_servo_jog_diagnostics.command_result = RESULT_PROFILE; return false; }
    if (!lift.valid || !pp)
    { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return false; }
    if (LimitBlocks(direction))
    {
        g_servo_jog_diagnostics.command_result = RESULT_LIMIT;
        return false;
    }
    BspIsl60prResult r;
    ServoJog_SetBrakeReleased(false);
    vTaskDelay(pdMS_TO_TICKS(SERVO_BRAKE_ZERO_CONFIRM_MS));
    /* Switch modes only while stationary; keep servo torque during the switch. */
    r = BSP_ISL60PR_SetJogSpeed(0U);
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_WriteRegister(0x1003U, pp ? 1U : 3U);
    if (r == BSP_ISL60PR_OK && pp) r = BSP_ISL60PR_WriteRegister32(0x1019U, speed);
    if (r == BSP_ISL60PR_OK && pp) r = BSP_ISL60PR_WriteRegister32(0x1017U, SERVO_MANUAL_SPEED_PULSES_PER_SECOND);
    /* Clear stale PP targets while the brake is still engaged. PV stays at zero. */
    if (r == BSP_ISL60PR_OK && pp) r = BSP_ISL60PR_WriteRegister32(0x100EU, (uint32_t)lift.count);
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_WriteRegister32(0x101BU, lift.home && !pp && lift.home_fine ? 20000U : PP_ACCEL);
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_WriteRegister32(0x101DU, SERVO_PP_DECELERATION_PULSES_PER_SECOND2);
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_WriteRegister32(0x101FU, SERVO_JOG_DECELERATION_PULSES_PER_SECOND2);
    if (r != BSP_ISL60PR_OK) { ServoJog_RecordResult(r); CommunicationFault(); return false; }
    int32_t hold_count = lift.count;
    vTaskDelay(pdMS_TO_TICKS(SERVO_BRAKE_ZERO_CONFIRM_MS));
    if (!ReadMonitors()) { CommunicationFault(); return false; }
    int64_t drift = (int64_t)lift.count - hold_count;
    if (!StoppedSpeedFeedback() ||
        drift > SERVO_BRAKE_STABLE_POSITION_UNITS || drift < -SERVO_BRAKE_STABLE_POSITION_UNITS)
    {
        ServoJog_SetBrakeReleased(false);
        g_servo_jog_diagnostics.command_result = RESULT_BUSY;
        return false;
    }
    /* Recheck limits, stop and keys after blocking bus configuration. */
    if (stop_pending || (processing_source == 2U && remote_cancel_requested) || LimitBlocks(direction) ||
        ((lift.home || (pp && !lift.manual_control)) && (BSP_Key_IsPressed(BSP_KEY_TOP_LEFT) || BSP_Key_IsPressed(BSP_KEY_TOP_RIGHT))) ||
        (lift.manual_control &&
         (!BSP_Key_IsPressed(direction == g_servo_jog_diagnostics.clockwise_command ? BSP_KEY_TOP_RIGHT : BSP_KEY_TOP_LEFT) ||
          BSP_Key_IsPressed(direction == g_servo_jog_diagnostics.clockwise_command ? BSP_KEY_TOP_LEFT : BSP_KEY_TOP_RIGHT))))
    {
        g_servo_jog_diagnostics.command_result = (stop_pending ||
            (processing_source == 2U && remote_cancel_requested)) ? RESULT_CANCELED :
            (LimitBlocks(direction) ? RESULT_LIMIT : RESULT_BUSY);
        return false;
    }
    ServoJog_SetBrakeReleased(true);
    /* Keep real drive feedback fresh while the relay settles. A blind delay
       here previously left released-brake telemetry stale at motion startup. */
    TickType_t release_started = xTaskGetTickCount();
    do
    {
        if (!ReadMonitors()) { CommunicationFault(); return false; }
        drift = (int64_t)lift.count - hold_count;
        if (!StoppedSpeedFeedback() || drift > SERVO_BRAKE_STABLE_POSITION_UNITS ||
            drift < -SERVO_BRAKE_STABLE_POSITION_UNITS)
        {
            g_servo_jog_diagnostics.guard_reason = 8U;
            g_servo_jog_diagnostics.guard_speed_rpm = g_servo_jog_diagnostics.speed_rpm;
            g_servo_jog_diagnostics.guard_error_units = (int32_t)drift;
            g_servo_jog_diagnostics.motion_inhibited = 1U;
            (void)StopMotion(true);
            g_servo_jog_diagnostics.command_result = RESULT_PROFILE;
            return false;
        }
        if (stop_pending || (processing_source == 2U && remote_cancel_requested) ||
            xTaskGetTickCount() - release_started >= pdMS_TO_TICKS(SERVO_BRAKE_RELEASE_SETTLE_TIME_MS))
            break;
        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    } while (true);
    if (stop_pending || (processing_source == 2U && remote_cancel_requested) || LimitBlocks(direction) ||
        ((lift.home || (pp && !lift.manual_control)) &&
         (BSP_Key_IsPressed(BSP_KEY_TOP_LEFT) || BSP_Key_IsPressed(BSP_KEY_TOP_RIGHT))) ||
        (lift.manual_control &&
         (!BSP_Key_IsPressed(direction == g_servo_jog_diagnostics.clockwise_command ? BSP_KEY_TOP_RIGHT : BSP_KEY_TOP_LEFT) ||
          BSP_Key_IsPressed(direction == g_servo_jog_diagnostics.clockwise_command ? BSP_KEY_TOP_LEFT : BSP_KEY_TOP_RIGHT))))
    {
        (void)StopMotion(false);
        ServoJog_SetBrakeReleased(false);
        g_servo_jog_diagnostics.command_result = RESULT_CANCELED;
        return false;
    }
    r = pp ? BSP_ISL60PR_MoveAbsolute(target) : BSP_ISL60PR_SetJogSpeed(speed);
    if (r == BSP_ISL60PR_OK && !pp) r = BSP_ISL60PR_Jog((BspIsl60prJogCommand)direction);
    ServoJog_RecordResult(r);
    if (r != BSP_ISL60PR_OK) { CommunicationFault(); return false; }
    if (direction == g_servo_jog_diagnostics.clockwise_command) lift.upper_return_seen = false;
    else lift.lower_return_seen = false;
    lift.mode = pp ? 1U : 3U;
    lift.target = target;
    lift.moving = direction;
    lift.zero_confirming = false;
    lift.stop_tracking = false;
    lift.stopping_direction = BSP_ISL60PR_JOG_STOP;
    lift.started_at = xTaskGetTickCount();
    lift.arrival_confirming = false;
    g_servo_jog_diagnostics.active_command = direction;
    g_servo_jog_diagnostics.commanded_speed = speed;
    lift.profile_speed = speed;
    g_servo_jog_diagnostics.motion_start_count++;
    g_servo_jog_diagnostics.state = direction == g_servo_jog_diagnostics.clockwise_command ? SERVO_JOG_STATE_CLOCKWISE : SERVO_JOG_STATE_COUNTERCLOCKWISE;
    return true;
}

static bool StartRouteSegment(void)
{
    int64_t relative = ((int64_t)lift.count - lift.origin) * lift.up_sign;
    LiftMotionSegment segment;
    if (!lift.valid || relative < INT32_MIN || relative > INT32_MAX)
    { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return false; }
    if (lift.route_continuing)
    {
        int64_t drift = (int64_t)lift.count - lift.target;
        if (drift < -POSITION_TOLERANCE || drift > POSITION_TOLERANCE)
        { g_servo_jog_diagnostics.command_result = RESULT_LIMIT; return false; }
        relative = ((int64_t)lift.target - lift.origin) * lift.up_sign;
    }
    if (!LiftMotion_Plan((int32_t)relative, lift.route_final, &lift.saved, &segment))
    { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return false; }
    int64_t target = (int64_t)lift.origin + (int64_t)segment.target_units * lift.up_sign;
    if (target < -1073741824LL || target > 1073741824LL)
    { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return false; }
    lift.manual_control = lift.route_manual;
    if (!segment.speed_units_s)
    { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return false; }
    if (lift.route_continuing)
    {
        /* Same-direction PP continuation from verified standstill. Keep the
           servo holding the load; do not cycle the brake between segments. */
        if (lift.mode != 1U || !g_servo_jog_diagnostics.brake_released || !ReadMonitors())
        { CommunicationFault(); return false; }
        int64_t drift = (int64_t)lift.count - lift.target;
        if (!StoppedSpeedFeedback() || drift < -POSITION_TOLERANCE || drift > POSITION_TOLERANCE)
        {
            g_servo_jog_diagnostics.guard_reason = 8U;
            g_servo_jog_diagnostics.guard_speed_rpm = g_servo_jog_diagnostics.speed_rpm;
            g_servo_jog_diagnostics.guard_error_units = (int32_t)drift;
            g_servo_jog_diagnostics.motion_inhibited = 1;
            (void)StopMotion(true);
            g_servo_jog_diagnostics.command_result = RESULT_PROFILE;
            return false;
        }
        bool key_blocked = lift.route_manual ?
            (!BSP_Key_IsPressed(lift.route_direction == g_servo_jog_diagnostics.clockwise_command ?
                BSP_KEY_TOP_RIGHT : BSP_KEY_TOP_LEFT) ||
             BSP_Key_IsPressed(lift.route_direction == g_servo_jog_diagnostics.clockwise_command ?
                BSP_KEY_TOP_LEFT : BSP_KEY_TOP_RIGHT)) :
            (BSP_Key_IsPressed(BSP_KEY_TOP_LEFT) || BSP_Key_IsPressed(BSP_KEY_TOP_RIGHT));
        if (stop_pending || key_blocked || LimitBlocks(lift.route_direction))
        { (void)StopMotion(false); g_servo_jog_diagnostics.command_result = RESULT_CANCELED; return false; }
        BspIsl60prResult r = BSP_ISL60PR_WriteRegister32(0x1019U, segment.speed_units_s);
        key_blocked = lift.route_manual ?
            (!BSP_Key_IsPressed(lift.route_direction == g_servo_jog_diagnostics.clockwise_command ?
                BSP_KEY_TOP_RIGHT : BSP_KEY_TOP_LEFT) ||
             BSP_Key_IsPressed(lift.route_direction == g_servo_jog_diagnostics.clockwise_command ?
                BSP_KEY_TOP_LEFT : BSP_KEY_TOP_RIGHT)) :
            (BSP_Key_IsPressed(BSP_KEY_TOP_LEFT) || BSP_Key_IsPressed(BSP_KEY_TOP_RIGHT));
        if (r == BSP_ISL60PR_OK && !stop_pending && !key_blocked && !LimitBlocks(lift.route_direction))
            r = BSP_ISL60PR_MoveAbsolute((int32_t)target);
        else if (r == BSP_ISL60PR_OK)
        { (void)StopMotion(false); g_servo_jog_diagnostics.command_result = RESULT_CANCELED; return false; }
        ServoJog_RecordResult(r);
        if (r != BSP_ISL60PR_OK) { CommunicationFault(); return false; }
        lift.target = (int32_t)target;
        lift.profile_speed = segment.speed_units_s;
        lift.arrival_confirming = false;
        lift.started_at = xTaskGetTickCount();
        lift.route_continuing = false;
        g_servo_jog_diagnostics.commanded_speed = segment.speed_units_s;
        g_servo_jog_diagnostics.motion_start_count++;
        return true;
    }
    bool started = StartMotion(lift.route_direction, true, (int32_t)target, segment.speed_units_s);
    if (started) lift.route_continuing = false;
    return started;
}

static bool MoveTo(int32_t relative)
{
    if (g_servo_jog_diagnostics.motion_inhibited)
    { g_servo_jog_diagnostics.command_result = RESULT_PROFILE; return false; }
    lift.absolute_return = false;
    int64_t target = (int64_t)lift.origin + (int64_t)relative * lift.up_sign;
    if (!lift.valid) { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return false; }
    if (relative < LiftMotion_MinTarget(&lift.saved) ||
        relative > LiftMotion_MaxTarget(&lift.saved) ||
        target < -1073741824LL || target > 1073741824LL)
    { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return false; }
    int64_t delta = target - lift.count;
    if (delta >= -POSITION_TOLERANCE && delta <= POSITION_TOLERANCE)
    {
        g_servo_jog_diagnostics.target_height_um = relative / 10;
        g_servo_jog_diagnostics.target_height_valid = 1;
        g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_REACHED;
        return true;
    }
    uint16_t direction = delta * lift.up_sign > 0 ? g_servo_jog_diagnostics.clockwise_command : g_servo_jog_diagnostics.counterclockwise_command;
    lift.route_final = relative;
    lift.route_manual = lift.manual_control;
    lift.route_direction = direction;
    lift.route_continuing = false;
    taskENTER_CRITICAL();
    lift.route_active = true;
    g_servo_jog_diagnostics.active_source = lift.route_manual ? 3U :
        (processing_source ? processing_source : 1U);
    taskEXIT_CRITICAL();
    if (!StartRouteSegment()) { lift.route_active = false; lift.manual_control = false; return false; }
    g_servo_jog_diagnostics.target_height_um = relative / 10;
    g_servo_jog_diagnostics.target_height_valid = 1;
    g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_MOVING;
    return true;
}

static bool RestoreAbsoluteReference(void)
{
    int64_t offset;
    int32_t before = lift.count;
    lift.valid = false;
    if (lift.saved.absolute_stage != 2U || lift.saved.up_sign != lift.up_sign ||
        !AbsoluteOffset(&offset)) return false;
    int64_t relative = offset * lift.saved.absolute_up_sign * SERVO_COMMAND_UNITS_PER_REVOLUTION /
                       lift.saved.absolute_resolution;
    if (relative < LIFT_MECHANICAL_MIN_MM * LIFT_UNITS_PER_MM ||
        relative > LIFT_MECHANICAL_MAX_MM * LIFT_UNITS_PER_MM ||
        !ReadMonitors() || !Stationary()) return false;
    int64_t drift = (int64_t)lift.count - before;
    if (drift > SERVO_BRAKE_STABLE_POSITION_UNITS || drift < -SERVO_BRAKE_STABLE_POSITION_UNITS) return false;
    int64_t origin = (int64_t)lift.count - relative * lift.up_sign;
    if (origin < -1073741824LL || origin > 1073741824LL) return false;
    lift.origin = (int32_t)origin;
    lift.valid = true;
    return true;
}

static bool Configure(void)
{
    ServoJog_SetBrakeReleased(false);
    BspIsl60prResult r = BSP_ISL60PR_ConfigureJogRs485(0U,
        SERVO_JOG_ACCELERATION_PULSES_PER_SECOND2, SERVO_JOG_DECELERATION_PULSES_PER_SECOND2);
    uint16_t direction = 0, motor = 0;
    uint32_t units = 0, numerator = 0, denominator = 0, resolution = 0;
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_ReadRegister(0x0202U, &direction);
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_ReadRegister(0x0000U, &motor);
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_ReadRegister32(0x0502U, &units);
    if (r == BSP_ISL60PR_OK && units == 0U)
    {
        r = BSP_ISL60PR_ReadRegister32(0x0507U, &numerator);
        if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_ReadRegister32(0x0509U, &denominator);
        if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_ReadRegister32(0x001FU, &resolution);
        if (r == BSP_ISL60PR_OK && numerator && denominator && resolution)
        {
            uint64_t scaled = (uint64_t)resolution * denominator;
            if (scaled % numerator == 0 && scaled / numerator <= UINT32_MAX)
                units = (uint32_t)(scaled / numerator);
        }
    }
    g_servo_jog_diagnostics.motor_code = motor;
    g_servo_jog_diagnostics.command_units_per_rev = units;
    if (r != BSP_ISL60PR_OK || direction > 1U)
    { ServoJog_RecordResult(r); return false; }
    /* Match the previously bench-verified K1 down / K2 up mapping. */
    uint16_t up = direction == 0 ? BSP_ISL60PR_JOG_NEGATIVE : BSP_ISL60PR_JOG_POSITIVE;
#if SERVO_REVERSE_DIRECTION_MAPPING
    up = up == BSP_ISL60PR_JOG_POSITIVE ? BSP_ISL60PR_JOG_NEGATIVE : BSP_ISL60PR_JOG_POSITIVE;
#endif
    lift.up_sign = up == BSP_ISL60PR_JOG_POSITIVE ? 1 : -1;
    g_servo_jog_diagnostics.direction_selection = direction;
    g_servo_jog_diagnostics.clockwise_command = up;
    g_servo_jog_diagnostics.counterclockwise_command = up == BSP_ISL60PR_JOG_POSITIVE ? BSP_ISL60PR_JOG_NEGATIVE : BSP_ISL60PR_JOG_POSITIVE;
    r = BSP_ISL60PR_WriteRegister32(0x1007U, POSITION_TOLERANCE);
    /* ConfigureJogRs485 left the servo disabled with the external brake applied.
       Battery-backed multi-turn feedback requires the absolute detection system.
       Never reset the encoder or its multi-turn data here. */
    uint32_t encoder_kind = (motor / 10U) % 10U;
    if (r == BSP_ISL60PR_OK && (encoder_kind == 2U || encoder_kind == 3U))
    {
        uint16_t absolute_system;
        r = BSP_ISL60PR_ReadRegister(0x0201U, &absolute_system);
        if (r == BSP_ISL60PR_OK && absolute_system == 0U)
            r = BSP_ISL60PR_WriteRegister(0x0201U, 1U);
        if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_ReadRegister(0x0201U, &absolute_system);
        if (r == BSP_ISL60PR_OK) g_servo_jog_diagnostics.absolute_system = absolute_system;
        if (r == BSP_ISL60PR_OK && absolute_system != 1U) return false;
    }
    if (r == BSP_ISL60PR_OK) r = BSP_ISL60PR_SetServoEnabled(true);
    ServoJog_RecordResult(r);
    if (r != BSP_ISL60PR_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(SERVO_ENABLE_SETTLE_TIME_MS));
    lift.moving = BSP_ISL60PR_JOG_STOP;
    lift.zero_confirming = false;
    for (unsigned elapsed = 0; elapsed <= SERVO_BRAKE_ZERO_CONFIRM_MS; elapsed += MONITOR_PERIOD_MS)
    {
        if (!ReadMonitors() || !StoppedSpeedFeedback()) return false;
        if (BrakeCanEngage()) break;
        if (elapsed == SERVO_BRAKE_ZERO_CONFIRM_MS) return false;
        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
    lift.stopped_at = xTaskGetTickCount();
    lift.mode = 3;
    g_servo_jog_diagnostics.state = SERVO_JOG_STATE_READY;
    return true;
}

static void BeginHomeMode(bool learn, bool recover_from_below)
{
    /* No blind optical scan or PV bypass in the repair/validation firmware. */
    if (learn || recover_from_below)
    { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
    /* A saved zero is not evidence that the drive retained its coordinate frame. */
    if (!learn && !recover_from_below && !lift.valid && lift.saved.absolute_stage == 2U)
    {
        ServoJog_SetBrakeReleased(false);
        if (!RestoreAbsoluteReference())
        { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
    }
    if (!learn && lift.valid && !recover_from_below)
    {
        if (lift.saved.up_sign != lift.up_sign || !(lift.saved.mask & 4U))
        { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
        g_servo_jog_diagnostics.target_position = 3;
        if (MoveTo(0) && lift.moving != BSP_ISL60PR_JOG_STOP) lift.absolute_return = true;
        return;
    }
    g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE;
}

static void BeginHome(bool learn)
{
    BeginHomeMode(learn, false);
}

static void ProcessCommandInternal(LiftCommand command, int32_t value)
{
    g_servo_jog_diagnostics.command_result = RESULT_OK;
    if (!Stationary() || lift.home || lift.route_active)
    { g_servo_jog_diagnostics.command_result = RESULT_BUSY; return; }
    if (command == LIFT_CMD_SLOW)
    { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return; }
    if (command == LIFT_CMD_FAST)
    {
        g_servo_jog_diagnostics.manual_speed_rpm = lift.saved.cruise_rpm;
        return;
    }
    if (g_servo_jog_diagnostics.command_units_per_rev != SERVO_COMMAND_UNITS_PER_REVOLUTION)
    { g_servo_jog_diagnostics.command_result = RESULT_SCALE; return; }
    if (command == LIFT_CMD_PROBE_UM)
    {
        g_servo_jog_diagnostics.command_result = RESULT_RANGE;
        return;
    }
    if (command == LIFT_CMD_LIMIT_LOWER || command == LIFT_CMD_LIMIT_UPPER ||
        command == LIFT_CMD_VERIFY_LIMITS || command == LIFT_CMD_PROFILE_SLOW)
    {
        g_servo_jog_diagnostics.command_result = RESULT_RANGE;
        return;
    }
    if (command == LIFT_CMD_SPEED || command == LIFT_CMD_BOOT_HOME)
    {
        if (!lift.valid || !(lift.saved.mask & 4U))
        { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
        if (command == LIFT_CMD_SPEED)
        {
            if (value < (int32_t)LIFT_PROFILE_SLOW_RPM || value > 3000)
            { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return; }
            lift.saved.cruise_rpm = (uint32_t)value;
        }
        else if (command == LIFT_CMD_BOOT_HOME)
        {
            if ((value != 0 && value != 1) || (value && lift.saved.absolute_stage != 2U))
            { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
            if (value) lift.saved.profile_flags |= LIFT_PROFILE_BOOT_HOME_ARMED;
            else lift.saved.profile_flags &= ~LIFT_PROFILE_BOOT_HOME_ARMED;
        }
        (void)SaveCalibration();
        return;
    }
    if (command == LIFT_CMD_RECOVER)
    {
        /* Recovery never bypasses position/brake/limit interlocks. */
        lift.recovery_mode = false;
        if (g_servo_jog_diagnostics.motion_inhibited)
        { g_servo_jog_diagnostics.command_result = RESULT_PROFILE; return; }
        if (!lift.valid && !RestoreAbsoluteReference())
            g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE;
        return;
    }
    if (command == LIFT_CMD_HOME_UP)
    { BeginHomeMode(false, true); return; }
    if (command == LIFT_CMD_ABS_READ)
    {
        uint64_t position;
        uint32_t resolution;
        if (!ReadAbsolute(&position, &resolution))
            g_servo_jog_diagnostics.command_result = RESULT_COMM;
        uint16_t turns;
        uint32_t single, low, high;
        if (BSP_ISL60PR_ReadRegister(0x0B46U, &turns) == BSP_ISL60PR_OK &&
            BSP_ISL60PR_ReadRegister32(0x0B47U, &single) == BSP_ISL60PR_OK &&
            BSP_ISL60PR_ReadRegister32(0x0B3AU, &low) == BSP_ISL60PR_OK &&
            BSP_ISL60PR_ReadRegister32(0x0B3CU, &high) == BSP_ISL60PR_OK)
        {
            g_servo_jog_diagnostics.absolute_turns = turns;
            g_servo_jog_diagnostics.absolute_single = single;
            g_servo_jog_diagnostics.mechanical_low = low;
            g_servo_jog_diagnostics.mechanical_high = high;
        }
        else g_servo_jog_diagnostics.command_result = RESULT_COMM;
        return;
    }
    if (command == LIFT_CMD_ABS_CHECK)
    {
        int64_t offset;
        if (!lift.valid || lift.origin != lift.saved.taught_origin || !AbsoluteOffset(&offset))
        { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
        if (!ReadMonitors() || !Stationary())
        { g_servo_jog_diagnostics.command_result = RESULT_BUSY; return; }
        int64_t relative = ((int64_t)lift.count - lift.origin) * lift.up_sign;
        if ((relative > -LIFT_UNITS_PER_MM && relative < LIFT_UNITS_PER_MM) ||
            relative < LIFT_WORK_MIN_UNITS || relative > LIFT_WORK_MAX_UNITS)
        { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return; }
        int32_t sign = (offset > 0) == (relative > 0) ? 1 : -1;
        int64_t expected = relative * lift.saved.absolute_resolution / SERVO_COMMAND_UNITS_PER_REVOLUTION;
        int64_t error = offset * sign - expected;
        if (error > (int64_t)lift.saved.absolute_resolution / 100 ||
            error < -(int64_t)lift.saved.absolute_resolution / 100)
        { g_servo_jog_diagnostics.command_result = RESULT_SCALE; return; }
        lift.saved.absolute_up_sign = sign;
        lift.saved.absolute_stage = 2;
        lift.saved.profile_flags |= LIFT_PROFILE_BOOT_HOME_ARMED;
        (void)SaveCalibration();
        return;
    }
    if (command == LIFT_CMD_HOME || command == LIFT_CMD_LEARN)
    { BeginHome(command == LIFT_CMD_LEARN); return; }
    /* A wide optical flag cannot rebase a persisted absolute coordinate. */
    if (command == LIFT_CMD_REFERENCE3)
    { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
    if (command == LIFT_CMD_SET && value == 3)
    {
        if (g_servo_jog_diagnostics.position_sensor_mask != SERVO_CENTER_SENSOR_MASK ||
            BSP_PositionSensor_ReadMask() != SERVO_CENTER_SENSOR_MASK)
        { g_servo_jog_diagnostics.command_result = RESULT_SENSOR; return; }
        if (command == LIFT_CMD_SET)
        {
            int32_t before = lift.count;
            uint64_t absolute = 0;
            uint32_t resolution = 0;
            bool absolute_valid = ReadAbsolute(&absolute, &resolution);
            if (!absolute_valid)
            { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
            if (!ReadMonitors() || !Stationary() ||
                (int64_t)lift.count - before > SERVO_BRAKE_STABLE_POSITION_UNITS ||
                (int64_t)lift.count - before < -SERVO_BRAKE_STABLE_POSITION_UNITS)
            { g_servo_jog_diagnostics.command_result = RESULT_BUSY; return; }
            bool paired = lift.saved.absolute_stage == 2U &&
                lift.saved.absolute_motor == g_servo_jog_diagnostics.motor_code &&
                lift.saved.absolute_resolution == resolution && lift.saved.up_sign == lift.up_sign;
            int32_t absolute_sign = lift.saved.absolute_up_sign;
            for (unsigned i = 0; i < 5; ++i) lift.saved.points[i] = 0;
            lift.saved.lower_edge_units = LIFT_WORK_MIN_UNITS;
            lift.saved.upper_edge_units = LIFT_WORK_MAX_UNITS;
            lift.saved.profile_flags = paired ? LIFT_PROFILE_BOOT_HOME_ARMED : 0U;
            g_servo_jog_diagnostics.edge_capture_mask = 0;
            lift.saved.mask = 4U;
            lift.saved.home_valid = 0;
            lift.saved.home_edge = 0;
            lift.saved.up_sign = lift.up_sign;
            lift.saved.taught_origin = lift.count;
            lift.saved.absolute_stage = 0;
            lift.saved.absolute_up_sign = 0;
            lift.saved.absolute_zero_low = lift.saved.absolute_zero_high = 0;
            lift.saved.absolute_resolution = lift.saved.absolute_motor = 0;
            if (absolute_valid)
            {
                lift.saved.absolute_zero_low = (uint32_t)absolute;
                lift.saved.absolute_zero_high = (uint32_t)(absolute >> 32);
                lift.saved.absolute_resolution = resolution;
                lift.saved.absolute_motor = g_servo_jog_diagnostics.motor_code;
                lift.saved.absolute_stage = 1;
                if (paired)
                {
                    lift.saved.absolute_stage = 2;
                    lift.saved.absolute_up_sign = absolute_sign;
                }
            }
        }
        else if (!(lift.saved.mask & 4U) || lift.saved.up_sign != lift.up_sign)
        { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
        lift.origin = lift.count;
        lift.valid = true;
        lift.recovery_mode = false;
        g_servo_jog_diagnostics.target_height_valid = 0;
        if (command == LIFT_CMD_SET) (void)SaveCalibration();
        return;
    }
    if (!lift.valid) { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
    if (command == LIFT_CMD_SET)
    {
        g_servo_jog_diagnostics.command_result = RESULT_RANGE;
    }
    else if (command == LIFT_CMD_GOTO)
    {
        if (value != 3 || !(lift.saved.mask & 4U))
        { g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE; return; }
        g_servo_jog_diagnostics.target_position = value;
        if (!MoveTo(lift.saved.points[value - 1]))
            g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_LIMIT_BLOCKED;
    }
    else if (command == LIFT_CMD_HEIGHT)
    {
        if (value < LIFT_WORK_MIN_MM || value > LIFT_WORK_MAX_MM)
        { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return; }
        g_servo_jog_diagnostics.target_position = 0;
        (void)MoveTo(value * LIFT_UNITS_PER_MM);
    }
    else if (command == LIFT_CMD_HEIGHT_UM)
    {
        if (value < LIFT_WORK_MIN_MM * 1000L || value > LIFT_WORK_MAX_MM * 1000L)
        { g_servo_jog_diagnostics.command_result = RESULT_RANGE; return; }
        g_servo_jog_diagnostics.target_position = 0;
        (void)MoveTo(value * 10L);
    }
}

static void ReportHeightCommand(LiftCommand command, int32_t value)
{
    if (command != LIFT_CMD_HEIGHT) return;
    taskENTER_CRITICAL();
    g_servo_jog_diagnostics.height_reply_mm = value;
    g_servo_jog_diagnostics.height_reply_result = g_servo_jog_diagnostics.command_result;
    g_servo_jog_diagnostics.height_reply_sequence++;
    taskEXIT_CRITICAL();
}

static void ProcessCommand(LiftCommand command, int32_t value)
{
    if (processing_source == 2U && remote_cancel_requested)
        g_servo_jog_diagnostics.command_result = RESULT_CANCELED;
    else ProcessCommandInternal(command, value);
    ReportHeightCommand(command, value);
}

static void ReportRemoteCommand(void)
{
    if (processing_source == 2U)
    {
        PublishPosition();
        taskENTER_CRITICAL();
        remote_reply_token = processing_token;
        remote_reply_result = g_servo_jog_diagnostics.command_result;
        remote_reply_ready = true;
        if (remote_reply_result != RESULT_OK || !lift.route_active ||
            g_servo_jog_diagnostics.active_source != 2U) remote_owned = false;
        taskEXIT_CRITICAL();
    }
    processing_source = 0;
}

static void ServoJog_Task(void *argument)
{
    (void)argument;
    bool armed = false;
    TickType_t last_monitor = 0;
    BSP_Key_Init();
    BSP_ISL60PR_Init();
    LiftPositions_Load(&lift.saved);
    lift.manual_control = false;
    g_servo_jog_diagnostics.manual_speed_rpm = LIFT_PROFILE_SLOW_RPM;
    lift.moving = BSP_ISL60PR_JOG_STOP;
    ServoJog_SetBrakeReleased(false);
    for (;;)
    {
        if (reset_pending)
        {
            /* Give the UART task time to emit the final ACK before reset.
               STOP cancels this window; no motion is queued during it. */
            if (xTaskGetTickCount() - reset_requested_at >= pdMS_TO_TICKS(250U))
            {
                taskENTER_CRITICAL();
                if (reset_pending) NVIC_SystemReset();
                taskEXIT_CRITICAL();
            }
            vTaskDelay(pdMS_TO_TICKS(10U));
            continue;
        }
        ServoJog_UpdateKeys();
        bool keys = BSP_Key_IsPressed(BSP_KEY_TOP_LEFT) || BSP_Key_IsPressed(BSP_KEY_TOP_RIGHT);
        if (ServoJog_KeysSettled() && !keys) armed = true;
        if (k1_state.stable_pressed && k2_state.stable_pressed) armed = false;
        if (!lift.configured)
        {
            armed = false;
            lift.valid = false;
            lift.configured = Configure();
            if (!lift.configured)
            {
                CommunicationFault();
                armed = false;
                PublishPosition();
                vTaskDelay(pdMS_TO_TICKS(SERVO_RECONNECT_PERIOD_MS));
                continue;
            }
            last_monitor = 0;
            lift.reference_pending = true;
            lift.last_reference_attempt = xTaskGetTickCount() - pdMS_TO_TICKS(500U);
            g_servo_jog_diagnostics.reference_pending = 1;
        }
        bool sampled = false;
        if (xTaskGetTickCount() - last_monitor >= pdMS_TO_TICKS(MONITOR_PERIOD_MS))
        {
            if (!ReadMonitors()) { CommunicationFault(); armed = false; continue; }
            sampled = true;
            last_monitor = xTaskGetTickCount();
        }
        /* Run before any early continue, including conflict and repeated STOP. */
        if (lift.configured && BrakeCanEngage())
        {
            ServoJog_SetBrakeReleased(false);
            lift.stop_tracking = false;
        }
        bool stop;
        LiftCommand command;
        int32_t value;
        uint8_t command_source;
        taskENTER_CRITICAL();
        stop = stop_pending;
        stop_pending = false;
        command = pending_command;
        value = pending_value;
        processing_source = command ? pending_source : 0U;
        command_source = processing_source;
        processing_token = pending_token;
        pending_command = 0;
        taskEXIT_CRITICAL();
        if (stop || (keys && (lift.home || (lift.route_active && !lift.route_manual) ||
            (lift.moving != BSP_ISL60PR_JOG_STOP && lift.mode == 1 && !lift.manual_control))))
        {
            lift.recovery_mode = false;
            if (lift.home) lift.valid = false;
            lift.home = 0;
            lift.save_edge = false;
            lift.finish_home = false;
            (void)StopMotion(false);
            armed = false;
            lift.boot_home_checked = true;
            g_servo_jog_diagnostics.command_result = RESULT_CANCELED;
            ReportHeightCommand(command, value);
            ReportRemoteCommand();
            command = 0;
            g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_CANCELED_BY_SERIAL;
            PublishPosition();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (lift.reference_pending && Stationary() &&
            xTaskGetTickCount() - lift.last_reference_attempt >= pdMS_TO_TICKS(500U))
        {
            lift.last_reference_attempt = xTaskGetTickCount();
            if (keys) lift.boot_home_checked = true;
            /* Legacy P0B-07 calibration never authorizes absolute boot motion. */
            lift.valid = false;
            ServoJog_SetBrakeReleased(false);
            g_servo_jog_diagnostics.command_result = RESULT_NEED_REFERENCE;
            if (!keys && g_servo_jog_diagnostics.command_units_per_rev == SERVO_COMMAND_UNITS_PER_REVOLUTION &&
                RestoreAbsoluteReference())
            {
                lift.reference_pending = false;
                g_servo_jog_diagnostics.command_result = RESULT_OK;
                g_servo_jog_diagnostics.target_position = 3;
                processing_source = 4U;
                bool auto_home = !lift.boot_home_checked;
                lift.boot_home_checked = true;
                g_servo_jog_diagnostics.target_height_valid = 0;
                g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_IDLE;
                if (auto_home && (lift.saved.profile_flags & LIFT_PROFILE_BOOT_HOME_ARMED) && MoveTo(0) &&
                    lift.moving != BSP_ISL60PR_JOG_STOP) lift.absolute_return = true;
                processing_source = command_source;
            }
            else if (lift.saved.absolute_stage != 2U)
            {
                lift.reference_pending = false;
                lift.boot_home_checked = true;
                g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_REJECTED_NO_POSITION;
            }
        }
        UpdateDirectionalLimits();
        uint16_t limit_direction = lift.moving;
        if (limit_direction == BSP_ISL60PR_JOG_STOP && lift.stop_tracking &&
            g_servo_jog_diagnostics.brake_released)
            limit_direction = lift.stopping_direction;
        /* A legal endpoint is also a target. Let the low-speed PP controller
           finish its existing stop inside the arrival tolerance; never exempt
           an outward start, overshoot, stale feedback or a fast approach. */
        bool endpoint_settling = lift.valid && lift.mode == 1U && g_servo_jog_diagnostics.position_speed_valid &&
            xTaskGetTickCount() - g_servo_jog_diagnostics.position_sample_tick <= pdMS_TO_TICKS(500U) &&
            LiftMotion_EndpointSettlingAllowed(
                ((int64_t)lift.count - lift.origin) * lift.up_sign,
                ((int64_t)lift.target - lift.origin) * lift.up_sign,
                limit_direction == g_servo_jog_diagnostics.clockwise_command,
                g_servo_jog_diagnostics.position_speed_rpm, lift.profile_speed);
        if (limit_direction != BSP_ISL60PR_JOG_STOP && LimitBlocks(limit_direction) && !endpoint_settling)
        {
            lift.recovery_mode = false;
            armed = false;
            (void)StopMotion(true);
            g_servo_jog_diagnostics.limit_stop_count++;
            {
                if (lift.home) lift.valid = false;
                lift.home = 0; lift.finish_home = false; lift.save_edge = false;
                g_servo_jog_diagnostics.command_result = RESULT_LIMIT;
                g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_LIMIT_BLOCKED;
            }
        }
        if (!lift.configured) continue;
        if (sampled && ((lift.route_active && lift.moving != BSP_ISL60PR_JOG_STOP) ||
            (lift.stop_tracking && lift.stopping_direction != BSP_ISL60PR_JOG_STOP)))
        {
            int64_t speed = g_servo_jog_diagnostics.speed_rpm;
            if (speed < 0) speed = -speed;
            int64_t position_speed = g_servo_jog_diagnostics.position_speed_rpm;
            if (position_speed < 0) position_speed = -position_speed;
            int sign = lift.route_direction == g_servo_jog_diagnostics.clockwise_command ? 1 : -1;
            int64_t beyond = ((int64_t)lift.count - lift.target) * lift.up_sign * sign;
            uint32_t cap = LiftMotion_GuardSpeedRpm(-beyond, lift.profile_speed);
            uint32_t cruise = lift.profile_speed * 60U / SERVO_COMMAND_UNITS_PER_REVOLUTION;
            uint32_t cruise_margin = cruise / 20U;
            if (cruise_margin < 30U) cruise_margin = 30U;
            bool invalid = !lift.valid || !g_servo_jog_diagnostics.position_speed_valid;
            /* P0B-00 is low-pass filtered (~0.3 s on the tested drive). Check
               the braking curve using position differences, while retaining
               the independent reported-speed ceiling and overshoot guard. */
            bool too_fast = speed > cruise + cruise_margin || position_speed > cap;
            if (invalid || too_fast || beyond > POSITION_TOLERANCE)
            {
                g_servo_jog_diagnostics.guard_reason = (invalid ? 1U : 0U) |
                    (too_fast ? 2U : 0U) | (beyond > POSITION_TOLERANCE ? 4U : 0U);
                g_servo_jog_diagnostics.guard_speed_rpm = position_speed > cap ?
                    g_servo_jog_diagnostics.position_speed_rpm : g_servo_jog_diagnostics.speed_rpm;
                g_servo_jog_diagnostics.guard_cap_rpm = position_speed > cap ? cap : cruise + cruise_margin;
                g_servo_jog_diagnostics.guard_error_units = (int32_t)beyond;
                g_servo_jog_diagnostics.guard_height_um = (int32_t)(((int64_t)lift.count - lift.origin) * lift.up_sign / 10);
                g_servo_jog_diagnostics.motion_inhibited = 1;
                if (sign > 0) lift.upper_latched = true;
                else lift.lower_latched = true;
                (void)StopMotion(true);
                armed = false;
                g_servo_jog_diagnostics.command_result = RESULT_PROFILE;
                g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_LIMIT_BLOCKED;
            }
        }
        if (lift.route_active && lift.route_manual &&
            (!keys || ServoJog_GetManualCommand(armed) != lift.route_direction))
        {
            (void)StopMotion(false);
            armed = false;
            g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_CANCELED_BY_KEY;
        }
        if (command)
        {
            if (keys)
            {
                g_servo_jog_diagnostics.command_result = RESULT_BUSY;
                ReportHeightCommand(command, value);
            }
            else ProcessCommand(command, value);
            ReportRemoteCommand();
        }
        if (!lift.configured) continue;
        if (lift.moving != BSP_ISL60PR_JOG_STOP && lift.mode == 1U && sampled)
        {
            if (PositionArrivalConfirmed())
            {
                bool more = lift.route_active &&
                    (((int64_t)lift.target - lift.origin) * lift.up_sign != lift.route_final);
                bool manual_route = lift.route_active && lift.route_manual;
                if (lift.recovery_mode) armed = false; /* Release the key before another step. */
                if (more)
                {
                    lift.route_continuing = true;
                    if (!StartRouteSegment())
                    {
                        if (lift.moving != BSP_ISL60PR_JOG_STOP) (void)StopMotion(true);
                        armed = false;
                        g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_LIMIT_BLOCKED;
                    }
                }
                else if (!StopMotion(false)) continue;
                if (!more)
                {
                    if (manual_route) armed = false;
                    lift.home = 0;
                    lift.absolute_return = false;
                    g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_REACHED;
                }
            }
            else if (xTaskGetTickCount() - lift.started_at > pdMS_TO_TICKS(HOME_TIMEOUT_MS))
            {
                (void)StopMotion(true);
                lift.home = 0;
                g_servo_jog_diagnostics.command_result = RESULT_TIMEOUT;
                g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_TIMEOUT;
            }
        }
        if (lift.route_active && lift.moving == BSP_ISL60PR_JOG_STOP && Stationary())
        {
            if (!StartRouteSegment())
            {
                lift.route_active = false;
                lift.manual_control = false;
                armed = false;
                g_servo_jog_diagnostics.target_status = SERVO_TARGET_STATUS_LIMIT_BLOCKED;
            }
        }
        if (!lift.home && !lift.route_active &&
            (lift.manual_control || lift.mode != 1U || lift.moving == BSP_ISL60PR_JOG_STOP))
        {
            uint16_t requested = ServoJog_GetManualCommand(armed);
            if (!keys) requested = BSP_ISL60PR_JOG_STOP;
            if (lift.moving != BSP_ISL60PR_JOG_STOP && requested != lift.moving)
                (void)StopMotion(false);
            else if (lift.configured && Stationary() && requested != BSP_ISL60PR_JOG_STOP && !LimitBlocks(requested))
            {
                /* Hold-to-run PP shares the same bounded profile as serial/HOME. */
                lift.manual_control = true;
                lift.absolute_return = false;
                g_servo_jog_diagnostics.target_height_valid = 0;
                g_servo_jog_diagnostics.target_position = 0;
                int32_t endpoint = requested == g_servo_jog_diagnostics.clockwise_command ?
                    LiftMotion_MaxTarget(&lift.saved) : LiftMotion_MinTarget(&lift.saved);
                int64_t current = ((int64_t)lift.count - lift.origin) * lift.up_sign;
                bool outward = requested == g_servo_jog_diagnostics.clockwise_command ?
                    current >= endpoint - POSITION_TOLERANCE : current <= endpoint + POSITION_TOLERANCE;
                if (outward || !MoveTo(endpoint))
                {
                    if (outward) g_servo_jog_diagnostics.command_result = RESULT_LIMIT;
                    lift.manual_control = false; armed = false;
                }
                else
                {
                    g_servo_jog_diagnostics.command_result = RESULT_OK;
                    if (lift.moving == BSP_ISL60PR_JOG_STOP)
                    { lift.manual_control = false; armed = false; }
                }
            }
        }
        if (lift.configured && BrakeCanEngage())
        {
            ServoJog_SetBrakeReleased(false);
            lift.stop_tracking = false;
        }
        PublishPosition();
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

void ServoJog_TaskCreate(void)
{
    (void)xTaskCreate(ServoJog_Task, SERVO_JOG_TASK_NAME, SERVO_JOG_TASK_STACK_WORDS,
                     NULL, SERVO_JOG_TASK_PRIORITY, NULL);
}
