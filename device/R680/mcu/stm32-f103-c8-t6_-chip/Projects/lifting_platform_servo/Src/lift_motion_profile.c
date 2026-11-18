#include "lift_motion_profile.h"

#include <limits.h>
#include <stddef.h>

#include "servo_app_config.h"

#if LIFT_UNITS_PER_MM <= 0 || SERVO_COMMAND_UNITS_PER_REVOLUTION == 0
#error "Motion profile requires positive position and revolution scales."
#endif
#if LIFT_MECHANICAL_MIN_MM * LIFT_UNITS_PER_MM < INT32_MIN || \
    LIFT_MECHANICAL_MAX_MM * LIFT_UNITS_PER_MM > INT32_MAX
#error "Motion profile endpoint units must fit int32_t."
#endif
#if SERVO_PP_DECELERATION_PULSES_PER_SECOND2 == 0
#error "Native PP profile requires a positive deceleration."
#endif
#if LIFT_PROFILE_SLOW_RPM == 0 || \
    LIFT_PROFILE_SLOW_RPM > LIFT_PROFILE_CRUISE_RPM || \
    LIFT_PROFILE_CRUISE_RPM > SERVO_MOTOR_RATED_SPEED_RPM
#error "Motion profile speeds must be ordered and not exceed the motor rating."
#endif
#if (LIFT_PROFILE_CRUISE_RPM * 1ULL * SERVO_COMMAND_UNITS_PER_REVOLUTION / 60ULL) > UINT32_MAX
#error "Motion profile speed units must fit uint32_t."
#endif

static bool ProfileSettingsValid(const LiftPositions *settings)
{
    if (settings == NULL || (settings->profile_flags & ~LIFT_PROFILE_KNOWN_FLAGS) ||
        settings->cruise_rpm < LIFT_PROFILE_SLOW_RPM ||
        settings->cruise_rpm > LIFT_PROFILE_CRUISE_RPM) return false;
    return settings->lower_edge_units == LIFT_WORK_MIN_UNITS &&
           settings->upper_edge_units == LIFT_WORK_MAX_UNITS;
}

bool LiftMotion_CanVerify(const LiftPositions *settings)
{
    return ProfileSettingsValid(settings);
}

int32_t LiftMotion_MinTarget(const LiftPositions *settings)
{
    (void)settings;
    return LIFT_WORK_MIN_UNITS;
}

int32_t LiftMotion_MaxTarget(const LiftPositions *settings)
{
    (void)settings;
    return LIFT_WORK_MAX_UNITS;
}

static uint32_t SpeedUnits(uint32_t rpm)
{
    return (uint32_t)((uint64_t)rpm * SERVO_COMMAND_UNITS_PER_REVOLUTION / 60U);
}

bool LiftMotion_EndpointSettlingAllowed(int64_t current, int64_t target,
                                     bool up, int32_t speed_rpm,
                                     uint32_t speed_units_s)
{
    int64_t edge = up ? LIFT_WORK_MAX_UNITS : LIFT_WORK_MIN_UNITS;
    int64_t speed = speed_rpm;
    if (speed < 0) speed = -speed;
    return target == edge && current >= edge - LIFT_POSITION_TOLERANCE_UNITS &&
           current <= edge + LIFT_POSITION_TOLERANCE_UNITS &&
           speed_units_s > 0U && speed_units_s <= SpeedUnits(LIFT_PROFILE_CRUISE_RPM) &&
           speed <= LIFT_PROFILE_SLOW_RPM + 30U;
}

uint32_t LiftMotion_GuardSpeedRpm(int64_t remaining_units, uint32_t speed_units_s)
{
    uint32_t cruise = (uint32_t)((uint64_t)speed_units_s * 60U / SERVO_COMMAND_UNITS_PER_REVOLUTION);
    uint32_t margin = cruise / 20U;
    if (margin < 30U) margin = 30U;
    /* Clamp before multiplication, even for corrupt/untrusted host test inputs. */
    if (remaining_units < 0) remaining_units = 0;
    if (remaining_units > 400L * LIFT_UNITS_PER_MM) remaining_units = 400L * LIFT_UNITS_PER_MM;
    uint64_t square = 2ULL * SERVO_PP_DECELERATION_PULSES_PER_SECOND2 *
        ((uint64_t)remaining_units + LIFT_UNITS_PER_MM / 2U);
    uint64_t lo = 0, hi = SpeedUnits(LIFT_PROFILE_CRUISE_RPM) + 1ULL;
    while (lo + 1U < hi)
    {
        uint64_t mid = (lo + hi) / 2U;
        if (mid * mid <= square) lo = mid;
        else hi = mid;
    }
    uint32_t curve = (uint32_t)(lo * 60U / SERVO_COMMAND_UNITS_PER_REVOLUTION) + 150U;
    uint32_t cap = cruise + margin;
    return curve < cap ? curve : cap;
}

bool LiftMotion_Plan(int32_t current, int32_t target,
                     const LiftPositions *settings, LiftMotionSegment *segment)
{
    if (segment == NULL) return false;
    segment->target_units = current;
    segment->speed_units_s = 0U;
    if (!ProfileSettingsValid(settings)) return false;
    const int64_t here = current;
    const int64_t final = target;
    const int32_t minimum = LiftMotion_MinTarget(settings);
    const int32_t maximum = LiftMotion_MaxTarget(settings);
    if (minimum >= maximum || final < minimum || final > maximum) return false;

    /* A target inside the permitted range is necessarily inward from outside. */
    if (here < minimum || here > maximum)
    {
        segment->target_units = here < minimum ? minimum : maximum;
        segment->speed_units_s = SpeedUnits(LIFT_PROFILE_SLOW_RPM);
        return true;
    }
    if (here == final) return true;

    segment->target_units = target;
    /* Configure ramps before starting; send the final target exactly once.
       Runtime PP velocity rewrites/retriggers are not used on this drive. */
    segment->speed_units_s = SpeedUnits(settings->cruise_rpm);
    return true;
}
