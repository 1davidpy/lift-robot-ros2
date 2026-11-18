#ifndef LIFT_MOTION_PROFILE_H
#define LIFT_MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "lift_positions.h"

/* Normal moves use one native PP trapezoid. Only outside-range recovery
   retains a separate slow segment to the nearest software endpoint. */
#define LIFT_PROFILE_MIN_SPAN_MM 120L /* Retained for legacy calibration validation. */
#define LIFT_PROFILE_SLOW_RPM 120U
#define LIFT_PROFILE_CRUISE_RPM 3000U
#define LIFT_POSITION_TOLERANCE_UNITS 100L

/* Only permits low-speed settling at an explicitly commanded endpoint.
   New outward commands must still be rejected by the directional limit. */
bool LiftMotion_EndpointSettlingAllowed(int64_t current, int64_t target,
                                     bool up, int32_t speed_rpm,
                                     uint32_t speed_units_s);

/* Measured-speed ceiling following the native braking curve, including
   bounded feedback regulation / position quantisation allowances. */
uint32_t LiftMotion_GuardSpeedRpm(int64_t remaining_units, uint32_t speed_units_s);

typedef struct
{
    int32_t target_units;
    uint32_t speed_units_s;
} LiftMotionSegment;

/**
 * @brief Check the fixed encoder profile and cruise speed.
 * @return False for an invalid setting.
 */
bool LiftMotion_CanVerify(const LiftPositions *settings);

/**
 * @brief Get the permitted lower target in signed command units.
 * @warning Callers must still handle LiftMotion_Plan rejecting invalid settings.
 */
int32_t LiftMotion_MinTarget(const LiftPositions *settings);

/**
 * @brief Get the permitted upper target in signed command units.
 * @warning Callers must still handle LiftMotion_Plan rejecting invalid settings.
 */
int32_t LiftMotion_MaxTarget(const LiftPositions *settings);

/**
 * @brief Plan one PP segment relative to the existing center, without motion IO.
 * @param current Current position in LIFT_UNITS_PER_MM units per mm.
 * @param target Final requested position; targets are never silently clamped.
 * @param settings Saved absolute reference and cruise speed.
 * @param segment Output segment; speed is zero when already at the target.
 * @return False for null arguments, invalid settings or an out-of-range target.
 * @warning Position validity and standstill are the caller's responsibility.
 *          An outside position first returns to the nearest endpoint at 120 rpm.
 */
bool LiftMotion_Plan(int32_t current, int32_t target,
                     const LiftPositions *settings, LiftMotionSegment *segment);

#endif /* LIFT_MOTION_PROFILE_H */
