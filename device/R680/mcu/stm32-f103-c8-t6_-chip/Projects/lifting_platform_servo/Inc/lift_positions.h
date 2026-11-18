#ifndef LIFT_POSITIONS_H
#define LIFT_POSITIONS_H
#include <stdbool.h>
#include <stdint.h>

#define LIFT_POINT_COUNT 5U
#define LIFT_POSITIONS_VERSION 6U
#define LIFT_UNITS_PER_MM 10000L /* Provisional: 10000 command units/rev, 1 mm/rev. */
#define LIFT_WORK_MIN_MM (-180L)
#define LIFT_WORK_MAX_MM 180L
#define LIFT_WORK_MIN_UNITS (LIFT_WORK_MIN_MM * LIFT_UNITS_PER_MM)
#define LIFT_WORK_MAX_UNITS (LIFT_WORK_MAX_MM * LIFT_UNITS_PER_MM)
/* Absolute feedback may be recovered just outside the commandable range so an
   inward correction remains possible. No command may target this reserve. */
#define LIFT_MECHANICAL_MIN_MM (-200L)
#define LIFT_MECHANICAL_MAX_MM 200L

#define LIFT_PROFILE_LOWER_SET       (1UL << 0)
#define LIFT_PROFILE_UPPER_SET       (1UL << 1)
#define LIFT_PROFILE_END_VERIFIED    (1UL << 2)
#define LIFT_PROFILE_BOOT_HOME_ARMED (1UL << 3)
#define LIFT_PROFILE_LEGACY_EDGE_FLAGS 7UL
#define LIFT_PROFILE_KNOWN_FLAGS     LIFT_PROFILE_BOOT_HOME_ARMED
#define LIFT_PROFILE_DEFAULT_CRUISE_RPM 3000U

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t mask;
    int32_t points[LIFT_POINT_COUNT]; /* Positive is physically up, relative to point 3. */
    int32_t home_edge;
    uint32_t home_valid;
    int32_t up_sign;
    int32_t taught_origin; /* Audit only: not trusted across a drive power cycle. */
    uint32_t absolute_zero_low; /* v4/v5: raw turns * resolution + single-turn count. */
    uint32_t absolute_zero_high;
    uint32_t absolute_resolution;
    uint32_t absolute_motor;
    int32_t absolute_up_sign;
    uint32_t absolute_stage; /* 0 unavailable, 1 zero captured, 2 direction verified. */
    /* v5 optical-edge fields are retained for binary migration only. v6 writes
       the fixed encoder limits here for diagnostics and does not use sensors. */
    int32_t lower_edge_units;
    int32_t upper_edge_units;
    uint32_t cruise_rpm;
    uint32_t profile_flags;
    uint32_t crc;
} LiftPositions;

void LiftPositions_Load(LiftPositions *positions);
bool LiftPositions_Save(LiftPositions *positions);
#endif
