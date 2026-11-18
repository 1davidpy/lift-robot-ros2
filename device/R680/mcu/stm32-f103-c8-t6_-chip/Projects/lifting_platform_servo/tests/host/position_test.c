/* Native pure-logic tests for the encoder-only fixed-range motion profile. */
#include <stdio.h>
#include <stdlib.h>

#include "bsp_position_sensor_config.h"
#include "lift_motion_profile.h"

#include "../../Src/lift_motion_profile.c"

static unsigned checks;

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

static LiftPositions Settings(uint32_t cruise_rpm)
{
    LiftPositions settings = {0};
    settings.version = LIFT_POSITIONS_VERSION;
    settings.mask = 4U;
    settings.points[2] = 0;
    settings.absolute_stage = 2U;
    settings.lower_edge_units = LIFT_WORK_MIN_UNITS;
    settings.upper_edge_units = LIFT_WORK_MAX_UNITS;
    settings.cruise_rpm = cruise_rpm;
    settings.profile_flags = LIFT_PROFILE_BOOT_HOME_ARMED;
    return settings;
}

static void ExpectPlan(int32_t current, int32_t target,
                       const LiftPositions *settings,
                       int32_t segment_target, uint32_t speed_units_s)
{
    LiftMotionSegment segment = {0};
    CHECK(LiftMotion_Plan(current, target, settings, &segment));
    CHECK(segment.target_units == segment_target);
    CHECK(segment.speed_units_s == speed_units_s);
}

int main(void)
{
    const uint32_t rpm3000 = 3000U * SERVO_COMMAND_UNITS_PER_REVOLUTION / 60U;
    const uint32_t rpm600 = 600U * SERVO_COMMAND_UNITS_PER_REVOLUTION / 60U;
    const uint32_t rpm120 = 120U * SERVO_COMMAND_UNITS_PER_REVOLUTION / 60U;
    LiftPositions settings = Settings(3000U);

    CHECK(BSP_POSITION_SENSOR_COUNT == 1U);
    CHECK(LIFT_POSITIONS_VERSION == 6U);
    CHECK(LIFT_WORK_MIN_MM == -180L && LIFT_WORK_MAX_MM == 180L);
    CHECK(LiftMotion_CanVerify(&settings));
    CHECK(LiftMotion_MinTarget(&settings) == -180L * LIFT_UNITS_PER_MM);
    CHECK(LiftMotion_MaxTarget(&settings) == 180L * LIFT_UNITS_PER_MM);

    ExpectPlan(0, 100L * LIFT_UNITS_PER_MM, &settings,
               100L * LIFT_UNITS_PER_MM, rpm3000);
    ExpectPlan(0, 180L * LIFT_UNITS_PER_MM, &settings,
               180L * LIFT_UNITS_PER_MM, rpm3000);
    ExpectPlan(120L * LIFT_UNITS_PER_MM, 180L * LIFT_UNITS_PER_MM, &settings,
               180L * LIFT_UNITS_PER_MM, rpm3000);
    ExpectPlan(160L * LIFT_UNITS_PER_MM, 180L * LIFT_UNITS_PER_MM, &settings,
               180L * LIFT_UNITS_PER_MM, rpm3000);

    ExpectPlan(0, -180L * LIFT_UNITS_PER_MM, &settings,
               -180L * LIFT_UNITS_PER_MM, rpm3000);
    ExpectPlan(-120L * LIFT_UNITS_PER_MM, -180L * LIFT_UNITS_PER_MM, &settings,
               -180L * LIFT_UNITS_PER_MM, rpm3000);
    ExpectPlan(-160L * LIFT_UNITS_PER_MM, -180L * LIFT_UNITS_PER_MM, &settings,
               -180L * LIFT_UNITS_PER_MM, rpm3000);

    /* Departing an endpoint is immediately allowed at cruise speed. */
    ExpectPlan(-180L * LIFT_UNITS_PER_MM, 0, &settings, 0, rpm3000);
    ExpectPlan(180L * LIFT_UNITS_PER_MM, 0, &settings, 0, rpm3000);

    /* A retained absolute position just outside the work range may only return inward. */
    ExpectPlan(190L * LIFT_UNITS_PER_MM, 0, &settings,
               180L * LIFT_UNITS_PER_MM, rpm120);
    ExpectPlan(-190L * LIFT_UNITS_PER_MM, 0, &settings,
               -180L * LIFT_UNITS_PER_MM, rpm120);

    LiftMotionSegment segment;
    for (int up = 0; up < 2; ++up)
    {
        int32_t edge = up ? LIFT_WORK_MAX_UNITS : LIFT_WORK_MIN_UNITS;
        CHECK(LiftMotion_EndpointSettlingAllowed(edge, edge, up, 0, rpm120));
        CHECK(LiftMotion_EndpointSettlingAllowed(edge - 100, edge, up, 120, rpm120));
        CHECK(LiftMotion_EndpointSettlingAllowed(edge + 100, edge, up, -150, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge - 101, edge, up, 0, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge + 101, edge, up, 0, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge - 1, up, 0, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge, !up, 0, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge, up, 151, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge, up, -151, rpm120));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge, up, INT32_MIN, rpm120));
        CHECK(LiftMotion_EndpointSettlingAllowed(edge, edge, up, 0, rpm3000));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge, up, 0, rpm3000 + 1));
        CHECK(!LiftMotion_EndpointSettlingAllowed(edge, edge, up, 0, 0));
    }
    CHECK(LiftMotion_GuardSpeedRpm(100L * LIFT_UNITS_PER_MM, rpm3000) == 3150);
    CHECK(LiftMotion_GuardSpeedRpm(0, rpm3000) <= 420);
    CHECK(LiftMotion_GuardSpeedRpm(20L * LIFT_UNITS_PER_MM, rpm3000) < 1900);
    CHECK(LiftMotion_GuardSpeedRpm(0, rpm120) == 150);
    CHECK(LiftMotion_GuardSpeedRpm(INT64_MAX, rpm3000) == 3150);
    CHECK(LiftMotion_GuardSpeedRpm(INT64_MIN, rpm3000) == LiftMotion_GuardSpeedRpm(0, rpm3000));
    uint32_t previous_cap = 0;
    for (int mm = 0; mm <= 360; ++mm)
    {
        uint32_t cap = LiftMotion_GuardSpeedRpm(mm * LIFT_UNITS_PER_MM, rpm3000);
        CHECK(cap >= previous_cap && cap <= 3150);
        previous_cap = cap;
    }
    settings = Settings(600);
    ExpectPlan(-180L * LIFT_UNITS_PER_MM, 180L * LIFT_UNITS_PER_MM, &settings,
               180L * LIFT_UNITS_PER_MM, rpm600);
    settings = Settings(3000);
    CHECK(!LiftMotion_Plan(0, 180L * LIFT_UNITS_PER_MM + 1, &settings, &segment));
    CHECK(!LiftMotion_Plan(0, -180L * LIFT_UNITS_PER_MM - 1, &settings, &segment));
    settings.profile_flags |= LIFT_PROFILE_LOWER_SET;
    CHECK(!LiftMotion_Plan(0, 0, &settings, &segment));
    settings = Settings(3000U);
    settings.lower_edge_units++;
    CHECK(!LiftMotion_Plan(0, 0, &settings, &segment));

    printf("PASS: %u encoder-only range/profile checks\n", checks);
    return 0;
}
