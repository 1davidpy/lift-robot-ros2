#include "lift_positions.h"
#include "lift_motion_profile.h"
#include <stddef.h>
#include <string.h>
#include "stm32f10x_flash.h"
#include "FreeRTOS.h"
#include "task.h"

/* Two independent 1 KiB pages, excluded from the application linker region. */
#define SLOT_A 0x0800F800UL
#define SLOT_B 0x0800FC00UL
#define MAGIC 0x4C465031UL

_Static_assert(offsetof(LiftPositions, absolute_zero_low) == 52U, "v2 CRC offset changed");
_Static_assert(offsetof(LiftPositions, lower_edge_units) == 76U, "v3/v4 CRC offset changed");
_Static_assert(offsetof(LiftPositions, crc) == 92U && sizeof(LiftPositions) == 96U,
               "Unexpected v5/v6 calibration layout");

static unsigned PayloadLength(const LiftPositions *p)
{
    if (p->version == 2U) return offsetof(LiftPositions, absolute_zero_low);
    if (p->version == 3U || p->version == 4U) return offsetof(LiftPositions, lower_edge_units);
    return offsetof(LiftPositions, crc);
}

static uint32_t Checksum(const LiftPositions *p)
{
    const uint8_t *bytes = (const uint8_t *)p;
    uint32_t crc = 0xFFFFFFFFUL;
    unsigned length = PayloadLength(p);
    for (unsigned i = 0; i < length; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
    return ~crc;
}

static bool Valid(const LiftPositions *p)
{
    if (p->magic != MAGIC || (p->version != 2U && p->version != 3U &&
        p->version != 4U && p->version != 5U &&
        p->version != LIFT_POSITIONS_VERSION)) return false;
    uint32_t stored_crc;
    memcpy(&stored_crc, (const uint8_t *)p + PayloadLength(p), sizeof(stored_crc));
    if (stored_crc != Checksum(p) ||
        (p->mask & ~31U) != 0U || p->home_valid > 3U ||
        (p->up_sign != 1 && p->up_sign != -1)) return false;
    if (p->version >= 3U && (p->absolute_stage > 2U ||
        (p->absolute_stage && (!(p->mask & 4U) ||
         (p->absolute_resolution != 131072U && p->absolute_resolution != 8388608U))) ||
        (p->absolute_stage == 2U && p->absolute_up_sign != 1 && p->absolute_up_sign != -1))) return false;
    if (p->version >= 4U && p->absolute_stage)
    {
        uint32_t kind = (p->absolute_motor / 10U) % 10U;
        uint64_t zero = ((uint64_t)p->absolute_zero_high << 32) | p->absolute_zero_low;
        if ((kind != 2U && kind != 3U) ||
            p->absolute_resolution != (kind == 2U ? 131072U : 8388608U) ||
            zero >= (uint64_t)65536U * p->absolute_resolution) return false;
    }
    if (p->version == 5U)
    {
        if ((p->profile_flags & ~(LIFT_PROFILE_LEGACY_EDGE_FLAGS |
                                  LIFT_PROFILE_BOOT_HOME_ARMED)) ||
            p->cruise_rpm < LIFT_PROFILE_SLOW_RPM || p->cruise_rpm > LIFT_PROFILE_CRUISE_RPM ||
            ((p->profile_flags & LIFT_PROFILE_BOOT_HOME_ARMED) && p->absolute_stage != 2U))
            return false;
        if ((p->profile_flags & LIFT_PROFILE_LOWER_SET) &&
            (p->lower_edge_units < LIFT_MECHANICAL_MIN_MM * LIFT_UNITS_PER_MM ||
             p->lower_edge_units >= 0)) return false;
        if ((p->profile_flags & LIFT_PROFILE_UPPER_SET) &&
            (p->upper_edge_units > LIFT_MECHANICAL_MAX_MM * LIFT_UNITS_PER_MM ||
             p->upper_edge_units <= 0)) return false;
        if ((p->profile_flags & LIFT_PROFILE_END_VERIFIED) &&
            (((p->profile_flags & (LIFT_PROFILE_LOWER_SET | LIFT_PROFILE_UPPER_SET)) !=
                (LIFT_PROFILE_LOWER_SET | LIFT_PROFILE_UPPER_SET)) ||
             p->lower_edge_units > -20L * LIFT_UNITS_PER_MM ||
             p->upper_edge_units < 60L * LIFT_UNITS_PER_MM ||
             (int64_t)p->upper_edge_units - p->lower_edge_units <
                LIFT_PROFILE_MIN_SPAN_MM * LIFT_UNITS_PER_MM)) return false;
    }
    if (p->version == LIFT_POSITIONS_VERSION &&
        ((p->profile_flags & ~LIFT_PROFILE_KNOWN_FLAGS) ||
         p->cruise_rpm < LIFT_PROFILE_SLOW_RPM ||
         p->cruise_rpm > LIFT_PROFILE_CRUISE_RPM ||
         ((p->profile_flags & LIFT_PROFILE_BOOT_HOME_ARMED) && p->absolute_stage != 2U) ||
         p->lower_edge_units != LIFT_WORK_MIN_UNITS ||
         p->upper_edge_units != LIFT_WORK_MAX_UNITS)) return false;
    if (p->home_valid && (!(p->mask & 4U) || p->home_edge >= 0 ||
                         p->home_edge < -20L * LIFT_UNITS_PER_MM)) return false;
    int32_t minimum = p->version < LIFT_POSITIONS_VERSION ?
        -100L * LIFT_UNITS_PER_MM : LIFT_WORK_MIN_UNITS;
    int32_t maximum = p->version < LIFT_POSITIONS_VERSION ?
        200L * LIFT_UNITS_PER_MM : LIFT_WORK_MAX_UNITS;
    for (unsigned i = 0; i < LIFT_POINT_COUNT; ++i)
        if (p->points[i] < minimum || p->points[i] > maximum)
            return false;
    return p->points[2] == 0;
}

static const LiftPositions *Latest(void)
{
    const LiftPositions *a = (const LiftPositions *)SLOT_A;
    const LiftPositions *b = (const LiftPositions *)SLOT_B;
    if (!Valid(a)) return Valid(b) ? b : NULL;
    if (!Valid(b)) return a;
    return (int32_t)(b->sequence - a->sequence) > 0 ? b : a;
}

void LiftPositions_Load(LiftPositions *positions)
{
    const LiftPositions *latest = Latest();
    memset(positions, 0, sizeof(*positions));
    positions->cruise_rpm = LIFT_PROFILE_DEFAULT_CRUISE_RPM;
    if (latest != NULL)
    {
        memcpy(positions, latest, PayloadLength(latest));
        /* Only v2/v3 predate the verified raw absolute source. v4 centers,
           direction and stage are preserved exactly; new flags remain zero. */
        if (latest->version < 4U)
            memset(&positions->absolute_zero_low, 0,
                   offsetof(LiftPositions, lower_edge_units) - offsetof(LiftPositions, absolute_zero_low));
        if (latest->version < LIFT_POSITIONS_VERSION)
        {
            /* Only the centre and battery-backed absolute pairing survive the
               transition from optical endpoints to fixed encoder limits. */
            for (unsigned i = 0; i < LIFT_POINT_COUNT; ++i) positions->points[i] = 0;
            positions->mask &= 4U;
            positions->home_edge = 0;
            positions->home_valid = 0;
            positions->lower_edge_units = LIFT_WORK_MIN_UNITS;
            positions->upper_edge_units = LIFT_WORK_MAX_UNITS;
            positions->profile_flags &= LIFT_PROFILE_BOOT_HOME_ARMED;
            if (positions->cruise_rpm < LIFT_PROFILE_SLOW_RPM ||
                positions->cruise_rpm > LIFT_PROFILE_CRUISE_RPM)
                positions->cruise_rpm = LIFT_PROFILE_DEFAULT_CRUISE_RPM;
        }
    }
    else
    {
        positions->lower_edge_units = LIFT_WORK_MIN_UNITS;
        positions->upper_edge_units = LIFT_WORK_MAX_UNITS;
    }
    positions->version = LIFT_POSITIONS_VERSION;
    positions->crc = Checksum(positions);
}

bool LiftPositions_Save(LiftPositions *positions)
{
    const LiftPositions *latest = Latest();
    uint32_t address = latest == (const LiftPositions *)SLOT_A ? SLOT_B : SLOT_A;
    LiftPositions next = *positions;
    next.magic = MAGIC;
    next.version = LIFT_POSITIONS_VERSION;
    next.sequence = latest == NULL ? 1U : latest->sequence + 1U;
    next.crc = Checksum(&next);
    if (!Valid(&next)) return false;
    /* Caller must have confirmed stationary and applied the brake. */
    taskENTER_CRITICAL();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_Status status = FLASH_ErasePage(address);
    for (unsigned i = 0; status == FLASH_COMPLETE && i < sizeof(next); i += 2U)
    {
        const uint8_t *bytes = (const uint8_t *)&next;
        uint16_t half = (uint16_t)(bytes[i] | ((uint16_t)bytes[i + 1U] << 8));
        status = FLASH_ProgramHalfWord(address + i, half);
    }
    FLASH_Lock();
    taskEXIT_CRITICAL();
    if (status != FLASH_COMPLETE || memcmp((const void *)address, &next, sizeof(next)))
        return false;
    *positions = next;
    return true;
}
