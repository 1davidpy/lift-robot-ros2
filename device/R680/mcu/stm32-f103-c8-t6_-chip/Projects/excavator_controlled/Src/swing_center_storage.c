#include "swing_center_storage.h"

#include <stddef.h>

#include "stm32f10x_flash.h"

#define SWING_CENTER_FLASH_PAGE_ADDRESS  0x0800FC00UL
#define SWING_CENTER_STORAGE_MAGIC       0x53434E54UL
#define SWING_CENTER_STORAGE_VERSION     1UL
#define SWING_CENTER_CHECK_XOR           0xA55AC33CUL

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t position_bits;
    uint32_t checksum;
} SwingCenterStorageRecord;

static uint32_t SwingCenterStorage_Checksum(uint32_t position_bits)
{
    return SWING_CENTER_STORAGE_VERSION ^ position_bits ^ SWING_CENTER_CHECK_XOR;
}

bool SwingCenterStorage_Load(int32_t *position_pulses)
{
    const volatile SwingCenterStorageRecord *record =
        (const volatile SwingCenterStorageRecord *)SWING_CENTER_FLASH_PAGE_ADDRESS;

    if (position_pulses == NULL ||
        record->magic != SWING_CENTER_STORAGE_MAGIC ||
        record->version != SWING_CENTER_STORAGE_VERSION ||
        record->checksum != SwingCenterStorage_Checksum(record->position_bits))
    {
        return false;
    }

    *position_pulses = (int32_t)record->position_bits;
    return true;
}

bool SwingCenterStorage_Save(int32_t position_pulses)
{
    uint32_t position_bits = (uint32_t)position_pulses;
    FLASH_Status status;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    status = FLASH_ErasePage(SWING_CENTER_FLASH_PAGE_ADDRESS);
    if (status == FLASH_COMPLETE)
    {
        status = FLASH_ProgramWord(SWING_CENTER_FLASH_PAGE_ADDRESS + 4U,
                                   SWING_CENTER_STORAGE_VERSION);
    }
    if (status == FLASH_COMPLETE)
    {
        status = FLASH_ProgramWord(SWING_CENTER_FLASH_PAGE_ADDRESS + 8U,
                                   position_bits);
    }
    if (status == FLASH_COMPLETE)
    {
        status = FLASH_ProgramWord(SWING_CENTER_FLASH_PAGE_ADDRESS + 12U,
                                   SwingCenterStorage_Checksum(position_bits));
    }
    if (status == FLASH_COMPLETE)
    {
        status = FLASH_ProgramWord(SWING_CENTER_FLASH_PAGE_ADDRESS,
                                   SWING_CENTER_STORAGE_MAGIC);
    }

    FLASH_Lock();
    return status == FLASH_COMPLETE &&
           SwingCenterStorage_Load(&position_pulses);
}
