#ifndef TEST_FLASH_H
#define TEST_FLASH_H
#include <stdint.h>
typedef enum { FLASH_COMPLETE, FLASH_ERROR_PG } FLASH_Status;
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_PGERR 2U
#define FLASH_FLAG_WRPRTERR 4U
void FLASH_Unlock(void);
void FLASH_Lock(void);
void FLASH_ClearFlag(uint32_t flags);
FLASH_Status FLASH_ErasePage(uint32_t address);
FLASH_Status FLASH_ProgramHalfWord(uint32_t address, uint16_t value);
#endif
