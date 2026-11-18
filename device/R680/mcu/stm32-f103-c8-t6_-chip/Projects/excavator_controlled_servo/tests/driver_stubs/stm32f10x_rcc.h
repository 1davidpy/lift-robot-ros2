#ifndef DRIVER_TEST_RCC_H
#define DRIVER_TEST_RCC_H
#include <stdint.h>
void RCC_APB2PeriphClockCmd(uint32_t clock, int enable);
void RCC_APB1PeriphClockCmd(uint32_t clock, int enable);
#endif
