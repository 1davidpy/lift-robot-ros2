#ifndef DRIVER_TEST_GPIO_H
#define DRIVER_TEST_GPIO_H
#include <stdint.h>
typedef struct { uint32_t GPIO_Pin, GPIO_Speed, GPIO_Mode; } GPIO_InitTypeDef;
#define GPIO_Speed_50MHz 0U
#define GPIO_Mode_AF_PP 0U
#define GPIO_Mode_IN_FLOATING 0U
void GPIO_Init(void *port, GPIO_InitTypeDef *init);
#endif
