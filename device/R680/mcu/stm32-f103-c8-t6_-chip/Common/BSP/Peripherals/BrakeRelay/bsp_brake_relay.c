#include "bsp_brake_relay.h"

#include "bsp_pin.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

static bool brake_released;

void BSP_BrakeRelay_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(BSP_BRAKE_RELAY_CLOCK, ENABLE);

    /* Establish the safe low level before changing the pin to an output. */
    GPIO_ResetBits(BSP_BRAKE_RELAY_PORT, BSP_BRAKE_RELAY_PIN);
    gpio_init.GPIO_Pin = BSP_BRAKE_RELAY_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(BSP_BRAKE_RELAY_PORT, &gpio_init);
    brake_released = false;
}

void BSP_BrakeRelay_SetReleased(bool released)
{
    if (released)
    {
        GPIO_SetBits(BSP_BRAKE_RELAY_PORT, BSP_BRAKE_RELAY_PIN);
    }
    else
    {
        GPIO_ResetBits(BSP_BRAKE_RELAY_PORT, BSP_BRAKE_RELAY_PIN);
    }
    brake_released = released;
}

bool BSP_BrakeRelay_IsReleased(void)
{
    return brake_released;
}
