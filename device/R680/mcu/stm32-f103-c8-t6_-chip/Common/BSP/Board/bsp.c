#include "bsp.h"

#include "bsp_led.h"
#include "bsp_pin.h"

#ifndef BSP_DEBUG_KEEP_SWD
#define BSP_DEBUG_KEEP_SWD 0
#endif

#if BSP_DEBUG_KEEP_SWD
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#endif

#ifdef BSP_BRAKE_RELAY_ENABLED
#include "bsp_brake_relay.h"
#endif

#ifdef BSP_POSITION_SENSORS_ENABLED
#include "bsp_position_sensor.h"
#endif

void BSP_Init(void)
{
#if BSP_DEBUG_KEEP_SWD
    /** @brief Keep PA13/PA14 available for SWD before peripheral and task startup. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
#endif
    BSP_LED_Init();
#ifdef BSP_BRAKE_RELAY_ENABLED
    BSP_BrakeRelay_Init();
#endif
#ifdef BSP_POSITION_SENSORS_ENABLED
    BSP_PositionSensor_Init();
#endif
}
