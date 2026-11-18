#include "bsp_position_sensor.h"

#include "bsp_pin.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t clock;
} BspPositionSensorPin;

static const BspPositionSensorPin position_sensor_pins[BSP_POSITION_SENSOR_COUNT] = {
    {BSP_POSITION_SENSOR_0_PORT,
     BSP_POSITION_SENSOR_0_PIN,
     BSP_POSITION_SENSOR_0_CLOCK},
#if BSP_POSITION_SENSOR_COUNT > 1U
    {BSP_POSITION_SENSOR_1_PORT,
     BSP_POSITION_SENSOR_1_PIN,
     BSP_POSITION_SENSOR_1_CLOCK},
#endif
#if BSP_POSITION_SENSOR_COUNT > 2U
    {BSP_POSITION_SENSOR_2_PORT,
     BSP_POSITION_SENSOR_2_PIN,
     BSP_POSITION_SENSOR_2_CLOCK},
#endif
#if BSP_POSITION_SENSOR_COUNT > 3U
    {BSP_POSITION_SENSOR_3_PORT,
     BSP_POSITION_SENSOR_3_PIN,
     BSP_POSITION_SENSOR_3_CLOCK},
#endif
#if BSP_POSITION_SENSOR_COUNT > 4U
    {BSP_POSITION_SENSOR_4_PORT,
     BSP_POSITION_SENSOR_4_PIN,
     BSP_POSITION_SENSOR_4_CLOCK},
#endif
#if BSP_POSITION_SENSOR_COUNT > 5U
    {BSP_POSITION_SENSOR_5_PORT,
     BSP_POSITION_SENSOR_5_PIN,
     BSP_POSITION_SENSOR_5_CLOCK},
#endif
#if BSP_POSITION_SENSOR_COUNT > 6U
    {BSP_POSITION_SENSOR_6_PORT,
     BSP_POSITION_SENSOR_6_PIN,
     BSP_POSITION_SENSOR_6_CLOCK},
#endif
#if BSP_POSITION_SENSOR_COUNT > 7U
    {BSP_POSITION_SENSOR_7_PORT,
     BSP_POSITION_SENSOR_7_PIN,
     BSP_POSITION_SENSOR_7_CLOCK},
#endif
};

void BSP_PositionSensor_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.GPIO_Mode = GPIO_Mode_IPD;
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;

    for (uint8_t i = 0U; i < BSP_POSITION_SENSOR_COUNT; i++)
    {
        RCC_APB2PeriphClockCmd(position_sensor_pins[i].clock, ENABLE);
        gpio_init.GPIO_Pin = position_sensor_pins[i].pin;
        GPIO_Init(position_sensor_pins[i].port, &gpio_init);
    }
}

uint8_t BSP_PositionSensor_ReadMask(void)
{
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < BSP_POSITION_SENSOR_COUNT; i++)
    {
        if (GPIO_ReadInputDataBit(position_sensor_pins[i].port,
                                  position_sensor_pins[i].pin) == Bit_SET)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}
