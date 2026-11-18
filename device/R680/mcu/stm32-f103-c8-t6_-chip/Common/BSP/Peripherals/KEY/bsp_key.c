#include "bsp_key.h"

#include "bsp_pin.h"

#ifndef BSP_KEY_PRESENT_MASK
#define BSP_KEY_PRESENT_MASK BSP_KEY_MASK_ALL
#endif

/* Unused entries need harmless compile-time placeholders on smaller key sets. */
#ifndef BSP_KEY_TOP_L_PORT
#define BSP_KEY_TOP_L_PORT   GPIOA
#define BSP_KEY_TOP_L_PIN    GPIO_Pin_0
#define BSP_KEY_TOP_L_CLOCK  RCC_APB2Periph_GPIOA
#endif
#ifndef BSP_KEY_TOP_R_PORT
#define BSP_KEY_TOP_R_PORT   GPIOA
#define BSP_KEY_TOP_R_PIN    GPIO_Pin_0
#define BSP_KEY_TOP_R_CLOCK  RCC_APB2Periph_GPIOA
#endif
#ifndef BSP_KEY_MODE_PORT
#define BSP_KEY_MODE_PORT    GPIOA
#define BSP_KEY_MODE_PIN     GPIO_Pin_0
#define BSP_KEY_MODE_CLOCK   RCC_APB2Periph_GPIOA
#endif
#ifndef BSP_KEY_SPEED_PORT
#define BSP_KEY_SPEED_PORT   GPIOA
#define BSP_KEY_SPEED_PIN    GPIO_Pin_0
#define BSP_KEY_SPEED_CLOCK  RCC_APB2Periph_GPIOA
#endif

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t clock;
} BspKeyPin;

static const BspKeyPin key_pins[BSP_KEY_COUNT] = {
    [BSP_KEY_TOP_LEFT] = {
        .port = BSP_KEY_TOP_L_PORT,
        .pin = BSP_KEY_TOP_L_PIN,
        .clock = BSP_KEY_TOP_L_CLOCK,
    },
    [BSP_KEY_TOP_RIGHT] = {
        .port = BSP_KEY_TOP_R_PORT,
        .pin = BSP_KEY_TOP_R_PIN,
        .clock = BSP_KEY_TOP_R_CLOCK,
    },
    [BSP_KEY_BOTTOM_LEFT] = {
        .port = BSP_KEY_MODE_PORT,
        .pin = BSP_KEY_MODE_PIN,
        .clock = BSP_KEY_MODE_CLOCK,
    },
    [BSP_KEY_BOTTOM_RIGHT] = {
        .port = BSP_KEY_SPEED_PORT,
        .pin = BSP_KEY_SPEED_PIN,
        .clock = BSP_KEY_SPEED_CLOCK,
    },
};

void BSP_Key_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init.GPIO_Speed = GPIO_Speed_2MHz;

    for (uint8_t i = 0U; i < (uint8_t)BSP_KEY_COUNT; i++)
    {
        if ((BSP_KEY_PRESENT_MASK & BSP_KEY_MASK(i)) == 0U)
        {
            continue;
        }

        RCC_APB2PeriphClockCmd(key_pins[i].clock, ENABLE);
        gpio_init.GPIO_Pin = key_pins[i].pin;
        GPIO_Init(key_pins[i].port, &gpio_init);
    }
}

bool BSP_Key_IsPressed(BspKeyId key)
{
    if (key >= BSP_KEY_COUNT ||
        (BSP_KEY_PRESENT_MASK & BSP_KEY_MASK(key)) == 0U)
    {
        return false;
    }

    return GPIO_ReadInputDataBit(key_pins[key].port, key_pins[key].pin) == Bit_RESET;
}
