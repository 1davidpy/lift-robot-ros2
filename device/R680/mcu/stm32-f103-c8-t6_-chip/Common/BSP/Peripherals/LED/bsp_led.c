#include "bsp_led.h"

#include "bsp_pin.h"

#ifndef BSP_LED_STATUS_AVAILABLE
#define BSP_LED_STATUS_AVAILABLE 1
#endif

#ifndef BSP_LED_AUX_AVAILABLE
#define BSP_LED_AUX_AVAILABLE 0
#endif

#if BSP_LED_STATUS_AVAILABLE || BSP_LED_AUX_AVAILABLE
static const BspPinConfig led_pins[BSP_LED_COUNT] = {
#if BSP_LED_STATUS_AVAILABLE
    [BSP_LED_STATUS] = {
        .port = BSP_LED_STATUS_PORT,
        .pin = BSP_LED_STATUS_PIN,
        .clock = BSP_LED_STATUS_CLOCK,
        .mode = BSP_LED_STATUS_MODE,
        .speed = BSP_LED_STATUS_SPEED,
        .active_level = BSP_LED_STATUS_ACTIVE,
    },
#endif
#if BSP_LED_AUX_AVAILABLE
    [BSP_LED_AUX] = {
        .port = BSP_LED_AUX_PORT,
        .pin = BSP_LED_AUX_PIN,
        .clock = BSP_LED_AUX_CLOCK,
        .mode = BSP_LED_AUX_MODE,
        .speed = BSP_LED_AUX_SPEED,
        .active_level = BSP_LED_AUX_ACTIVE,
    },
#endif
};

static uint8_t led_available(BspLed led)
{
    return (uint32_t)led < BSP_LED_COUNT && led_pins[led].port != 0;
}

static void led_apply(const BspPinConfig *config, uint8_t active)
{
    uint8_t set_pin = (config->active_level == BSP_PIN_ACTIVE_HIGH) ? active : !active;

    if (set_pin != 0U)
    {
        GPIO_SetBits(config->port, config->pin);
    }
    else
    {
        GPIO_ResetBits(config->port, config->pin);
    }
}
#endif

void BSP_LED_Init(void)
{
#if BSP_LED_STATUS_AVAILABLE || BSP_LED_AUX_AVAILABLE
    GPIO_InitTypeDef gpio_init;

    for (uint32_t i = 0; i < BSP_LED_COUNT; i++)
    {
        const BspPinConfig *config = &led_pins[i];

        if (!led_available((BspLed)i)) continue;

        RCC_APB2PeriphClockCmd(config->clock, ENABLE);
        led_apply(config, 0U);

        gpio_init.GPIO_Pin = config->pin;
        gpio_init.GPIO_Speed = config->speed;
        gpio_init.GPIO_Mode = config->mode;
        GPIO_Init(config->port, &gpio_init);

    }
#endif
}

void BSP_LED_On(BspLed led)
{
#if BSP_LED_STATUS_AVAILABLE || BSP_LED_AUX_AVAILABLE
    if (led_available(led))
    {
        led_apply(&led_pins[led], 1U);
    }
#else
    (void)led;
#endif
}

void BSP_LED_Off(BspLed led)
{
#if BSP_LED_STATUS_AVAILABLE || BSP_LED_AUX_AVAILABLE
    if (led_available(led))
    {
        led_apply(&led_pins[led], 0U);
    }
#else
    (void)led;
#endif
}

void BSP_LED_Toggle(BspLed led)
{
#if BSP_LED_STATUS_AVAILABLE || BSP_LED_AUX_AVAILABLE
    if (led_available(led))
    {
        const BspPinConfig *config = &led_pins[led];
        config->port->ODR ^= config->pin;
    }
#else
    (void)led;
#endif
}

void BSP_LED_Write(BspLed led, uint8_t state)
{
#if BSP_LED_STATUS_AVAILABLE || BSP_LED_AUX_AVAILABLE
    if (led_available(led))
    {
        led_apply(&led_pins[led], state != 0U);
    }
#else
    (void)led;
    (void)state;
#endif
}
