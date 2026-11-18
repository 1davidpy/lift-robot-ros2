#include "bsp_analog.h"

#include "bsp_pin.h"

#ifndef BSP_JOYSTICK_LEFT_Y_ADC_CHANNEL
#error "BSP_Analog requires project ADC channel macros in bsp_pin.h"
#endif

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t clock;
    uint8_t adc_channel;
} BspAnalogPin;

static const BspAnalogPin analog_pins[BSP_ANALOG_COUNT] = {
    [BSP_ANALOG_LEFT_Y] = {
        .port = BSP_JOYSTICK_LEFT_X_PORT,
        .pin = BSP_JOYSTICK_LEFT_X_PIN,
        .clock = BSP_JOYSTICK_LEFT_X_CLOCK,
        .adc_channel = BSP_JOYSTICK_LEFT_X_ADC_CHANNEL,
    },
    [BSP_ANALOG_LEFT_X] = {
        .port = BSP_JOYSTICK_LEFT_Y_PORT,
        .pin = BSP_JOYSTICK_LEFT_Y_PIN,
        .clock = BSP_JOYSTICK_LEFT_Y_CLOCK,
        .adc_channel = BSP_JOYSTICK_LEFT_Y_ADC_CHANNEL,
    },
    [BSP_ANALOG_RIGHT_Y] = {
        .port = BSP_JOYSTICK_RIGHT_X_PORT,
        .pin = BSP_JOYSTICK_RIGHT_X_PIN,
        .clock = BSP_JOYSTICK_RIGHT_X_CLOCK,
        .adc_channel = BSP_JOYSTICK_RIGHT_X_ADC_CHANNEL,
    },
    [BSP_ANALOG_RIGHT_X] = {
        .port = BSP_JOYSTICK_RIGHT_Y_PORT,
        .pin = BSP_JOYSTICK_RIGHT_Y_PIN,
        .clock = BSP_JOYSTICK_RIGHT_Y_CLOCK,
        .adc_channel = BSP_JOYSTICK_RIGHT_Y_ADC_CHANNEL,
    },
    [BSP_ANALOG_WHEEL_LEFT] = {
        .port = BSP_WHEEL_LEFT_PORT,
        .pin = BSP_WHEEL_LEFT_PIN,
        .clock = BSP_WHEEL_LEFT_CLOCK,
        .adc_channel = BSP_WHEEL_LEFT_ADC_CHANNEL,
    },
    [BSP_ANALOG_WHEEL_RIGHT] = {
        .port = BSP_WHEEL_RIGHT_PORT,
        .pin = BSP_WHEEL_RIGHT_PIN,
        .clock = BSP_WHEEL_RIGHT_CLOCK,
        .adc_channel = BSP_WHEEL_RIGHT_ADC_CHANNEL,
    },
};

static void analog_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init;

    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AIN;

    for (uint32_t i = 0U; i < BSP_ANALOG_COUNT; i++)
    {
        RCC_APB2PeriphClockCmd(analog_pins[i].clock, ENABLE);
        gpio_init.GPIO_Pin = analog_pins[i].pin;
        GPIO_Init(analog_pins[i].port, &gpio_init);
    }
}

void BSP_Analog_Init(void)
{
    ADC_InitTypeDef adc_init;

    analog_gpio_init();

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_DeInit(ADC1);
    adc_init.ADC_Mode = ADC_Mode_Independent;
    adc_init.ADC_ScanConvMode = DISABLE;
    adc_init.ADC_ContinuousConvMode = DISABLE;
    adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc_init.ADC_DataAlign = ADC_DataAlign_Right;
    adc_init.ADC_NbrOfChannel = 1U;
    ADC_Init(ADC1, &adc_init);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) != RESET)
    {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) != RESET)
    {
    }
}

uint16_t BSP_Analog_Read(BspAnalogChannel channel)
{
    if (channel >= BSP_ANALOG_COUNT)
    {
        return 0U;
    }

    ADC_RegularChannelConfig(ADC1,
                             analog_pins[channel].adc_channel,
                             1U,
                             ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
    }

    return ADC_GetConversionValue(ADC1);
}
