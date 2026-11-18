#include "stm32f10x.h"

#include "bsp_pin.h"
#include "gps_output.h"

#define MASTER_TIMER_PERIOD          999U
#define MASTER_TIMER_PRESCALER       7199U
#define PWM_CCR_VALUE                100U
#define PPS_MASTER_PERIODS           10U
#define PWM_1HZ_CCR_VALUE            1U

#define BREATH_PWM_TICK_HZ           10000U
#define BREATH_PWM_PERIOD_TICKS      50U
#define BREATH_ENVELOPE_STEP_TICKS   100U
#define BREATH_MAX_DUTY              BREATH_PWM_PERIOD_TICKS

void SysTick_Handler(void)
{
    static uint16_t envelope_step_tick = 0U;
    static uint8_t pwm_phase = 0U;
    static uint8_t duty = 0U;
    static uint8_t increasing = 1U;

    pwm_phase++;
    if (pwm_phase >= BREATH_PWM_PERIOD_TICKS)
    {
        pwm_phase = 0U;
    }

    envelope_step_tick++;
    if (envelope_step_tick >= BREATH_ENVELOPE_STEP_TICKS)
    {
        envelope_step_tick = 0U;

        if (increasing != 0U)
        {
            if (duty < BREATH_MAX_DUTY)
            {
                duty++;
            }
            else
            {
                increasing = 0U;
                duty--;
            }
        }
        else if (duty > 0U)
        {
            duty--;
        }
        else
        {
            increasing = 1U;
            duty++;
        }
    }

    /* The PC13 board LED is active-low; open-drain low turns it on. */
    if (pwm_phase < duty)
    {
        BSP_HEARTBEAT_GPIO->BSRR = GPIO_BSRR_BR13;
    }
    else
    {
        BSP_HEARTBEAT_GPIO->BSRR = GPIO_BSRR_BS13;
    }
}

static void Gpio_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    AFIO->MAPR = (AFIO->MAPR & ~AFIO_MAPR_TIM3_REMAP) |
                 BSP_PWM_1HZ_TIMER_REMAP;

    /* PA1: TIM2_CH2, 10 Hz PWM output, alternate function push-pull, 50 MHz. */
    BSP_PWM_10HZ_GPIO->CRL &= ~BSP_PWM_10HZ_GPIO_MODE_MASK;
    BSP_PWM_10HZ_GPIO->CRL |= BSP_PWM_10HZ_GPIO_AF_CONFIG;

    /* PB5: TIM3_CH2, 1 Hz PPS output, alternate function push-pull, 50 MHz. */
    BSP_PWM_1HZ_GPIO->CRL &= ~BSP_PWM_1HZ_GPIO_MODE_MASK;
    BSP_PWM_1HZ_GPIO->CRL |= BSP_PWM_1HZ_GPIO_AF_CONFIG;

    /* PA9: USART1_TX, simulated GPS output, alternate function push-pull. */
    BSP_GPS_TX_GPIO->CRH &= ~BSP_GPS_TX_GPIO_MODE_MASK;
    BSP_GPS_TX_GPIO->CRH |= BSP_GPS_TX_GPIO_AF_CONFIG;

    /* PC13: breathing LED, open-drain output, 2 MHz. */
    BSP_HEARTBEAT_GPIO->CRH &= ~BSP_HEARTBEAT_GPIO_MODE_MASK;
    BSP_HEARTBEAT_GPIO->CRH |= BSP_HEARTBEAT_GPIO_OUTPUT_CONFIG;
    BSP_HEARTBEAT_GPIO->BSRR = GPIO_BSRR_BS13;
}

static void TimerChannel2Pwm_Init(TIM_TypeDef *timer, uint16_t period, uint16_t compare)
{
    timer->ARR = period;
    timer->CCR2 = compare;
    timer->CCMR1 = (timer->CCMR1 & ~(TIM_CCMR1_OC2M | TIM_CCMR1_CC2S)) |
                   TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    timer->CCER = (timer->CCER & ~TIM_CCER_CC2P) | TIM_CCER_CC2E;
    timer->CR1 |= TIM_CR1_ARPE;
}

static void SynchronizedOutput_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    BSP_PWM_10HZ_TIMER->CR1 = 0U;
    BSP_PWM_10HZ_TIMER->CR2 = 0U;
    BSP_PWM_10HZ_TIMER->SMCR = 0U;
    BSP_PWM_10HZ_TIMER->PSC = MASTER_TIMER_PRESCALER;
    TimerChannel2Pwm_Init(BSP_PWM_10HZ_TIMER, MASTER_TIMER_PERIOD, PWM_CCR_VALUE);
    BSP_PWM_10HZ_TIMER->EGR = TIM_EGR_UG;
    BSP_PWM_10HZ_TIMER->CR2 = TIM_CR2_MMS_1; /* Update event is TIM2 TRGO. */

    BSP_PWM_1HZ_TIMER->CR1 = 0U;
    BSP_PWM_1HZ_TIMER->CR2 = 0U;
    BSP_PWM_1HZ_TIMER->SMCR = 0U;
    BSP_PWM_1HZ_TIMER->PSC = 0U;
    TimerChannel2Pwm_Init(BSP_PWM_1HZ_TIMER,
                         PPS_MASTER_PERIODS - 1U,
                         PWM_1HZ_CCR_VALUE);
    BSP_PWM_1HZ_TIMER->CNT = 0U;
    BSP_PWM_1HZ_TIMER->SR = 0U;
    BSP_PWM_1HZ_TIMER->DIER = TIM_DIER_UIE;
    BSP_PWM_1HZ_TIMER->SMCR = TIM_SMCR_TS_0 | TIM_SMCR_SMS_2 |
                              TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
    NVIC_SetPriority(TIM3_IRQn, 1U);
    NVIC_EnableIRQ(TIM3_IRQn);
    BSP_PWM_1HZ_TIMER->CR1 |= TIM_CR1_CEN;

    BSP_PWM_10HZ_TIMER->CR1 |= TIM_CR1_CEN;
}

void TIM3_IRQHandler(void)
{
    if ((BSP_PWM_1HZ_TIMER->SR & TIM_SR_UIF) != 0U)
    {
        BSP_PWM_1HZ_TIMER->SR &= ~TIM_SR_UIF;
        GPS_OutputOnSecondTick();
    }
}

int main(void)
{
    SystemCoreClockUpdate();

    Gpio_Init();
    GPS_OutputInit();
    SynchronizedOutput_Init();
    (void)SysTick_Config(SystemCoreClock / BREATH_PWM_TICK_HZ);

    for (;;)
    {
        GPS_OutputProcess();
    }
}
