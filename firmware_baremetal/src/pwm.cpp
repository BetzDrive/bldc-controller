/*
 * PWM HAL implementation for STM32F405.
 * TIM1: Motor PWM (center-aligned, 40kHz, active low)
 * TIM3: ADC trigger (one-pulse, slaved to TIM1)
 * TIM5: LED PWM (84MHz, active low)
 */

#include "hal/hal_pwm.h"
#include "baremetal_config.h"
#include "stm32f4xx.h"

/* Motor PWM auto-reload: 168MHz / 40kHz = 4200 */
#define MOTOR_PWM_ARR  (APB2_TIMER_HZ / 40000U)

/* LED PWM auto-reload (matches existing ChibiOS config) */
#define LED_PWM_ARR    52500U

/* ── Motor callback ──────────────────────────────── */

static hal_pwm_motor_callback_t motor_callback;

extern "C" void hal_pwm_motor_set_callback(hal_pwm_motor_callback_t cb) {
    motor_callback = cb;
}

/* ── TIM1 Update ISR ─────────────────────────────── */

extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
    TIM1->SR &= ~TIM_SR_UIF;
    if (motor_callback) {
        motor_callback();
    }
}

/* ── Motor PWM (TIM1) + ADC trigger (TIM3) ───────── */

extern "C" void hal_pwm_motor_init(void) {
    /* Enable TIM1 clock (APB2) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    /* Configure TIM1 */
    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = MOTOR_PWM_ARR;
    TIM1->RCR = 1;  /* Update every 2 overflows -> 20kHz update rate */

    /* Center-aligned mode 2 (CMS=10) */
    TIM1->CR1 = TIM_CR1_CMS_1;

    /* CH1/2/3: PWM Mode 1 (OCxM=110), preload enable */
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE
                 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;

    /* All channels: output enable, active LOW (CCxP=1) */
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P
               | TIM_CCER_CC2E | TIM_CCER_CC2P
               | TIM_CCER_CC3E | TIM_CCER_CC3P;

    /* Main output enable */
    TIM1->BDTR = TIM_BDTR_MOE;

    /* MMS=010: Update event as trigger output (TRGO) for TIM3 */
    TIM1->CR2 = TIM_CR2_MMS_1;

    /* Enable update interrupt */
    TIM1->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, NVIC_PRIO_TIM1_UP);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    /* Initialize duties to 0 */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    /* ── TIM3: ADC trigger, one-pulse slaved to TIM1 ── */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CR1 = TIM_CR1_OPM;   /* One-pulse mode */
    TIM3->PSC = 1;              /* 84MHz / 2 = 42MHz (matches adc_pwm_cycle_freq) */
    TIM3->ARR = 5;              /* Short delay after TIM1 trigger */

    /* MMS=010: Update as TRGO -> triggers ADCs */
    TIM3->CR2 = TIM_CR2_MMS_1;

    /* TS=000 (ITR0=TIM1), SMS=110 (trigger mode) */
    TIM3->SMCR = (TIM3->SMCR & ~(TIM_SMCR_TS | TIM_SMCR_SMS))
               | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
}

extern "C" void hal_pwm_motor_start(void) {
    TIM1->CNT = 0;
    TIM3->CNT = 0;
    /* Start TIM3 first (slave), then TIM1 (master starts TIM3 via trigger) */
    TIM3->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
}

extern "C" void hal_pwm_motor_set_duty(uint8_t channel, float duty) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    uint32_t ccr = (uint32_t)(duty * MOTOR_PWM_ARR);

    /* Channel mapping (from DRV8312 constructor: channels 2,1,0):
     *   channel 0 = Phase A = TIM1_CH3 (PA10)
     *   channel 1 = Phase B = TIM1_CH2 (PA9)
     *   channel 2 = Phase C = TIM1_CH1 (PA8)
     */
    switch (channel) {
    case 0: TIM1->CCR3 = ccr; break;
    case 1: TIM1->CCR2 = ccr; break;
    case 2: TIM1->CCR1 = ccr; break;
    default: break;
    }
}

extern "C" void hal_pwm_motor_enable_channel(uint8_t channel) {
    switch (channel) {
    case 0: TIM1->CCER |= TIM_CCER_CC3E; break;
    case 1: TIM1->CCER |= TIM_CCER_CC2E; break;
    case 2: TIM1->CCER |= TIM_CCER_CC1E; break;
    default: break;
    }
}

extern "C" void hal_pwm_motor_disable_channel(uint8_t channel) {
    switch (channel) {
    case 0: TIM1->CCER &= ~TIM_CCER_CC3E; break;
    case 1: TIM1->CCER &= ~TIM_CCER_CC2E; break;
    case 2: TIM1->CCER &= ~TIM_CCER_CC1E; break;
    default: break;
    }
}

/* ── LED PWM (TIM5) ──────────────────────────────── */

extern "C" void hal_pwm_led_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    /* Fully reset TIM5 to clear any bootloader state */
    TIM5->CR1 = 0;
    TIM5->CR2 = 0;
    TIM5->SMCR = 0;
    TIM5->DIER = 0;
    TIM5->SR = 0;
    TIM5->CNT = 0;

    TIM5->PSC = 0;  /* 84MHz timer clock, no prescaler */
    TIM5->ARR = LED_PWM_ARR;

    /* Force update to load PSC/ARR shadow registers */
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;  /* Clear UIF set by UG */

    /* CH1/2/3: PWM Mode 1, preload enable */
    TIM5->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE
                 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
    TIM5->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;

    /* All channels: output enable, active LOW (CCxP=1) */
    TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P
               | TIM_CCER_CC2E | TIM_CCER_CC2P
               | TIM_CCER_CC3E | TIM_CCER_CC3P;

    TIM5->CCR1 = 0;
    TIM5->CCR2 = 0;
    TIM5->CCR3 = 0;

    TIM5->CR1 = TIM_CR1_CEN;
}

extern "C" void hal_pwm_led_set(uint8_t channel, uint16_t pulse_width) {
    /* CH1=Green (PA0), CH2=Blue (PA1), CH3=Red (PA2) */
    switch (channel) {
    case 0: TIM5->CCR1 = pulse_width; break;
    case 1: TIM5->CCR2 = pulse_width; break;
    case 2: TIM5->CCR3 = pulse_width; break;
    default: break;
    }
}
