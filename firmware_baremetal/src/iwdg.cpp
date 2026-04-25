/*
 * Independent Watchdog (IWDG) HAL implementation for STM32F405.
 * Prescaler /4, reload 80 = ~10ms timeout at 32kHz LSI.
 */

#include "stm32f4xx.h"
#include "hal/hal_iwdg.h"

/* IWDG key values */
#define IWDG_KEY_ENABLE   0xCCCCU
#define IWDG_KEY_ACCESS   0x5555U
#define IWDG_KEY_RELOAD   0xAAAAU

/* Saved values for pause/resume */
static uint32_t saved_pr;
static uint32_t saved_rlr;

extern "C" void hal_iwdg_init(void) {
    /* Start IWDG */
    IWDG->KR = IWDG_KEY_ENABLE;

    /* Enable register access */
    IWDG->KR = IWDG_KEY_ACCESS;

    /* Prescaler /4 (PR = 0) */
    IWDG->PR = 0;

    /* Reload value 80 (~10ms at 32kHz/4 = 8kHz) */
    IWDG->RLR = 80;

    /* Wait for registers to update */
    while (IWDG->SR != 0) {}

    /* Reload counter */
    IWDG->KR = IWDG_KEY_RELOAD;
}

extern "C" void hal_iwdg_kick(void) {
    IWDG->KR = IWDG_KEY_RELOAD;
}

extern "C" void hal_iwdg_pause(void) {
    /* Save current prescaler and reload values, then set max timeout */
    saved_pr = IWDG->PR;
    saved_rlr = IWDG->RLR;

    IWDG->KR = IWDG_KEY_ACCESS;
    IWDG->PR = 6;        /* Prescaler /256 */
    IWDG->RLR = 0xFFF;   /* Max reload = ~32 seconds */
    while (IWDG->SR != 0) {}
    IWDG->KR = IWDG_KEY_RELOAD;
}

extern "C" void hal_iwdg_resume(void) {
    /* Restore saved prescaler and reload values */
    IWDG->KR = IWDG_KEY_ACCESS;
    IWDG->PR = saved_pr;
    IWDG->RLR = saved_rlr;
    while (IWDG->SR != 0) {}
    IWDG->KR = IWDG_KEY_RELOAD;
}

extern "C" void hal_iwdg_clear_reset_flag(void) {
    RCC->CSR |= RCC_CSR_RMVF;
}
