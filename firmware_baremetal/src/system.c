/*
 * System clock initialization and timer HAL for STM32F405.
 * HSE 8MHz -> PLL -> 168MHz SYSCLK.
 */

#include "stm32f4xx.h"
#include "baremetal_config.h"
#include "hal/hal_timer.h"

static volatile uint32_t systick_ms;

void SysTick_Handler(void) {
    systick_ms++;
}

void system_init(void) {
    /* Set vector table to firmware location (after bootloader) */
    SCB->VTOR = FLASH_FW_ADDR;

    /* Clear all NVIC interrupt enables and pending flags inherited from
     * the bootloader.  Without this, bootloader-enabled IRQs (USART1,
     * DMA, etc.) can fire before the firmware sets up its own handlers,
     * sending the CPU into Default_Handler's while(1) loop. */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* The bootloader leaves PLL running as SYSCLK.  We must switch away
     * from PLL before reconfiguring it (RM0090 §7.3.2: PLL parameters
     * cannot be changed while PLL is enabled). */

    /* Enable HSI (in case bootloader disabled it) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    /* Switch SYSCLK to HSI */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}

    /* Now safe to disable PLL */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}

    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {}

    /* Configure PLL: HSE/4 * 168 / 2 = 168MHz (safe now that PLL is off) */
    RCC->PLLCFGR = (PLL_M << 0)
                  | (PLL_N << 6)
                  | (((PLL_P / 2) - 1) << 16)
                  | RCC_PLLCFGR_PLLSRC_HSE
                  | (PLL_Q << 24);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* Flash: 5 wait states, prefetch, instruction cache, data cache */
    FLASH->ACR = FLASH_ACR_LATENCY_5WS
               | FLASH_ACR_PRFTEN
               | FLASH_ACR_ICEN
               | FLASH_ACR_DCEN;

    /* Bus prescalers: AHB /1, APB1 /4, APB2 /2 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV4
              | RCC_CFGR_PPRE2_DIV2
              | RCC_CFGR_SW_PLL;

    /* Wait for PLL as system clock */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

    /* Enable GPIO clocks (A, B, C, D, E) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN
                  | RCC_AHB1ENR_GPIODEN
                  | RCC_AHB1ENR_GPIOEEN;

    /* Enable DMA clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

    /* Enable APB2 peripherals: TIM1, USART1, ADC1/2/3 */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN
                  | RCC_APB2ENR_USART1EN
                  | RCC_APB2ENR_ADC1EN
                  | RCC_APB2ENR_ADC2EN
                  | RCC_APB2ENR_ADC3EN;

    /* Enable APB1 peripherals: TIM3, TIM5, SPI3, I2C2 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
                  | RCC_APB1ENR_TIM5EN
                  | RCC_APB1ENR_SPI3EN
                  | RCC_APB1ENR_I2C2EN;

    /* NVIC priority grouping: 4 bits group, 0 bits sub */
    NVIC_SetPriorityGrouping(0);

    /* SysTick: 1ms interrupts */
    SysTick_Config(SYSCLK_HZ / 1000);

    /* Enable DWT cycle counter for microsecond timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* Re-enable interrupts.  The bootloader disables them (PRIMASK=1)
     * before jumping to firmware.  Without this, SysTick and all other
     * ISRs are permanently blocked. */
    __enable_irq();
}

/* ── hal_timer implementation ──────────────────────────────────── */

uint32_t hal_timer_msec(void) {
    return systick_ms;
}

uint32_t hal_timer_usec(void) {
    uint32_t ms = systick_ms;
    uint32_t ticks = SysTick->LOAD - SysTick->VAL;
    return ms * 1000 + ticks / (SYSCLK_HZ / 1000000);
}

void hal_timer_delay_ms(uint32_t ms) {
    uint32_t start = systick_ms;
    while ((systick_ms - start) < ms) {}
}

void hal_timer_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SYSCLK_HZ / 1000000);
    while ((DWT->CYCCNT - start) < cycles) {}
}
