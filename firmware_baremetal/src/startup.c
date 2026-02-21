/*
 * STM32F405 vector table and Reset_Handler for baremetal firmware.
 * Placed at 0x08010000 (after bootloader).
 */

#include <stdint.h>

/* Linker-provided symbols */
extern uint32_t __main_stack_end__;
extern uint32_t _textdata;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss_start;
extern uint32_t _bss_end;
extern uint32_t __init_array_start;
extern uint32_t __init_array_end;

/* main() is in C++ */
extern int main(void);

/* Forward declarations */
void ResetHandler(void);
void Default_Handler(void);
void HardFault_Handler(void);

/* Cortex-M4 exception handlers (weak aliases to Default_Handler) */
void NMI_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__((weak, alias("Default_Handler")));

/* STM32F405 IRQ handlers (weak aliases to Default_Handler) */
void WWDG_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void TAMP_STAMP_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream3_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream4_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream5_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream6_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void ADC_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void CAN1_TX_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM9_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM10_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_TIM11_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TIM4_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void USART3_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void OTG_FS_WKUP_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void TIM8_BRK_TIM12_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void TIM8_UP_TIM13_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void DMA1_Stream7_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void FSMC_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void SDIO_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void TIM5_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void SPI3_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler(void)               __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream3_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream4_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void ETH_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));
void ETH_WKUP_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void CAN2_TX_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void CAN2_RX0_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void CAN2_RX1_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void CAN2_SCE_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void OTG_FS_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream5_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream6_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void DMA2_Stream7_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void USART6_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void)             __attribute__((weak, alias("Default_Handler")));
void OTG_HS_EP1_OUT_IRQHandler(void)      __attribute__((weak, alias("Default_Handler")));
void OTG_HS_EP1_IN_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void OTG_HS_WKUP_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void OTG_HS_IRQHandler(void)              __attribute__((weak, alias("Default_Handler")));
void DCMI_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void CRYP_IRQHandler(void)                __attribute__((weak, alias("Default_Handler")));
void HASH_RNG_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void)                 __attribute__((weak, alias("Default_Handler")));

/*
 * Vector table - placed in "vectors" section which the linker script
 * puts at the start of flash_fw (0x08010000).
 */
__attribute__((section("vectors")))
const void *vector_table[] = {
    &__main_stack_end__,            /* 0: Initial SP */
    ResetHandler,                   /* 1: Reset */
    NMI_Handler,                    /* 2: NMI */
    HardFault_Handler,              /* 3: HardFault */
    MemManage_Handler,              /* 4: MemManage */
    BusFault_Handler,               /* 5: BusFault */
    UsageFault_Handler,             /* 6: UsageFault */
    0,                              /* 7: Reserved */
    0,                              /* 8: Reserved */
    0,                              /* 9: Reserved */
    0,                              /* 10: Reserved */
    SVC_Handler,                    /* 11: SVC */
    DebugMon_Handler,               /* 12: DebugMonitor */
    0,                              /* 13: Reserved */
    PendSV_Handler,                 /* 14: PendSV */
    SysTick_Handler,                /* 15: SysTick */
    /* STM32F405 IRQs */
    WWDG_IRQHandler,                /* 0: Window Watchdog */
    PVD_IRQHandler,                 /* 1: PVD through EXTI */
    TAMP_STAMP_IRQHandler,          /* 2: Tamper/Timestamp */
    RTC_WKUP_IRQHandler,            /* 3: RTC Wakeup */
    FLASH_IRQHandler,               /* 4: Flash */
    RCC_IRQHandler,                 /* 5: RCC */
    EXTI0_IRQHandler,               /* 6: EXTI Line 0 */
    EXTI1_IRQHandler,               /* 7: EXTI Line 1 */
    EXTI2_IRQHandler,               /* 8: EXTI Line 2 */
    EXTI3_IRQHandler,               /* 9: EXTI Line 3 */
    EXTI4_IRQHandler,               /* 10: EXTI Line 4 */
    DMA1_Stream0_IRQHandler,        /* 11: DMA1 Stream 0 */
    DMA1_Stream1_IRQHandler,        /* 12: DMA1 Stream 1 */
    DMA1_Stream2_IRQHandler,        /* 13: DMA1 Stream 2 */
    DMA1_Stream3_IRQHandler,        /* 14: DMA1 Stream 3 */
    DMA1_Stream4_IRQHandler,        /* 15: DMA1 Stream 4 */
    DMA1_Stream5_IRQHandler,        /* 16: DMA1 Stream 5 */
    DMA1_Stream6_IRQHandler,        /* 17: DMA1 Stream 6 */
    ADC_IRQHandler,                 /* 18: ADC1/2/3 */
    CAN1_TX_IRQHandler,             /* 19: CAN1 TX */
    CAN1_RX0_IRQHandler,            /* 20: CAN1 RX0 */
    CAN1_RX1_IRQHandler,            /* 21: CAN1 RX1 */
    CAN1_SCE_IRQHandler,            /* 22: CAN1 SCE */
    EXTI9_5_IRQHandler,             /* 23: EXTI Lines 5-9 */
    TIM1_BRK_TIM9_IRQHandler,       /* 24: TIM1 Break / TIM9 */
    TIM1_UP_TIM10_IRQHandler,       /* 25: TIM1 Update / TIM10 */
    TIM1_TRG_COM_TIM11_IRQHandler,  /* 26: TIM1 Trig+Com / TIM11 */
    TIM1_CC_IRQHandler,             /* 27: TIM1 Capture Compare */
    TIM2_IRQHandler,                /* 28: TIM2 */
    TIM3_IRQHandler,                /* 29: TIM3 */
    TIM4_IRQHandler,                /* 30: TIM4 */
    I2C1_EV_IRQHandler,             /* 31: I2C1 Event */
    I2C1_ER_IRQHandler,             /* 32: I2C1 Error */
    I2C2_EV_IRQHandler,             /* 33: I2C2 Event */
    I2C2_ER_IRQHandler,             /* 34: I2C2 Error */
    SPI1_IRQHandler,                /* 35: SPI1 */
    SPI2_IRQHandler,                /* 36: SPI2 */
    USART1_IRQHandler,              /* 37: USART1 */
    USART2_IRQHandler,              /* 38: USART2 */
    USART3_IRQHandler,              /* 39: USART3 */
    EXTI15_10_IRQHandler,           /* 40: EXTI Lines 10-15 */
    RTC_Alarm_IRQHandler,           /* 41: RTC Alarm A/B */
    OTG_FS_WKUP_IRQHandler,         /* 42: USB OTG FS Wakeup */
    TIM8_BRK_TIM12_IRQHandler,      /* 43: TIM8 Break / TIM12 */
    TIM8_UP_TIM13_IRQHandler,       /* 44: TIM8 Update / TIM13 */
    TIM8_TRG_COM_TIM14_IRQHandler,  /* 45: TIM8 Trig+Com / TIM14 */
    TIM8_CC_IRQHandler,             /* 46: TIM8 Capture Compare */
    DMA1_Stream7_IRQHandler,        /* 47: DMA1 Stream 7 */
    FSMC_IRQHandler,                /* 48: FSMC */
    SDIO_IRQHandler,                /* 49: SDIO */
    TIM5_IRQHandler,                /* 50: TIM5 */
    SPI3_IRQHandler,                /* 51: SPI3 */
    UART4_IRQHandler,               /* 52: UART4 */
    UART5_IRQHandler,               /* 53: UART5 */
    TIM6_DAC_IRQHandler,            /* 54: TIM6 / DAC */
    TIM7_IRQHandler,                /* 55: TIM7 */
    DMA2_Stream0_IRQHandler,        /* 56: DMA2 Stream 0 */
    DMA2_Stream1_IRQHandler,        /* 57: DMA2 Stream 1 */
    DMA2_Stream2_IRQHandler,        /* 58: DMA2 Stream 2 */
    DMA2_Stream3_IRQHandler,        /* 59: DMA2 Stream 3 */
    DMA2_Stream4_IRQHandler,        /* 60: DMA2 Stream 4 */
    ETH_IRQHandler,                 /* 61: Ethernet */
    ETH_WKUP_IRQHandler,            /* 62: Ethernet Wakeup */
    CAN2_TX_IRQHandler,             /* 63: CAN2 TX */
    CAN2_RX0_IRQHandler,            /* 64: CAN2 RX0 */
    CAN2_RX1_IRQHandler,            /* 65: CAN2 RX1 */
    CAN2_SCE_IRQHandler,            /* 66: CAN2 SCE */
    OTG_FS_IRQHandler,              /* 67: USB OTG FS */
    DMA2_Stream5_IRQHandler,        /* 68: DMA2 Stream 5 */
    DMA2_Stream6_IRQHandler,        /* 69: DMA2 Stream 6 */
    DMA2_Stream7_IRQHandler,        /* 70: DMA2 Stream 7 */
    USART6_IRQHandler,              /* 71: USART6 */
    I2C3_EV_IRQHandler,             /* 72: I2C3 Event */
    I2C3_ER_IRQHandler,             /* 73: I2C3 Error */
    OTG_HS_EP1_OUT_IRQHandler,      /* 74: USB OTG HS EP1 Out */
    OTG_HS_EP1_IN_IRQHandler,       /* 75: USB OTG HS EP1 In */
    OTG_HS_WKUP_IRQHandler,         /* 76: USB OTG HS Wakeup */
    OTG_HS_IRQHandler,              /* 77: USB OTG HS */
    DCMI_IRQHandler,                /* 78: DCMI */
    CRYP_IRQHandler,                /* 79: CRYP */
    HASH_RNG_IRQHandler,            /* 80: Hash / RNG */
    FPU_IRQHandler,                 /* 81: FPU */
};

__attribute__((naked))
void ResetHandler(void) {
    /* Enable FPU: set CP10 and CP11 full access */
    __asm volatile (
        "ldr r0, =0xE000ED88\n"
        "ldr r1, [r0]\n"
        "orr r1, r1, #(0xF << 20)\n"
        "str r1, [r0]\n"
        "dsb\n"
        "isb\n"
    );

    /* Copy .data from flash to RAM */
    {
        uint32_t *src = &_textdata;
        uint32_t *dst = &_data;
        while (dst < &_edata) {
            *dst++ = *src++;
        }
    }

    /* Zero .bss */
    {
        uint32_t *dst = &_bss_start;
        while (dst < &_bss_end) {
            *dst++ = 0;
        }
    }

    /* Call C++ static constructors */
    {
        typedef void (*init_func_t)(void);
        init_func_t *fn = (init_func_t *)&__init_array_start;
        while (fn < (init_func_t *)&__init_array_end) {
            (*fn)();
            fn++;
        }
    }

    main();

    /* Should never return */
    while (1) {}
}

void Default_Handler(void) {
    while (1) {}
}

void HardFault_Handler(void) {
    /* Jump to firmware at 0x08010000 (reset) */
    typedef void (*reset_fn)(void);
    uint32_t *vectors = (uint32_t *)0x08010000;
    /* vectors[0] is initial SP, vectors[1] is reset handler */
    __asm volatile ("msr MSP, %0" : : "r" (vectors[0]));
    reset_fn fn = (reset_fn)vectors[1];
    fn();
}
