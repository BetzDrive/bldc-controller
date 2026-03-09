/*
 * ADC HAL implementation for STM32F405.
 * Triple ADC with DMA, triggered by TIM3 TRGO.
 *
 * ADC1 (DMA2 Stream4 Ch0): CH12 (ISENSE_A/PC2) + CH13 (VSENSE_VIN/PC3)
 * ADC2 (DMA2 Stream2 Ch1): CH11 (ISENSE_B/PC1)
 * ADC3 (DMA2 Stream1 Ch2): CH10 (ISENSE_C/PC0)
 */

#include "hal/hal_adc.h"
#include "baremetal_config.h"
#include "stm32f4xx.h"

/* DMA circular buffers (volatile - written by DMA hardware) */
static volatile uint16_t adc1_buf[4];  /* 2 channels x 2 (double-buffered) */
static volatile uint16_t adc2_buf[2];  /* 1 channel x 2 */
static volatile uint16_t adc3_buf[2];  /* 1 channel x 2 */

/* ADC external trigger: TIM3 TRGO = EXTSEL 1000 (RM0090 Table 68) */
#define ADC_EXTSEL_TIM3_TRGO   (8U << 24)
#define ADC_EXTEN_RISING       (1U << 28)

/* DMA flag clear masks (RM0090 Section 10.5.5/10.5.6) */
#define DMA_LIFCR_STREAM1_ALL  0x00000F40U
#define DMA_LIFCR_STREAM2_ALL  0x003D0000U
#define DMA_HIFCR_STREAM4_ALL  0x0000003DU

extern "C" void hal_adc_init(void) {
    /* Enable clocks */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    /* ADC Common: ADCPRE = 01 (/4) -> 84MHz/4 = 21MHz ADC clock */
    ADC->CCR = (1U << 16);

    /* ── ADC1: CH12 (ISENSE_A) + CH13 (VSENSE_VIN), 2 conversions ── */
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_DMA | ADC_CR2_DDS
              | ADC_EXTSEL_TIM3_TRGO | ADC_EXTEN_RISING;
    ADC1->SQR1 = (1U << 20);                    /* L=1 (2 conversions) */
    ADC1->SQR3 = (12U << 0) | (13U << 5);       /* SQ1=CH12, SQ2=CH13 */
    ADC1->SMPR1 = (1U << 6) | (1U << 9);        /* SMP12=001(15cyc), SMP13=001 */

    /* DMA2 Stream4 Ch0 for ADC1 */
    DMA2_Stream4->CR = 0;
    while (DMA2_Stream4->CR & DMA_SxCR_EN) {}
    DMA2->HIFCR = DMA_HIFCR_STREAM4_ALL;
    DMA2_Stream4->PAR = (uint32_t)&ADC1->DR;
    DMA2_Stream4->M0AR = (uint32_t)adc1_buf;
    DMA2_Stream4->NDTR = 4;
    DMA2_Stream4->CR = (0U << 25)               /* CHSEL=0 */
                     | DMA_SxCR_MSIZE_0          /* 16-bit memory */
                     | DMA_SxCR_PSIZE_0          /* 16-bit peripheral */
                     | DMA_SxCR_MINC             /* Memory increment */
                     | DMA_SxCR_CIRC             /* Circular mode */
                     | (2U << 16);               /* Priority high */
    DMA2_Stream4->CR |= DMA_SxCR_EN;

    /* ── ADC2: CH11 (ISENSE_B), 1 conversion ── */
    ADC2->CR1 = 0;
    ADC2->CR2 = ADC_CR2_DMA | ADC_CR2_DDS
              | ADC_EXTSEL_TIM3_TRGO | ADC_EXTEN_RISING;
    ADC2->SQR1 = 0;                             /* L=0 (1 conversion) */
    ADC2->SQR3 = 11U;                           /* SQ1=CH11 */
    ADC2->SMPR1 = (1U << 3);                    /* SMP11=001(15cyc) */

    /* DMA2 Stream2 Ch1 for ADC2 */
    DMA2_Stream2->CR = 0;
    while (DMA2_Stream2->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR = DMA_LIFCR_STREAM2_ALL;
    DMA2_Stream2->PAR = (uint32_t)&ADC2->DR;
    DMA2_Stream2->M0AR = (uint32_t)adc2_buf;
    DMA2_Stream2->NDTR = 2;
    DMA2_Stream2->CR = (1U << 25)               /* CHSEL=1 */
                     | DMA_SxCR_MSIZE_0
                     | DMA_SxCR_PSIZE_0
                     | DMA_SxCR_MINC
                     | DMA_SxCR_CIRC
                     | (2U << 16);
    DMA2_Stream2->CR |= DMA_SxCR_EN;

    /* ── ADC3: CH10 (ISENSE_C), 1 conversion ── */
    ADC3->CR1 = 0;
    ADC3->CR2 = ADC_CR2_DMA | ADC_CR2_DDS
              | ADC_EXTSEL_TIM3_TRGO | ADC_EXTEN_RISING;
    ADC3->SQR1 = 0;
    ADC3->SQR3 = 10U;                           /* SQ1=CH10 */
    ADC3->SMPR1 = (1U << 0);                    /* SMP10=001(15cyc) */

    /* DMA2 Stream1 Ch2 for ADC3 */
    DMA2_Stream1->CR = 0;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR = DMA_LIFCR_STREAM1_ALL;
    DMA2_Stream1->PAR = (uint32_t)&ADC3->DR;
    DMA2_Stream1->M0AR = (uint32_t)adc3_buf;
    DMA2_Stream1->NDTR = 2;
    DMA2_Stream1->CR = (2U << 25)               /* CHSEL=2 */
                     | DMA_SxCR_MSIZE_0
                     | DMA_SxCR_PSIZE_0
                     | DMA_SxCR_MINC
                     | DMA_SxCR_CIRC
                     | (2U << 16);
    DMA2_Stream1->CR |= DMA_SxCR_EN;
}

extern "C" void hal_adc_start(void) {
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC2->CR2 |= ADC_CR2_ADON;
    ADC3->CR2 |= ADC_CR2_ADON;
}

extern "C" struct AdcSamples hal_adc_get_latest(void) {
    struct AdcSamples s;

    /*
     * Each ADC uses circular DMA double-buffering. 16-bit reads are
     * atomic on Cortex-M4, and the rolling average in the control loop
     * smooths any timing jitter. Read directly from buffer position 0.
     */
    s.ia   = adc1_buf[0];  /* CH12 = ISENSE_A */
    s.vbus = adc1_buf[1];  /* CH13 = VSENSE_VIN */
    s.ib   = adc2_buf[0];  /* CH11 = ISENSE_B */
    s.ic   = adc3_buf[0];  /* CH10 = ISENSE_C */

    return s;
}
