/*
 * STM32F405 USART1 + DMA2 HAL implementation.
 *
 * RX: DMA2 Stream5 Channel4, circular mode.
 *     Software tracks position via NDTR to provide circular buffer semantics.
 * TX: DMA2 Stream7 Channel4, normal mode.
 *     TX complete ISR clears RS485 DIR pin (PD2).
 *
 * RS485 half-duplex: DIR pin HIGH = transmit, LOW = receive.
 */

#include "hal/hal_uart.h"

#include "baremetal_config.h"

#ifdef STM32F405xx

#include "stm32f4xx.h"

/* ── Static state ──────────────────────────────────────────── */

static uint8_t rx_dma_buf[UART_RX_BUF_SIZE] __attribute__((aligned(4)));
static volatile size_t rx_read_pos;  /* software read pointer */

/* ── Helpers ───────────────────────────────────────────────── */

static inline size_t dma_write_pos(void) {
    return UART_RX_BUF_SIZE - USART1_RX_DMA_STREAM->NDTR;
}

/* ── Init ──────────────────────────────────────────────────── */

void hal_uart_init(uint32_t baud) {
    /* Enable clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Configure USART1: 8N1, TX+RX enabled */
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;  /* DMA for TX and RX */
    USART1->BRR = APB2_HZ / baud;                     /* 84MHz / 1MHz = 84 */

    /* ── RX DMA: DMA2 Stream5 Channel4 circular ─────────── */
    USART1_RX_DMA_STREAM->CR = 0;
    while (USART1_RX_DMA_STREAM->CR & DMA_SxCR_EN) {}  /* Wait disable */

    /* Clear all interrupt flags for Stream5 (HIFCR bits 6..11) */
    DMA2->HIFCR = 0x0F40;

    USART1_RX_DMA_STREAM->PAR = (uint32_t)&USART1->DR;
    USART1_RX_DMA_STREAM->M0AR = (uint32_t)rx_dma_buf;
    USART1_RX_DMA_STREAM->NDTR = UART_RX_BUF_SIZE;
    USART1_RX_DMA_STREAM->CR =
        (USART1_RX_DMA_CHANNEL << 25) |  /* Channel 4 */
        DMA_SxCR_MINC |                                    /* Memory increment */
        DMA_SxCR_CIRC |                                    /* Circular mode */
        DMA_SxCR_EN;                                       /* Enable */

    /* ── TX DMA: DMA2 Stream7 Channel4 normal (configured per transfer) ── */
    USART1_TX_DMA_STREAM->CR = 0;
    while (USART1_TX_DMA_STREAM->CR & DMA_SxCR_EN) {}

    /* Clear all interrupt flags for Stream7 (HIFCR bits 22..27) */
    DMA2->HIFCR = 0x0F400000;

    USART1_TX_DMA_STREAM->PAR = (uint32_t)&USART1->DR;

    /* Enable TX complete interrupt at NVIC level */
    NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_PRIO_DMA_USART1_TX);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    /* Enable USART */
    rx_read_pos = 0;
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* Start in receive mode (DIR low) */
    hal_uart_set_tx_mode(false);
}

/* ── RX operations ─────────────────────────────────────────── */

size_t hal_uart_rx_available(void) {
    size_t wp = dma_write_pos();
    return (wp + UART_RX_BUF_SIZE - rx_read_pos) % UART_RX_BUF_SIZE;
}

size_t hal_uart_rx_read(uint8_t *buf, size_t max_len) {
    size_t avail = hal_uart_rx_available();
    size_t n = (avail < max_len) ? avail : max_len;
    for (size_t i = 0; i < n; i++) {
        buf[i] = rx_dma_buf[rx_read_pos];
        rx_read_pos = (rx_read_pos + 1) % UART_RX_BUF_SIZE;
    }
    return n;
}

bool hal_uart_rx_peek(uint8_t *out) {
    if (hal_uart_rx_available() == 0) return false;
    *out = rx_dma_buf[rx_read_pos];
    return true;
}

void hal_uart_rx_consume(size_t count) {
    size_t avail = hal_uart_rx_available();
    if (count > avail) count = avail;
    rx_read_pos = (rx_read_pos + count) % UART_RX_BUF_SIZE;
}

/* ── TX operations ─────────────────────────────────────────── */

bool hal_uart_tx_send(const uint8_t *data, size_t len) {
    if (len == 0) return true;

    /* Wait for any previous TX DMA to finish */
    if (USART1_TX_DMA_STREAM->CR & DMA_SxCR_EN) {
        return false;  /* Busy */
    }

    /* Clear Stream7 interrupt flags */
    DMA2->HIFCR = 0x0F400000;

    USART1_TX_DMA_STREAM->M0AR = (uint32_t)data;
    USART1_TX_DMA_STREAM->NDTR = len;
    USART1_TX_DMA_STREAM->CR =
        (USART1_TX_DMA_CHANNEL << 25) |
        DMA_SxCR_MINC |                  /* Memory increment */
        (0x1 << 6) |      /* Memory to peripheral */
        DMA_SxCR_TCIE |                   /* Transfer complete interrupt */
        DMA_SxCR_EN;

    return true;
}

bool hal_uart_tx_busy(void) {
    return (USART1_TX_DMA_STREAM->CR & DMA_SxCR_EN) != 0;
}

void hal_uart_tx_wait_complete(void) {
    /* Wait for DMA transfer to finish */
    while (USART1_TX_DMA_STREAM->CR & DMA_SxCR_EN) {}

    /* Wait for USART to shift out the last byte */
    while (!(USART1->SR & USART_SR_TC)) {}
    USART1->SR &= ~USART_SR_TC;
}

/* ── RS485 direction ───────────────────────────────────────── */

void hal_uart_set_tx_mode(bool transmit) {
    if (transmit) {
        /* DIR high = transmit */
        ((GPIO_TypeDef *)RS485_DIR_PORT)->BSRR = (1U << RS485_DIR_PIN);
    } else {
        /* DIR low = receive */
        ((GPIO_TypeDef *)RS485_DIR_PORT)->BSRR = (1U << (RS485_DIR_PIN + 16));
    }
}

/* ── TX DMA complete ISR ───────────────────────────────────── */

extern "C" void DMA2_Stream7_IRQHandler(void) {
    /* Clear all Stream7 interrupt flags.
     * DIR pin switching is handled by the caller via
     * hal_uart_tx_wait_complete() + hal_uart_set_tx_mode(). */
    DMA2->HIFCR = 0x0F400000;
}

#endif /* STM32F405xx */
