/*
 * STM32F405 USART1 + DMA2 implementation using ST HAL.
 *
 * RX: DMA2 Stream5 Channel4, circular mode.
 *     Software tracks position via NDTR for circular buffer semantics.
 * TX: DMA2 Stream7 Channel4, normal mode.
 *     DMA complete callback updates HAL state.
 *
 * RS485 half-duplex: DIR pin HIGH = transmit, LOW = receive.
 */

#include "hal/hal_uart.h"

#include <cstring>

#include "baremetal_config.h"

#ifdef STM32F405xx

#include "stm32f4xx_hal.h"

/* ── Static state ──────────────────────────────────────────── */

static UART_HandleTypeDef huart1;
static DMA_HandleTypeDef hdma_rx;
static DMA_HandleTypeDef hdma_tx;

static uint8_t rx_dma_buf[UART_RX_BUF_SIZE] __attribute__((aligned(4)));
static volatile size_t rx_read_pos;

static uint8_t tx_dma_buf[UART_TX_BUF_SIZE] __attribute__((aligned(4)));

/* ── Helpers ───────────────────────────────────────────────── */

static inline size_t dma_write_pos(void) {
    return UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_rx);
}

/* ── HAL MSP callback (called by HAL_UART_Init) ───────────── */

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;

    /* Clocks already enabled by system_init() */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* RX DMA: DMA2 Stream5 Channel4, circular */
    hdma_rx.Instance = DMA2_Stream5;
    hdma_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_CIRCULAR;
    hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_rx);
    __HAL_LINKDMA(huart, hdmarx, hdma_rx);

    /* TX DMA: DMA2 Stream7 Channel4, normal */
    hdma_tx.Instance = DMA2_Stream7;
    hdma_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma_tx);
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);

    /* Enable DMA interrupts at NVIC level */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_PRIO_DMA_USART1_RX, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_PRIO_DMA_USART1_TX, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    /* Enable USART1 interrupt for TX complete (TCIE).
     * After DMA TX finishes, the HAL enables TCIE and expects
     * USART1_IRQHandler to finalize gState back to READY. */
    HAL_NVIC_SetPriority(USART1_IRQn, NVIC_PRIO_USART1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ── Init ──────────────────────────────────────────────────── */

void hal_uart_init(uint32_t baud) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baud;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    /* Clear any error flags inherited from bootloader.
     * In DMA mode, error flags can block DMA RXNE requests. */
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    /* Start circular RX DMA */
    rx_read_pos = 0;
    HAL_UART_Receive_DMA(&huart1, rx_dma_buf, UART_RX_BUF_SIZE);

    /* Disable USART error interrupts to prevent ORE from aborting circular DMA.
     * HAL_UART_Receive_DMA enables EIE+PEIE, but HAL_UART_IRQHandler would
     * abort RX DMA on any error in DMA mode. */
    CLEAR_BIT(huart1.Instance->CR1, USART_CR1_PEIE);
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);

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
    if (len > UART_TX_BUF_SIZE) return false;
    if (huart1.gState != HAL_UART_STATE_READY) return false;

    memcpy(tx_dma_buf, data, len);
    return HAL_UART_Transmit_DMA(&huart1, tx_dma_buf, (uint16_t)len) == HAL_OK;
}

bool hal_uart_tx_busy(void) {
    return huart1.gState != HAL_UART_STATE_READY;
}

void hal_uart_tx_wait_complete(void) {
    /* Wait for DMA transfer to finish */
    while (huart1.gState != HAL_UART_STATE_READY) {}

    /* Wait for USART to shift out the last byte */
    while (!(USART1->SR & USART_SR_TC)) {}
    USART1->SR &= ~USART_SR_TC;
}

/* ── RS485 direction ───────────────────────────────────────── */

void hal_uart_set_tx_mode(bool transmit) {
    if (transmit) {
        ((GPIO_TypeDef *)RS485_DIR_PORT)->BSRR = (1U << RS485_DIR_PIN);
    } else {
        ((GPIO_TypeDef *)RS485_DIR_PORT)->BSRR = (1U << (RS485_DIR_PIN + 16));
    }
}

/* ── Debug ─────────────────────────────────────────────────── */

void hal_uart_rx_debug(uint32_t *write_pos, uint32_t *read_pos,
                       uint32_t *ndtr_val, uint32_t *buf_size) {
    *write_pos = (uint32_t)dma_write_pos();
    *read_pos = (uint32_t)rx_read_pos;
    *ndtr_val = (uint32_t)__HAL_DMA_GET_COUNTER(&hdma_rx);
    *buf_size = UART_RX_BUF_SIZE;
}

/* ── DMA ISRs ──────────────────────────────────────────────── */

extern "C" void DMA2_Stream5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_rx);
}

extern "C" void DMA2_Stream7_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tx);
}

/* ── USART1 ISR (TX complete) ─────────────────────────────── */

extern "C" void USART1_IRQHandler(void) {
    /* Call HAL handler for TX complete (TCIE).
     * Safe because EIE+PEIE are disabled — the HAL checks those bits
     * before entering the error/abort path, so it won't abort RX DMA. */
    HAL_UART_IRQHandler(&huart1);
}

#endif /* STM32F405xx */
