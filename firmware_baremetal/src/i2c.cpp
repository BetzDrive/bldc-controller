/*
 * I2C2 HAL implementation for STM32F405.
 * Non-blocking IRQ+DMA driver. GPIO configured in gpio.cpp.
 *
 * State machine driven by I2C2 event/error interrupts:
 *   IDLE -> START_SENT -> ADDR_W_SENT -> TX_DATA ->
 *     (write-only: STOP -> COMPLETE)
 *     (write-read: RESTART_SENT -> ADDR_R_SENT -> RX_DMA -> COMPLETE)
 */

#include "stm32f4xx.h"
#include "baremetal_config.h"
#include "hal/hal_i2c.h"

/* ── Transfer state machine ─────────────────────────────────────── */

enum I2CState {
    ST_IDLE = 0,
    ST_START_SENT,     /* Waiting for SB after START */
    ST_ADDR_W_SENT,    /* Waiting for ADDR after write-address sent */
    ST_TX_DATA,        /* Transmitting bytes via TXE */
    ST_RESTART_SENT,   /* Waiting for SB after repeated START */
    ST_ADDR_R_SENT,    /* Waiting for ADDR after read-address sent */
    ST_RX_DMA,         /* DMA receiving data */
    ST_COMPLETE,       /* Transfer done */
    ST_ERROR,          /* Transfer failed */
};

static volatile enum I2CState i2c_state;
static volatile hal_i2c_status_t i2c_error;

/* Transfer parameters (set by start functions, used by ISR) */
static uint8_t xfer_addr;
static const uint8_t *xfer_tx_buf;
static volatile size_t xfer_tx_len;
static volatile size_t xfer_tx_idx;
static uint8_t *xfer_rx_buf;
static volatile size_t xfer_rx_len;
static volatile bool xfer_has_rx;  /* true = write_read, false = write-only */

/* ── Helpers ────────────────────────────────────────────────────── */

static void i2c_stop_with_error(hal_i2c_status_t err) {
    I2C2->CR1 |= I2C_CR1_STOP;
    /* Disable all I2C interrupts */
    I2C2->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN |
                    I2C_CR2_DMAEN | I2C_CR2_LAST);
    /* Disable DMA streams */
    I2C2_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    I2C2_TX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    i2c_error = err;
    i2c_state = ST_ERROR;
}

static void setup_rx_dma(void) {
    /* Disable stream before configuring */
    I2C2_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (I2C2_RX_DMA_STREAM->CR & DMA_SxCR_EN) {}

    /* Clear DMA1 Stream2 flags (bits 21:16 in LIFCR) */
    DMA1->LIFCR = (0x3FU << 16);

    I2C2_RX_DMA_STREAM->PAR  = (uint32_t)&I2C2->DR;
    I2C2_RX_DMA_STREAM->M0AR = (uint32_t)xfer_rx_buf;
    I2C2_RX_DMA_STREAM->NDTR = (uint16_t)xfer_rx_len;
    I2C2_RX_DMA_STREAM->CR   =
        (I2C2_RX_DMA_CHANNEL << 25) |   /* Channel 7 */
        DMA_SxCR_MINC |                 /* Memory increment */
        DMA_SxCR_TCIE;                  /* Transfer complete IRQ */
    /* Periph-to-memory (DIR=00), no FIFO, 8-bit */

    I2C2_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
}

/* ── Public API ─────────────────────────────────────────────────── */

static void i2c_bus_recovery(void) {
    /*
     * If a slave was interrupted mid-transfer (e.g. by reset), SDA may
     * be held low.  Toggle SCL 9 times as GPIO to clock out the stuck
     * byte, then generate a STOP condition to release the bus.
     */
    GPIO_TypeDef *scl_gpio = (GPIO_TypeDef *)I2C_SCL_PORT;
    GPIO_TypeDef *sda_gpio = (GPIO_TypeDef *)I2C_SDA_PORT;

    /* Temporarily switch SCL to GPIO output open-drain */
    uint32_t scl_moder_save = scl_gpio->MODER;
    scl_gpio->MODER = (scl_gpio->MODER & ~(3U << (I2C_SCL_PIN * 2)))
                    | (1U << (I2C_SCL_PIN * 2));  /* Output mode */

    for (int i = 0; i < 9; i++) {
        scl_gpio->BSRR = (1U << (I2C_SCL_PIN + 16));  /* SCL low */
        for (volatile int d = 0; d < 100; d++) {}
        scl_gpio->BSRR = (1U << I2C_SCL_PIN);          /* SCL high */
        for (volatile int d = 0; d < 100; d++) {}
        /* Check if SDA released */
        if (sda_gpio->IDR & (1U << I2C_SDA_PIN)) break;
    }

    /* Generate STOP: SDA low then high while SCL is high */
    /* Switch SDA to GPIO output open-drain temporarily */
    uint32_t sda_moder_save = sda_gpio->MODER;
    sda_gpio->MODER = (sda_gpio->MODER & ~(3U << (I2C_SDA_PIN * 2)))
                    | (1U << (I2C_SDA_PIN * 2));
    sda_gpio->BSRR = (1U << (I2C_SDA_PIN + 16));       /* SDA low */
    for (volatile int d = 0; d < 100; d++) {}
    sda_gpio->BSRR = (1U << I2C_SDA_PIN);               /* SDA high (STOP) */
    for (volatile int d = 0; d < 100; d++) {}

    /* Restore AF mode */
    scl_gpio->MODER = scl_moder_save;
    sda_gpio->MODER = sda_moder_save;
}

extern "C" void hal_i2c_init(void) {
    /* Enable clocks */
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    /* Release any stuck I2C slave before init */
    i2c_bus_recovery();

    /* Reset I2C2 */
    I2C2->CR1 = I2C_CR1_SWRST;
    I2C2->CR1 = 0;

    /* APB1 = 42MHz */
    I2C2->CR2 = (APB1_HZ / 1000000U);

    /* Standard mode 100kHz: CCR = APB1 / (2 * 100kHz) = 210 */
    I2C2->CCR = APB1_HZ / (2 * 100000U);

    /* TRISE = (APB1 / 1MHz) + 1 = 43 */
    I2C2->TRISE = (APB1_HZ / 1000000U) + 1;

    /* Enable I2C */
    I2C2->CR1 = I2C_CR1_PE;

    i2c_state = ST_IDLE;
    i2c_error = HAL_I2C_OK;

    /* Enable NVIC for I2C2 event and error interrupts */
    NVIC_SetPriority(I2C2_EV_IRQn, NVIC_PRIO_I2C2);
    NVIC_SetPriority(I2C2_ER_IRQn, NVIC_PRIO_I2C2);
    NVIC_EnableIRQ(I2C2_EV_IRQn);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

    /* Enable NVIC for DMA1 Stream2 (I2C2 RX) */
    NVIC_SetPriority(DMA1_Stream2_IRQn, NVIC_PRIO_I2C2);
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

extern "C" bool hal_i2c_start_write_read(uint8_t addr,
                                          const uint8_t *tx_buf, size_t tx_len,
                                          uint8_t *rx_buf, size_t rx_len) {
    if (i2c_state != ST_IDLE && i2c_state != ST_COMPLETE &&
        i2c_state != ST_ERROR) {
        return false;
    }

    xfer_addr   = addr;
    xfer_tx_buf = tx_buf;
    xfer_tx_len = tx_len;
    xfer_tx_idx = 0;
    xfer_rx_buf = rx_buf;
    xfer_rx_len = rx_len;
    xfer_has_rx = true;
    i2c_error   = HAL_I2C_OK;
    i2c_state   = ST_START_SENT;

    /* Enable event + error interrupts, generate START */
    I2C2->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    I2C2->CR1 |= I2C_CR1_START;

    return true;
}

extern "C" bool hal_i2c_start_write(uint8_t addr,
                                     const uint8_t *tx_buf, size_t tx_len) {
    if (i2c_state != ST_IDLE && i2c_state != ST_COMPLETE &&
        i2c_state != ST_ERROR) {
        return false;
    }

    xfer_addr   = addr;
    xfer_tx_buf = tx_buf;
    xfer_tx_len = tx_len;
    xfer_tx_idx = 0;
    xfer_rx_buf = NULL;
    xfer_rx_len = 0;
    xfer_has_rx = false;
    i2c_error   = HAL_I2C_OK;
    i2c_state   = ST_START_SENT;

    I2C2->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
    I2C2->CR1 |= I2C_CR1_START;

    return true;
}

extern "C" bool hal_i2c_busy(void) {
    return (i2c_state != ST_IDLE &&
            i2c_state != ST_COMPLETE &&
            i2c_state != ST_ERROR);
}

extern "C" hal_i2c_status_t hal_i2c_error(void) {
    return i2c_error;
}

/* ── I2C2 Event ISR ─────────────────────────────────────────────── */

extern "C" void I2C2_EV_IRQHandler(void) {
    uint16_t sr1 = I2C2->SR1;

    switch (i2c_state) {

    case ST_START_SENT:
        if (sr1 & I2C_SR1_SB) {
            /* SB set: send slave address (write) */
            I2C2->DR = (uint8_t)(xfer_addr << 1);
            i2c_state = ST_ADDR_W_SENT;
        }
        break;

    case ST_ADDR_W_SENT:
        if (sr1 & I2C_SR1_ADDR) {
            /* Clear ADDR by reading SR1+SR2 */
            (void)I2C2->SR1;
            (void)I2C2->SR2;
            /* Enable buffer interrupt for TXE */
            I2C2->CR2 |= I2C_CR2_ITBUFEN;
            i2c_state = ST_TX_DATA;
        }
        break;

    case ST_TX_DATA:
        if (sr1 & I2C_SR1_TXE) {
            if (xfer_tx_idx < xfer_tx_len) {
                I2C2->DR = xfer_tx_buf[xfer_tx_idx++];
            } else if (sr1 & I2C_SR1_BTF) {
                /* All bytes shifted out */
                I2C2->CR2 &= ~I2C_CR2_ITBUFEN;
                if (xfer_has_rx && xfer_rx_len > 0) {
                    /* Generate repeated START for read phase */
                    I2C2->CR1 |= I2C_CR1_START;
                    i2c_state = ST_RESTART_SENT;
                } else {
                    /* Write-only: generate STOP */
                    I2C2->CR1 |= I2C_CR1_STOP;
                    I2C2->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
                    i2c_state = ST_COMPLETE;
                }
            }
        }
        break;

    case ST_RESTART_SENT:
        if (sr1 & I2C_SR1_SB) {
            /* Send slave address (read) */
            I2C2->DR = (uint8_t)((xfer_addr << 1) | 1);
            i2c_state = ST_ADDR_R_SENT;
        }
        break;

    case ST_ADDR_R_SENT:
        if (sr1 & I2C_SR1_ADDR) {
            if (xfer_rx_len == 1) {
                /* Single byte: NACK before clearing ADDR, no DMA */
                I2C2->CR1 &= ~I2C_CR1_ACK;
                (void)I2C2->SR1;
                (void)I2C2->SR2;
                I2C2->CR1 |= I2C_CR1_STOP;
                /* Enable RXNE interrupt to read the byte */
                I2C2->CR2 |= I2C_CR2_ITBUFEN;
                i2c_state = ST_RX_DMA; /* Reuse state; RXNE handled below */
            } else {
                /* Multi-byte: use DMA */
                I2C2->CR1 |= I2C_CR1_ACK;
                setup_rx_dma();
                I2C2->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
                /* Clear ADDR to start reception */
                (void)I2C2->SR1;
                (void)I2C2->SR2;
                /* Disable event IRQ buffer bits; DMA TC will finish */
                I2C2->CR2 &= ~I2C_CR2_ITBUFEN;
                i2c_state = ST_RX_DMA;
            }
        }
        break;

    case ST_RX_DMA:
        /* Single-byte RX: read via RXNE interrupt */
        if ((sr1 & I2C_SR1_RXNE) && xfer_rx_len == 1) {
            xfer_rx_buf[0] = (uint8_t)I2C2->DR;
            I2C2->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN |
                            I2C_CR2_ITBUFEN);
            i2c_state = ST_COMPLETE;
        }
        break;

    default:
        break;
    }
}

/* ── I2C2 Error ISR ─────────────────────────────────────────────── */

extern "C" void I2C2_ER_IRQHandler(void) {
    uint16_t sr1 = I2C2->SR1;
    hal_i2c_status_t err = HAL_I2C_ERR_BUS;

    if (sr1 & I2C_SR1_AF) {
        I2C2->SR1 = (uint16_t)~I2C_SR1_AF;
        err = HAL_I2C_ERR_NACK;
    }
    if (sr1 & I2C_SR1_ARLO) {
        I2C2->SR1 = (uint16_t)~I2C_SR1_ARLO;
        err = HAL_I2C_ERR_BUS;
    }
    if (sr1 & I2C_SR1_BERR) {
        I2C2->SR1 = (uint16_t)~I2C_SR1_BERR;
        err = HAL_I2C_ERR_BUS;
    }

    i2c_stop_with_error(err);
}

/* ── DMA1 Stream2 ISR (I2C2 RX complete) ────────────────────────── */

extern "C" void DMA1_Stream2_IRQHandler(void) {
    /* Check transfer complete flag for Stream2 (bit 21 in LISR) */
    if (DMA1->LISR & (1U << 21)) {
        /* Clear all Stream2 flags */
        DMA1->LIFCR = (0x3FU << 16);

        /* Disable DMA on I2C */
        I2C2->CR2 &= ~(I2C_CR2_DMAEN | I2C_CR2_LAST);
        I2C2_RX_DMA_STREAM->CR &= ~DMA_SxCR_EN;

        /* Generate STOP */
        I2C2->CR1 |= I2C_CR1_STOP;

        /* Disable I2C interrupts */
        I2C2->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN | I2C_CR2_ITBUFEN);

        i2c_state = ST_COMPLETE;
    }
}
