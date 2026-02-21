/*
 * SPI HAL implementation for STM32F405.
 * SPI3 for AS5047D encoder on PC10/11/12 (AF6), CS on PB9.
 */

#include "hal/hal_spi.h"
#include "baremetal_config.h"
#include "stm32f4xx.h"

extern "C" void hal_spi_init(void) {
    /* Enable SPI3 clock */
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

    /* Ensure CS is high (deasserted) */
    GPIOB->BSRR = (1U << ENC_CSN_PIN);

    /* Configure SPI3:
     *   Master mode, 16-bit data frame
     *   CPOL=0, CPHA=1 (SPI Mode 1 - AS5047D requirement)
     *   Software NSS management, SSI high
     *   Baudrate: APB1(42MHz) / 8 = 5.25 MHz (AS5047D max 10MHz)
     *     BR = 010 -> SPI_CR1_BR_1
     */
    SPI3->CR1 = SPI_CR1_MSTR
              | SPI_CR1_DFF
              | SPI_CR1_SSM | SPI_CR1_SSI
              | SPI_CR1_CPHA
              | SPI_CR1_BR_1;

    SPI3->CR2 = 0;

    /* Enable SPI */
    SPI3->CR1 |= SPI_CR1_SPE;
}

extern "C" uint16_t hal_spi_transfer16(uint16_t tx_data) {
    /* Assert CS (active low) */
    GPIOB->BSRR = (1U << (ENC_CSN_PIN + 16));

    /* Wait for TXE then write */
    while (!(SPI3->SR & SPI_SR_TXE)) {}
    SPI3->DR = tx_data;

    /* Wait for RXNE then read */
    while (!(SPI3->SR & SPI_SR_RXNE)) {}
    uint16_t rx_data = SPI3->DR;

    /* Wait until not busy */
    while (SPI3->SR & SPI_SR_BSY) {}

    /* Deassert CS */
    GPIOB->BSRR = (1U << ENC_CSN_PIN);

    return rx_data;
}
