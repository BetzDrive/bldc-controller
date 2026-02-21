/*
 * GPIO configuration and HAL implementation for STM32F405.
 * Pin assignments from baremetal_config.h / board.h.
 */

#include "stm32f4xx.h"
#include "baremetal_config.h"
#include "hal/hal_gpio.h"

/* Helper to set a pin's mode in GPIOx->MODER (2 bits per pin) */
static inline void set_mode(GPIO_TypeDef *gpio, uint32_t pin, uint32_t mode) {
    gpio->MODER = (gpio->MODER & ~(3U << (pin * 2))) | (mode << (pin * 2));
}

/* Helper to set output type (1 bit per pin) */
static inline void set_otype(GPIO_TypeDef *gpio, uint32_t pin, uint32_t otype) {
    gpio->OTYPER = (gpio->OTYPER & ~(1U << pin)) | (otype << pin);
}

/* Helper to set output speed (2 bits per pin) */
static inline void set_ospeed(GPIO_TypeDef *gpio, uint32_t pin, uint32_t speed) {
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3U << (pin * 2))) | (speed << (pin * 2));
}

/* Helper to set pull-up/pull-down (2 bits per pin) */
static inline void set_pupd(GPIO_TypeDef *gpio, uint32_t pin, uint32_t pupd) {
    gpio->PUPDR = (gpio->PUPDR & ~(3U << (pin * 2))) | (pupd << (pin * 2));
}

/* Helper to set alternate function (4 bits per pin, AFR[0] for pins 0-7, AFR[1] for 8-15) */
static inline void set_af(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af) {
    uint32_t idx = pin >> 3;
    uint32_t pos = (pin & 7U) * 4;
    gpio->AFR[idx] = (gpio->AFR[idx] & ~(0xFU << pos)) | (af << pos);
}

/* MODER values */
#define MODE_INPUT   0U
#define MODE_OUTPUT  1U
#define MODE_AF      2U
#define MODE_ANALOG  3U

/* OTYPER values */
#define OTYPE_PP     0U
#define OTYPE_OD     1U

/* OSPEEDR values */
#define SPEED_LOW    0U
#define SPEED_MED    1U
#define SPEED_HIGH   2U
#define SPEED_VHIGH  3U

/* PUPDR values */
#define PUPD_NONE    0U
#define PUPD_UP      1U
#define PUPD_DOWN    2U

extern "C" void gpio_init(void) {
    /* ── GPIOA ──────────────────────────────────────────── */

    /* PA0-2: AF2 (TIM5 CH1-3, LEDs), push-pull, high speed */
    for (uint32_t pin = 0; pin <= 2; pin++) {
        set_mode(GPIOA, pin, MODE_AF);
        set_af(GPIOA, pin, 2);
        set_otype(GPIOA, pin, OTYPE_PP);
        set_ospeed(GPIOA, pin, SPEED_HIGH);
    }

    /* PA8-10: AF1 (TIM1 CH1-3, motor PWM), push-pull, very high speed */
    for (uint32_t pin = 8; pin <= 10; pin++) {
        set_mode(GPIOA, pin, MODE_AF);
        set_af(GPIOA, pin, 1);
        set_otype(GPIOA, pin, OTYPE_PP);
        set_ospeed(GPIOA, pin, SPEED_VHIGH);
    }

    /* PA12: Output push-pull (RS485 shunt enable) */
    set_mode(GPIOA, 12, MODE_OUTPUT);
    set_otype(GPIOA, 12, OTYPE_PP);

    /* PA13-14: Leave as SWD (don't touch) */

    /* PA15: Output push-pull (disco bus out), initially high */
    set_mode(GPIOA, 15, MODE_OUTPUT);
    set_otype(GPIOA, 15, OTYPE_PP);
    GPIOA->BSRR = (1U << 15);

    /* ── GPIOB ──────────────────────────────────────────── */

    /* PB4: Input floating (disco bus in) */
    set_mode(GPIOB, 4, MODE_INPUT);
    set_pupd(GPIOB, 4, PUPD_NONE);

    /* PB6: AF7 (USART1_TX), push-pull, high speed */
    set_mode(GPIOB, 6, MODE_AF);
    set_af(GPIOB, 6, 7);
    set_otype(GPIOB, 6, OTYPE_PP);
    set_ospeed(GPIOB, 6, SPEED_HIGH);

    /* PB7: AF7 (USART1_RX), pull-up */
    set_mode(GPIOB, 7, MODE_AF);
    set_af(GPIOB, 7, 7);
    set_pupd(GPIOB, 7, PUPD_UP);

    /* PB8: Output push-pull (LED_Y) */
    set_mode(GPIOB, 8, MODE_OUTPUT);
    set_otype(GPIOB, 8, OTYPE_PP);

    /* PB9: Output push-pull (ENC_CSN), initially high */
    set_mode(GPIOB, 9, MODE_OUTPUT);
    set_otype(GPIOB, 9, OTYPE_PP);
    GPIOB->BSRR = (1U << 9);

    /* PB10: AF4 open-drain (I2C2_SCL), pull-up */
    set_mode(GPIOB, 10, MODE_AF);
    set_af(GPIOB, 10, 4);
    set_otype(GPIOB, 10, OTYPE_OD);
    set_pupd(GPIOB, 10, PUPD_UP);

    /* PB11: AF4 open-drain (I2C2_SDA), pull-up */
    set_mode(GPIOB, 11, MODE_AF);
    set_af(GPIOB, 11, 4);
    set_otype(GPIOB, 11, OTYPE_OD);
    set_pupd(GPIOB, 11, PUPD_UP);

    /* PB13-14: Input (TEMP_INT, IMU_INT1) */
    set_mode(GPIOB, 13, MODE_INPUT);
    set_mode(GPIOB, 14, MODE_INPUT);

    /* PB15: Input pull-up (MDRV_NOCTW, active low) */
    set_mode(GPIOB, 15, MODE_INPUT);
    set_pupd(GPIOB, 15, PUPD_UP);

    /* ── GPIOC ──────────────────────────────────────────── */

    /* PC0-3: Analog input (ADC current/voltage sense) */
    for (uint32_t pin = 0; pin <= 3; pin++) {
        set_mode(GPIOC, pin, MODE_ANALOG);
    }

    /* PC6: Input pull-up (MDRV_NFAULT, active low) */
    set_mode(GPIOC, 6, MODE_INPUT);
    set_pupd(GPIOC, 6, PUPD_UP);

    /* PC7-9: Output push-pull (MDRV_RST_A/B/C), initially low (driver disabled) */
    for (uint32_t pin = 7; pin <= 9; pin++) {
        set_mode(GPIOC, pin, MODE_OUTPUT);
        set_otype(GPIOC, pin, OTYPE_PP);
        GPIOC->BSRR = (1U << (pin + 16));  /* Clear bit (set low) */
    }

    /* PC10: AF6 (SPI3_SCK), push-pull, high speed */
    set_mode(GPIOC, 10, MODE_AF);
    set_af(GPIOC, 10, 6);
    set_otype(GPIOC, 10, OTYPE_PP);
    set_ospeed(GPIOC, 10, SPEED_HIGH);

    /* PC11: AF6 (SPI3_MISO) */
    set_mode(GPIOC, 11, MODE_AF);
    set_af(GPIOC, 11, 6);

    /* PC12: AF6 (SPI3_MOSI), push-pull, high speed */
    set_mode(GPIOC, 12, MODE_AF);
    set_af(GPIOC, 12, 6);
    set_otype(GPIOC, 12, OTYPE_PP);
    set_ospeed(GPIOC, 12, SPEED_HIGH);

    /* ── GPIOD ──────────────────────────────────────────── */

    /* PD2: Output push-pull (RS485_DIR), initially low (receive mode) */
    set_mode(GPIOD, 2, MODE_OUTPUT);
    set_otype(GPIOD, 2, OTYPE_PP);
    GPIOD->BSRR = (1U << (2 + 16));  /* Clear bit (set low) */
}

/* ── hal_gpio implementation ───────────────────────────────── */

extern "C" void hal_gpio_set(uint32_t port_base, uint32_t pin) {
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)port_base;
    gpio->BSRR = (1U << pin);
}

extern "C" void hal_gpio_clear(uint32_t port_base, uint32_t pin) {
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)port_base;
    gpio->BSRR = (1U << (pin + 16));
}

extern "C" void hal_gpio_write(uint32_t port_base, uint32_t pin, bool value) {
    if (value)
        hal_gpio_set(port_base, pin);
    else
        hal_gpio_clear(port_base, pin);
}

extern "C" bool hal_gpio_read(uint32_t port_base, uint32_t pin) {
    GPIO_TypeDef *gpio = (GPIO_TypeDef *)port_base;
    return (gpio->IDR & (1U << pin)) != 0;
}
