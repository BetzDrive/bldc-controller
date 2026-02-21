#ifdef STM32F405xx

#include "hal/hal_flash.h"
#include "stm32f4xx.h"

#include <cstring>

/* STM32F405 flash sector layout */
static const struct {
    uint32_t start;
    uint32_t size;
} flash_sectors[12] = {
    { 0x08000000,  16 * 1024 },  /* Sector 0  - 16KB  */
    { 0x08004000,  16 * 1024 },  /* Sector 1  - 16KB  */
    { 0x08008000,  16 * 1024 },  /* Sector 2  - 16KB  */
    { 0x0800C000,  16 * 1024 },  /* Sector 3  - 16KB  */
    { 0x08010000,  64 * 1024 },  /* Sector 4  - 64KB  */
    { 0x08020000, 128 * 1024 },  /* Sector 5  - 128KB */
    { 0x08040000, 128 * 1024 },  /* Sector 6  - 128KB */
    { 0x08060000, 128 * 1024 },  /* Sector 7  - 128KB */
    { 0x08080000, 128 * 1024 },  /* Sector 8  - 128KB */
    { 0x080A0000, 128 * 1024 },  /* Sector 9  - 128KB */
    { 0x080C0000, 128 * 1024 },  /* Sector 10 - 128KB */
    { 0x080E0000, 128 * 1024 },  /* Sector 11 - 128KB */
};

static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
}

static void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_busy(void) {
    while (FLASH->SR & FLASH_SR_BSY) {}
}

/* Find sector number containing address. Returns -1 if not found. */
static int flash_sector_of(uint32_t addr) {
    for (int i = 0; i < 12; i++) {
        if (addr >= flash_sectors[i].start &&
            addr < flash_sectors[i].start + flash_sectors[i].size) {
            return i;
        }
    }
    return -1;
}

bool hal_flash_erase(uint32_t addr, size_t len) {
    flash_unlock();

    uint32_t end = addr + len;
    for (int s = flash_sector_of(addr); s >= 0 && s < 12; s++) {
        if (flash_sectors[s].start >= end) break;

        flash_wait_busy();
        FLASH->CR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
        FLASH->CR |= FLASH_CR_SER | FLASH_CR_PSIZE_1 |
                      ((uint32_t)s << 3);  /* SNB starts at bit 3 */
        FLASH->CR |= FLASH_CR_STRT;
        flash_wait_busy();
        FLASH->CR &= ~FLASH_CR_SER;

        if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)) {
            FLASH->SR = FLASH->SR; /* Clear errors */
            flash_lock();
            return false;
        }
    }

    flash_lock();
    return true;
}

bool hal_flash_write(uint32_t addr, const void *data, size_t len) {
    const uint8_t *src = (const uint8_t *)data;
    flash_unlock();

    /* Program word-at-a-time where aligned, byte otherwise */
    FLASH->CR &= ~(FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_CR_PG | FLASH_CR_PSIZE_1; /* 32-bit parallelism */

    size_t i = 0;
    /* Handle unaligned prefix byte-by-byte */
    while (i < len && (addr + i) % 4 != 0) {
        FLASH->CR &= ~FLASH_CR_PSIZE;
        FLASH->CR |= FLASH_CR_PG; /* 8-bit */
        *(volatile uint8_t *)(addr + i) = src[i];
        flash_wait_busy();
        i++;
    }

    /* Word-aligned writes */
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PG | FLASH_CR_PSIZE_1; /* 32-bit */
    while (i + 4 <= len) {
        uint32_t word;
        std::memcpy(&word, &src[i], 4);
        *(volatile uint32_t *)(addr + i) = word;
        flash_wait_busy();
        i += 4;
    }

    /* Trailing bytes */
    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PG; /* 8-bit */
    while (i < len) {
        *(volatile uint8_t *)(addr + i) = src[i];
        flash_wait_busy();
        i++;
    }

    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();

    bool ok = !(FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR));
    FLASH->SR = FLASH->SR; /* Clear flags */
    return ok;
}

void hal_flash_read(uint32_t addr, void *buf, size_t len) {
    std::memcpy(buf, (const void *)addr, len);
}

bool hal_flash_verify(uint32_t addr, const void *data, size_t len) {
    return std::memcmp((const void *)addr, data, len) == 0;
}

bool hal_flash_verify_erased(uint32_t addr, size_t len) {
    const uint8_t *p = (const uint8_t *)addr;
    for (size_t i = 0; i < len; i++) {
        if (p[i] != 0xFF) return false;
    }
    return true;
}

void hal_flash_jump(uint32_t addr) {
    /* Disable all interrupts */
    __disable_irq();

    /* Set vector table to target */
    SCB->VTOR = addr;

    /* Set MSP to the value at addr[0] and jump to addr[4] */
    uint32_t msp = *(volatile uint32_t *)addr;
    uint32_t reset = *(volatile uint32_t *)(addr + 4);
    __set_MSP(msp);
    ((void (*)(void))reset)();
    __builtin_unreachable();
}

#endif /* STM32F405xx */
