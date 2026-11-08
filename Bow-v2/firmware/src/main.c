/* Bow V2/V3 shared firmware: STM32C011F6P6, ICM-45686, TX-only J1 UART.
 * Common UART pin mapping (V2/V3); J1.3 DATA, J1.2 GND, J1.4 5V.
 * ICM register definitions: DS-000577 Rev 1.0 (not ICM-42688).
 */
#include "stm32c011xx.h"
#include <stdbool.h>

#ifndef SPI_BR
#define SPI_BR 3UL
#endif
#if SPI_BR < 0 || SPI_BR > 7
#error Invalid SPI_BR
#endif
#ifndef ADC_SPI_BR
#define ADC_SPI_BR SPI_BR
#endif
#ifndef BOARD_REV
#error Select config/v2.h or config/v3.h
#endif
#if BOARD_REV != 2 && BOARD_REV != 3
#error BOARD_REV must be 2 or 3
#endif
#define CPU_HZ 48000000UL
#ifndef BAUD
#define BAUD 3000000UL
#endif
#define IMU_CS (1UL << 7)     /* PB7, U3.1 */
#define ADC_CS (1UL << 15)    /* PC15, U3.3 */
static volatile uint32_t ticks;
uint32_t SystemCoreClock = CPU_HZ;

void SysTick_Handler(void) { ++ticks; }
static void delay_ms(uint32_t n) {
    uint32_t start = ticks;
    while ((uint32_t)(ticks - start) < n) __WFI();
}
static bool wait_bits(volatile uint32_t *reg, uint32_t mask,
                      bool set, uint32_t timeout) {
    uint32_t start = ticks;
    while (((*reg & mask) != 0) != set)
        if ((uint32_t)(ticks - start) >= timeout) return false;
    return true;
}
static void gpio_mode(GPIO_TypeDef *p, unsigned pin, unsigned mode) {
    p->MODER = (p->MODER & ~(3UL << (pin * 2))) | (mode << (pin * 2));
    p->PUPDR &= ~(3UL << (pin * 2));
}
static void gpio_af(GPIO_TypeDef *p, unsigned pin, unsigned af) {
    unsigned index = pin / 8, shift = (pin % 8) * 4;
    p->AFR[index] = (p->AFR[index] & ~(15UL << shift)) | (af << shift);
    gpio_mode(p, pin, 2);
}
static void board_init(void) {
    /* Reset starts on HSI48/4. Raise Flash latency BEFORE selecting HSI48/1. */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_0;
    while (!(FLASH->ACR & FLASH_ACR_LATENCY_0)) {}
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    RCC->CFGR = 0; /* HSI SYSCLK, AHB /1, APB /1 */
    while (RCC->CFGR & RCC_CFGR_SWS) {}
    RCC->CR &= ~RCC_CR_HSIDIV;
    SysTick_Config(CPU_HZ / 1000);
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;
    (void)RCC->IOPENR;
#if BOARD_REV >= 3
    GPIOA->BSRR = 1UL << 4; /* Active-low LED: set OFF before enabling output. */
    GPIOA->OTYPER &= ~(1UL << 4);
    gpio_mode(GPIOA, 4, 1);
    gpio_mode(GPIOA, 6, 0); /* IMU INT1; no external pull required for push-pull. */
#endif
    /* Deassert BOTH shared-SPI selects before enabling any SPI clocks. */
    GPIOB->BSRR = IMU_CS;
    GPIOC->BSRR = ADC_CS;
    gpio_mode(GPIOB, 7, 1);
    gpio_mode(GPIOC, 15, 1);
    /* Leave SWD PA13/PA14 in their reset configuration; no remapping. */
    gpio_mode(GPIOA, 0, 3); /* unused LTC input in this test */
    gpio_mode(GPIOA, 3, 0); /* ADC DRDY input; Board2 R7 pulls ADC_CS up, not DRDY. */
    GPIOC->OTYPER &= ~(1UL << 14); /* V2 TX-only push-pull; adapter TX must stay disconnected. */
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~(3UL << 28)) | (1UL << 28);
    gpio_af(GPIOC, 14, 0);       /* USART1_TX */
    RCC->APBENR2 |= RCC_APBENR2_USART1EN | RCC_APBENR2_SPI1EN;
    RCC->CCIPR &= ~RCC_CCIPR_USART1SEL; /* USART1 clock = PCLK=48MHz */
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0; /* Ordinary UART, TX only; no single-wire half-duplex. */
    USART1->BRR = (CPU_HZ + BAUD / 2) / BAUD;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE; /* 8N1, RX disabled */
    gpio_af(GPIOB, 6, 10); /* SPI1_SCK: AF10, not AF0 */
    gpio_af(GPIOA, 7, 0);  /* SPI1_MOSI */
    gpio_af(GPIOA, 11, 0); /* SPI1_MISO */
    /* Bring-up workaround: valid WHO_AM_I observed only with MCU MISO pull-up.
       Keep this weak bias during transfers; hardware cause remains unverified. */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << 22)) | (1UL << 22);
    GPIOB->OSPEEDR |= 2UL << (6 * 2);
    GPIOA->OSPEEDR |= 2UL << (7 * 2);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                (SPI_BR << SPI_CR1_BR_Pos); /* mode 0, MSB first */
    SPI1->CR2 = (7UL << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH; /* 8-bit */
    SPI1->CR1 |= SPI_CR1_SPE;
}
static void put_char(char c) {
    if (wait_bits(&USART1->ISR, USART_ISR_TXE_TXFNF, true, 20))
        USART1->TDR = (uint8_t)c;
}
static void put_str(const char *s) { while (*s) put_char(*s++); }
static void put_uint(uint32_t v) {
    char digits[10]; unsigned n = 0;
    do { digits[n++] = '0' + v % 10; v /= 10; } while (v);
    while (n) put_char(digits[--n]);
}
static void put_int(int32_t v) {
    if (v < 0) { put_char('-'); put_uint((uint32_t)(-(v + 1)) + 1); }
    else put_uint((uint32_t)v);
}
static bool spi_byte(uint8_t tx, uint8_t *rx) {
    if (!wait_bits(&SPI1->SR, SPI_SR_TXE, true, 10)) return false;
    *(volatile uint8_t *)&SPI1->DR = tx;
    if (!wait_bits(&SPI1->SR, SPI_SR_RXNE, true, 10)) return false;
    *rx = *(volatile uint8_t *)&SPI1->DR;
    return true;
}
/* Change mode only with both slaves deselected and the shifter idle. */
static bool spi_mode(bool adc) {
    if (!wait_bits(&SPI1->SR, SPI_SR_BSY, false, 10)) return false;
    GPIOB->BSRR = IMU_CS;
    GPIOC->BSRR = ADC_CS;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (SPI1->CR1 & ~(SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR)) |
                (adc ? SPI_CR1_CPHA : 0) |
                ((adc ? ADC_SPI_BR : SPI_BR) << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
    return true;
}
static bool imu_transfer(uint8_t addr, uint8_t *data, unsigned n, bool read) {
    uint8_t discard;
    if (!spi_mode(false)) return false;
    GPIOB->BSRR = IMU_CS << 16;
    bool ok = spi_byte(addr | (read ? 0x80 : 0), &discard);
    for (unsigned i = 0; ok && i < n; ++i) {
        uint8_t rx;
        ok = spi_byte(read ? 0 : data[i], &rx);
        if (read && ok) data[i] = rx;
    }
    if (!wait_bits(&SPI1->SR, SPI_SR_BSY, false, 10)) ok = false;
    GPIOB->BSRR = IMU_CS;
    return ok;
}
static bool rd(uint8_t addr, uint8_t *value) { return imu_transfer(addr, value, 1, true); }
static bool wr(uint8_t addr, uint8_t value) { return imu_transfer(addr, &value, 1, false); }
static bool ireg_done(void) {
    uint8_t value;
    /* Datasheet minimum IREG gap is 4us; 1ms is ample at startup. */
    for (unsigned i = 0; i < 20; ++i) {
        delay_ms(1);
        if (!rd(0x7F, &value)) return false;
        if (value & 1) return true;
    }
    return false;
}
static bool ireg_update(uint16_t addr, uint8_t mask, uint8_t set) {
    uint8_t a[3] = {addr >> 8, addr & 0xFF, 0};
    if (!imu_transfer(0x7C, a, 2, false) || !ireg_done() || !rd(0x7E, &a[2])) return false;
    /* Reading auto-increments/prefetches; allow completion before new address. */
    if (!ireg_done()) return false;
    a[2] = (a[2] & (uint8_t)~mask) | set;
    return imu_transfer(0x7C, a, 3, false) && ireg_done();
}
static bool ireg_clear(uint16_t addr, uint8_t mask) {
    return ireg_update(addr, mask, 0);
}
/* Read-only probes after an invalid ID. Weak input pulls do not drive MISO.
 * These distinguish floating/no-response from a line held low, not exact faults.
 * Restore both peripheral clock divider and GPIO pulls before returning. */
static void imu_diagnose(void) {
    uint32_t cr1 = SPI1->CR1;
    uint32_t pulls = GPIOA->PUPDR;
    uint8_t value;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = (cr1 & ~(SPI_CR1_SPE | SPI_CR1_BR)) | (7UL << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE; /* 187.5kHz */
    GPIOA->PUPDR = pulls & ~(3UL << 22);
    delay_ms(1);
    put_str("# DIAG WHO slow187k=");
    if (rd(0x72, &value)) put_uint(value); else put_str("TIMEOUT");
    GPIOA->PUPDR = (pulls & ~(3UL << 22)) | (1UL << 22);
    delay_ms(1);
    put_str(" pullup=");
    if (rd(0x72, &value)) put_uint(value); else put_str("TIMEOUT");
    GPIOA->PUPDR = (pulls & ~(3UL << 22)) | (2UL << 22);
    delay_ms(1);
    put_str(" pulldown=");
    if (rd(0x72, &value)) put_uint(value); else put_str("TIMEOUT");
    GPIOA->PUPDR = pulls;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = cr1;
    put_str(" ADC_CS_pin="); put_uint((GPIOC->IDR >> 15) & 1);
    put_str(" IMU_CS_idle_pin="); put_uint((GPIOB->IDR >> 7) & 1);
    put_str("\r\n");
}
static bool init_error(const char *stage) {
    put_str("# ERROR init stage: "); put_str(stage); put_str("\r\n");
    return false;
}
static bool imu_init(void) {
    uint8_t value = 0;
    if (!rd(0x72, &value)) { put_str("# ERROR SPI timeout\r\n"); return false; }
    put_str("# WHO_AM_I="); put_uint(value); put_str(" (expected 233 / 0xE9)\r\n");
    if (value != 0xE9) { imu_diagnose(); return false; }
    if (!wr(0x7F, 0x02)) return init_error("soft reset write"); /* SOFT_RST */
    delay_ms(100);
    if (!rd(0x19, &value) || !(value & 0x80)) return init_error("RESET_DONE read"); /* RESET_DONE */
    if (!wr(0x1B, 0x39) || !wr(0x16, 0x04) || !wr(0x10, 0x03)) return init_error("accel configuration write");
    /* +/-4g, 100Hz; DRDY status enabled; accel LN, gyro OFF. */
    delay_ms(100);
    /* Active AP SPI pulls off; preserve the unused AUX/INT pad pulls.
       Explicit little-endian, matching DS-000577 section 15. */
    if (!ireg_clear(0xA03A, 0x48) || !ireg_clear(0xA03B, 0x12) ||
        !ireg_clear(0xA267, 0x02)) return init_error("IREG pads/endian");
    if (!rd(0x1B, &value) || value != 0x39) return init_error("ACCEL_CONFIG0 readback");
    if (!rd(0x10, &value) || (value & 0x0F) != 0x03) return init_error("PWR_MGMT0 readback");
    return true;
}
#include "fast_sensors.inc"
