/* Dedicated nonblocking near-1kHz acquisition. No software smoothing.
 * TI SBASAI9 Table 7-6: mode3/OSR256 latency392/1.024MHz=382.8us.
 * ICM DS-000577: 1600Hz LN, HIRES fixed scale +/-32g +/-4000dps, fixed FIR AAF + UI LPF ODR/4=400Hz. */
#include <stddef.h>
void *memset(void *dst, int value, size_t n) {
    unsigned char *p = dst;
    for (size_t i = 0; i < n; ++i) p[i] = (unsigned char)value;
    return dst;
}
#define ADC_SPEED_MODE 3
#define ADC_FILTER 3
#include "adc_common.inc"
#include "fast_protocol.h"
#include "fast_tx.inc"
/* Extend TIM3's 16-bit 1MHz counter; called much more often than 65ms.
 * Timer starts only after blocking initialization. Runtime never sleeps. */
static uint32_t clock_us(void) {
    static uint16_t previous;
    static uint32_t elapsed;
    uint16_t current = TIM3->CNT;
    elapsed += (uint16_t)(current - previous);
    previous = current;
    return elapsed;
}
#include "fast_timer.inc"
static bool ireg_checked(uint16_t address, uint8_t mask, uint8_t value) {
    uint8_t a[2] = {address >> 8, address & 255}, got;
    return ireg_update(address, mask, value) &&
        imu_transfer(0x7C, a, 2, false) && ireg_done() && rd(0x7E, &got) &&
        ireg_done() && (got & mask) == value;
}
static bool fast_write(uint8_t r, uint8_t v) {
    uint8_t got;
    return wr(r, v) && rd(r, &got) && got == v;
}
static bool fast_imu_init(void) {
    if (!imu_init()) return false;
    if (!fast_write(0x10, 0) || !fast_write(0x1B, 0x25) ||
        !fast_write(0x1C, 0x15)) return init_error("1600Hz ODR/range");
    /* Preserve unrelated reserved/OIS bits. Fixed FIR on, interpolator off. */
    if (!ireg_checked(0xA4A6, 0x60, 0x20) || !ireg_checked(0xA57B, 0x03, 0x01) ||
        !ireg_checked(0xA4AC, 0x07, 0x01) || !ireg_checked(0xA583, 0x07, 0x01) ||
        !ireg_checked(0xA258, 0x03, 0x01)) return init_error("AAF/LPF/timestamp");
    if (!fast_write(0x10, 0x0F)) return false;
    delay_ms(100);
#if BOARD_REV >= 3
    /* DS-000577 sections 17.21-23, 17.29-31: disable sources before
       changing INT1 polarity/mode; watermark is in FIFO frames, not bytes. */
    if (!fast_write(0x16, 0) || !fast_write(0x17, 0) ||
        !fast_write(0x18, 0x03) || /* push-pull, latched, active high */
        !fast_write(0x1D, 7) || !fast_write(0x20, 0x28) ||
        !fast_write(0x1E, 2) || !fast_write(0x1F, 0))
        return init_error("INT1 FIFO watermark");
#endif
    return fast_write(0x23, 0) && fast_write(0x22, 2) && fast_write(0x21, 14) &&
        fast_write(0x1D, 7) && fast_write(0x16, BOARD_REV >= 3 ? 3 : 1) &&
        fast_write(0x1D, 0x87) && fast_write(0x21, 15);
}
static bool adc_alive, imu_alive;
static uint32_t imu_seq, fault_flags;
#include "fast_adc.inc"
enum { IMU_GUARD_US = 200, IMU_FIFO_CAPACITY = 2048 / 20 };
static bool background_allowed(void) {
    if (!adc_alive) return true;
    uint32_t events, due;
    adc_timer_snapshot(&events, &due);
    if (adc_result_ready || events != adc_events_used) return false;
    uint32_t now = clock_us();
    return (int32_t)((due + ADC_SLOT_US) - now) > IMU_GUARD_US;
}
#include "fast_ltc.inc"
#include "fast_board.inc"
#include "fast_imu.inc"
int main(void) {
    board_init(); delay_ms(300);
    put_str("# Bow hardware V"); put_uint(BOARD_REV);
    put_str(" fast timer-DMA v9 FIFOcached LTC25: IMU1600Hz HIRES fixed-scale32g/4000dps AAF+LPF400Hz; bridges1000SPS each mode3 OSR256 gain32; SPI=");
    put_uint(CPU_HZ >> (SPI_BR + 1)); put_str(" UART"); put_uint(BAUD); put_str("; binary v2 follows init\r\n");
#ifdef DISABLE_LTC_IRQ
    put_str("# LTC disabled: PA0 EXTI off; IMU/ADC full stream; LTC counters stay zero\r\n");
#endif
    adc_alive = adc_init();
    if (!adc_alive) fault_flags |= 1;
    while (!fast_imu_init()) {
        put_str("# IMU init retry\r\n");
        for (unsigned i=0; i<100; ++i) { status_led_poll(true); delay_ms(10); }
    }
    imu_alive = true;
    put_str(BOARD_REV >= 3 ? "# INT1 PA6: 2-frame watermark, 5ms fallback; LED PA4 active-low\r\n" : "# V2 FIFO polling\r\n");
    put_str("# BINARY_START\r\n");
    tx_init();
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    (void)RCC->APBENR1;
    TIM3->PSC = 47; TIM3->ARR = 65535; TIM3->EGR = TIM_EGR_UG;
    TIM3->CNT = 0; TIM3->SR = 0;
    TIM3->CCR1 = 500; TIM3->DIER = TIM_DIER_CC1IE;
    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_SetPriority(SysTick_IRQn, 1);
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 = TIM_CR1_CEN;
    ltc_init(); imu_irq_init();
    uint32_t next_status = 0, status_seq = 0;
    for (;;) {
        tx_pump(); fast_adc_poll();
        status_led_poll(!adc_alive || !imu_alive || fault_flags || tx_errors || tx_drops);
        /* A completed ADC sample waits only until the next scheduled START.
           Keep timestamp order and avoid IMU/CRC work in that short gap. */
        if (!background_allowed()) continue;
        uint32_t now = clock_us();
        if (imu_alive && imu_service_due(now)) {
            fast_imu_poll();
        }
        if (!background_allowed()) continue;
        fast_ltc_poll();
        if (!background_allowed()) continue;
        now = clock_us();
        if ((int32_t)(now - next_status) >= 0) {
            uint8_t payload[18] = {0};
#if BOARD_REV >= 3
            /* Diagnostic only: data still flows through the fallback poll. */
            if (now > 100000 && imu_alive && !imu_irq_count) fault_flags |= 128;
#endif
            pack32(payload, tx_drops); pack32(payload + 4, late_slots);
            pack32(payload + 8, max_late); pack32(payload + 12, fault_flags | (tx_errors ? 64 : 0));
            pack16(payload + 16, (adc_alive ? 1 : 0) | (imu_alive ? 2 : 0));
            send_packet(3, status_seq++, now, payload);
            imu_send_diagnostics(now);
            next_status = now + 1000000;
        }
    }
}
