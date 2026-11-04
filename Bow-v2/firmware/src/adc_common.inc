/* ADS122S14 SBASAI9, sections 7.4, 7.5 and 8.
 * Commands are right-aligned and register responses arrive in the NEXT frame.
 * SPI mode 1, 187.5kHz; ADC and IMU use separate chip selects.
 */
#ifndef ADC_SPEED_MODE
#define ADC_SPEED_MODE 1
#endif
#ifndef ADC_FILTER
#define ADC_FILTER 5
#endif
static unsigned adc_header;
static bool adc_frame(const uint8_t *tx, uint8_t *rx, unsigned n) {
    if (!spi_mode(true)) return false;
    GPIOC->BSRR = ADC_CS << 16;
    bool ok = true;
    for (unsigned i = 0; i < n && ok; ++i) {
        uint8_t value;
        ok = spi_byte(tx ? tx[i] : 0, &value);
        if (rx && ok) rx[i] = value;
    }
    if (!wait_bits(&SPI1->SR, SPI_SR_BSY, false, 10)) ok = false;
    GPIOC->BSRR = ADC_CS;
    return ok;
}
static bool adc_wr(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = {0x80 | addr, value};
    return adc_frame(tx, 0, 2);
}
static bool adc_rd(uint8_t addr, uint8_t *value) {
    uint8_t tx[2] = {0x40 | addr, 0}, rx[5];
    if (!adc_frame(tx, 0, 2) || !adc_frame(0, rx, 3 + adc_header)) return false;
    /* Verify echoed address and pad to reject stuck-low/high MISO. */
    if (rx[adc_header + 1] != addr || rx[adc_header + 2] != 0) return false;
    *value = rx[adc_header];
    return true;
}
static bool adc_config(uint8_t addr, uint8_t value) {
    uint8_t actual = 0;
    if (!adc_wr(addr, value)) return false;
    if (addr == 0x0A) adc_header = (value & 0x10) ? 2 : 0;
    if (!adc_rd(addr, &actual) || actual != value) {
        put_str("# ADC ERROR register readback addr="); put_uint(addr);
        put_str(" expected="); put_uint(value); put_str(" got="); put_uint(actual);
        put_str("\r\n"); return false;
    }
    return true;
}
static bool adc_finish(int32_t *raw, uint16_t *status) {
    uint8_t rx[5];
    if (!adc_frame(0, rx, 5)) return false;
    *status = ((uint16_t)rx[0] << 8) | rx[1];
    if (!(rx[0] & 1)) {
        put_str("# ADC ERROR DRDY pin low but status has no fresh sample\r\n");
        return false;
    }
    uint32_t code = ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    *raw = (int32_t)code - ((code & 0x800000) ? 16777216 : 0);
    return true;
}
static bool adc_sample(uint8_t mux, int32_t *raw, uint16_t *status) {
    if (!adc_wr(0x07, mux) || !adc_wr(0x04, 0x02)) return false;
    uint32_t start = ticks;
    while (GPIOA->IDR & (1UL << 3)) {
        if ((uint32_t)(ticks - start) > 100) {
            uint8_t snapshot[5];
            put_str("# ADC ERROR DRDY timeout; PA3=1 STATUS_MSB=");
            /* Snapshot header directly: a register read would clear DRDY in frame1. */
            if (adc_frame(0, snapshot, 5)) put_uint(snapshot[0]); else put_str("READ_FAIL");
            put_str("; check U1.8->U3.10 and GPIO configuration\r\n");
            return false;
        }
    }
    return adc_finish(raw, status);
}
static bool adc_init(void) {
    /* 1024 ones + CS rising resets regardless of the prior CRC/config state. */
    uint8_t ones[128];
    for (unsigned i = 0; i < sizeof ones; ++i) ones[i] = 0xFF;
    if (!adc_frame(ones, 0, sizeof ones)) return false;
    adc_header = 0;
    delay_ms(10); /* >=500us after reset */
    uint8_t id = 0, revision = 0;
    if (!adc_rd(0, &id) || !adc_rd(1, &revision)) {
        put_str("# ADC ERROR ID read/echo failed\r\n"); return false;
    }
    put_str("# ADC DEVICE_ID="); put_uint(id);
    put_str(" REVISION_ID="); put_uint(revision);
    put_str(" (DEVICE_ID low nibble must be 11)\r\n");
    if ((id & 0x0F) != 0x0B) return false;
    /* Single-shot, idle between samples; selectable speed/filter; gain32; AVDD ref.
       Configure GPIO3 push-pull DRDY; Board2 R7 is on ADC_CS. */
    if (!adc_config(0x05, 0x04 | ADC_SPEED_MODE) || !adc_config(0x06, ADC_FILTER) ||
        !adc_config(0x08, 0x09) || !adc_config(0x09, 0x02) ||
        !adc_config(0x0C, 0x80) || !adc_config(0x0B, 0x80) ||
        !adc_config(0x0A, 0x10) || !adc_wr(0x02, 0xC0)) return false;
    /* Isolate ADC conversion from external bridge wiring first. */
    if (!adc_config(0x08, 0x19)) return false;
    for (unsigned i = 0; i < 3; ++i) {
        int32_t raw; uint16_t status;
        if (!adc_sample(0x10, &raw, &status)) return false;
        put_str("# ADC internal_short raw="); put_int(raw);
        put_str(" status="); put_uint(status); put_str("\r\n");
    }
    if (!adc_config(0x08, 0x09)) return false;
    put_str("# ADC ready; gain=32 ref=AVDD nominal3300mV; speed_mode="); put_uint(ADC_SPEED_MODE);
    put_str(" filter_code="); put_uint(ADC_FILTER); put_str("\r\n");
    return true;
}
