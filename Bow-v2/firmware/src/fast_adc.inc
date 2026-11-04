/* One raw result is retained until the next START has completed. Packing/CRC
 * then overlaps the next ADC conversion instead of lengthening ADC idle time. */
enum { ADC_SLOT_US = 500, ADC_TIMEOUT_US = 20000 };
static bool pending, adc_result_ready;
static unsigned channel;
static uint32_t adc_events_used, start_adc, adc_seq, late_slots, max_late;
static struct {
    uint8_t bytes[5];
    unsigned channel;
    uint32_t start_us, ready_us, drdy_us;
} adc_result;
static void adc_emit_result(void) {
    if (!adc_result_ready) return;
    const uint8_t *rx = adc_result.bytes;
    uint8_t payload[18] = {0};
    uint32_t code = ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    int32_t raw = (int32_t)code - ((code & 0x800000) ? 16777216 : 0);
    pack32(payload, (uint32_t)raw);
    pack16(payload + 4, ((uint16_t)rx[0] << 8) | rx[1]);
    payload[6] = adc_result.channel; payload[7] = 32;
    pack32(payload + 8, adc_result.start_us);
    pack32(payload + 12, adc_result.drdy_us - adc_result.start_us);
    send_packet(2, adc_seq++, adc_result.ready_us, payload);
    adc_result_ready = false;
}
static void fast_adc_poll(void) {
    if (!adc_alive) { adc_emit_result(); return; }
    uint32_t now = clock_us();
    if (pending) {
        uint32_t drdy_us;
        if (!adc_drdy_take(start_adc, &drdy_us)) {
            if ((uint32_t)(now - start_adc) > ADC_TIMEOUT_US) {
                adc_alive = false; fault_flags |= 1;
            }
            return;
        }
        now = clock_us();
        if (!adc_frame(0, adc_result.bytes, 5) || !(adc_result.bytes[0] & 1)) {
            adc_alive = false; fault_flags |= 1; return;
        }
        adc_result.channel = channel;
        adc_result.start_us = start_adc;
        /* Keep packet timestamps in main-loop observation order. An earlier
           ISR timestamp could otherwise look like a reset after an IMU packet. */
        adc_result.ready_us = now;
        adc_result.drdy_us = drdy_us;
        adc_result_ready = true;
        pending = false; channel ^= 1;
    }
    /* Consume the newest hardware timer event once. If several deadlines
       elapsed, skip old slots explicitly rather than issuing catch-up bursts. */
    uint32_t events, due;
    adc_timer_snapshot(&events, &due);
    if (events == adc_events_used) return;
    now = clock_us();
    uint32_t late = now - due;
    if (late > max_late) max_late = late;
    late_slots += events - adc_events_used - 1;
    adc_events_used = events;
    bool mux_ok = adc_wr(0x07, channel ? 0x23 : 0x10);
    adc_drdy_prepare();
    if (!mux_ok || !adc_wr(0x04, 2)) {
        adc_alive = false; fault_flags |= 1;
    } else {
        start_adc = clock_us(); pending = true;
    }
    adc_emit_result(); /* Also preserve the last valid sample on START failure. */
}
