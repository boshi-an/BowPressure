static uint32_t last_imu_us;
static unsigned imu_cached_count;
/* Freeze the last SPI observation when IMU is stopped; report separately from data. */
static uint16_t imu_last_count, imu_peak_count;
static uint8_t imu_last_status;
static uint32_t imu_poll_count, imu_observed_us, imu_diag_seq;
static void imu_send_diagnostics(uint32_t now) {
    uint8_t p[18] = {0};
    p[0] = imu_last_status; p[1] = (GPIOA->IDR >> 6) & 1;
    pack16(p + 2, imu_last_count); pack32(p + 4, imu_irq_count);
    pack32(p + 8, imu_poll_count); pack32(p + 12, imu_observed_us);
    pack16(p + 16, imu_peak_count);
    send_packet(5, imu_diag_seq++, now, p);
}
static void fast_imu_poll(void) {
    if (!background_allowed()) return;
    if (!imu_cached_count) {
    imu_service_begin(clock_us());
    uint8_t status, count_data[2];
    if (!rd(0x19, &status) || !imu_transfer(0x12, count_data, 2, true)) {
        imu_alive = false; fault_flags |= 2; return;
    }
    unsigned count = count_data[0] | ((unsigned)count_data[1] << 8);
    imu_last_status = status; imu_last_count = count;
    if (count > imu_peak_count) imu_peak_count = count;
    ++imu_poll_count; imu_observed_us = clock_us();
    if ((status & 1) || count >= IMU_FIFO_CAPACITY) { imu_alive = false; fault_flags |= 4; return; }
    if (!count && (uint32_t)(clock_us() - last_imu_us) > 100000) {
        imu_alive = false; fault_flags |= 32; return;
    }
    imu_cached_count = count;
    }
    /* A count snapshot remains a safe lower bound as hardware adds frames.
       Preserve it across ADC deadlines instead of spending the next window
       querying STATUS/COUNT again before reading the same waiting frames. */
    unsigned count = imu_cached_count;
    if (count > 4) count = 4;
    for (unsigned n = 0; n < count; ++n) {
        if (!background_allowed()) break;
        uint8_t frame[20], payload[18] = {0};
        if (!imu_transfer(0x14, frame, 20, true) || (frame[0] & 0xFC) != 0x78) {
            imu_alive = false; fault_flags |= 8; return;
        }
        uint32_t read_us = clock_us();
        last_imu_us = read_us;
        bool valid = true;
        for (unsigned axis = 0; axis < 6; ++axis) {
            unsigned j = 1 + 2 * axis;
            uint8_t low = axis < 3 ? frame[17 + axis] >> 4 : frame[14 + axis] & 15;
            /* Only 0x80000 is invalid; 0x80002 is a valid negative endpoint. */
            if (frame[j] == 0 && frame[j + 1] == 0x80 && low == 0) valid = false;
        }
        if (valid) {
            for (unsigned j = 0; j < 12; ++j) payload[j] = frame[j + 1];
            payload[12] = frame[15]; payload[13] = frame[16];
            /* High 16 bits retain FIFO little endian; paired low nibbles
               occupy the previously reserved UART payload bytes. */
            for (unsigned j = 0; j < 3; ++j) payload[14 + j] = frame[17 + j];
            send_packet(1, imu_seq, read_us, payload);
        } else fault_flags |= 16;
        ++imu_seq; --imu_cached_count;
        fast_adc_poll(); tx_pump();
        if (adc_result_ready) break; /* Do not publish newer IMU timestamps first. */
    }
    imu_backlog = imu_cached_count != 0;
}
