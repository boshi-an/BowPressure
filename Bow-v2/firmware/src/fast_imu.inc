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
#ifdef BOW_OPTIMIZED
/* Stage whole FIFO frames in the DMA destination across ADC service windows.
 * Offset zero receives the dummy byte clocked during the SPI register address. */
static uint8_t imu_batch[1 + IMU_FIFO_DMA_MAX_FRAMES * IMU_FIFO_FRAME_BYTES];
static unsigned imu_dma_frames, imu_batch_next, imu_batch_count;

/* Retire SPI before the ADC can use it. This does not publish IMU packets,
 * which stay behind any pending ADC packet in the wire timestamp order. */
static bool fast_imu_dma_service(void) {
    if (!imu_dma_frames) return false;
    ImuFifoDmaResult result = imu_fifo_dma_poll();
    if (result == IMU_FIFO_DMA_BUSY) {
        if (background_time_left_us() > IMU_GUARD_US) return true;
        imu_fifo_dma_abort();
    }
    if (result == IMU_FIFO_DMA_DONE) {
        imu_batch_next = 0;
        imu_batch_count = imu_dma_frames;
        last_imu_us = clock_us();
    } else {
        imu_backlog = false;
        imu_alive = false;
        fault_flags |= 8;
    }
    imu_dma_frames = 0;
    return false;
}

static bool imu_read_fifo_count(void) {
    imu_service_begin(clock_us());
    uint8_t status, count_data[2];
    /* ICM-45686 erratum AN-000364: use the second FIFO_COUNT reading. */
    if (!rd(0x19, &status) || !imu_transfer(0x12, count_data, 2, true) ||
        !imu_transfer(0x12, count_data, 2, true)) {
        imu_alive = false;
        fault_flags |= 2;
        return false;
    }
    unsigned count = count_data[0] | ((unsigned)count_data[1] << 8);
    imu_last_status = status;
    imu_last_count = count;
    if (count > imu_peak_count) imu_peak_count = count;
    ++imu_poll_count;
    imu_observed_us = clock_us();
    if ((status & 1) || count >= IMU_FIFO_CAPACITY) {
        imu_alive = false;
        fault_flags |= 4;
        return false;
    }
    if (!count && (uint32_t)(imu_observed_us - last_imu_us) > 100000) {
        imu_alive = false;
        fault_flags |= 32;
        return false;
    }
    imu_cached_count = count;
    imu_backlog = count != 0;
    return count != 0;
}

static bool imu_emit_frame(uint8_t *frame) {
    if ((frame[0] & 0xFC) != 0x78) {
        imu_backlog = false;
        imu_alive = false;
        fault_flags |= 8;
        return false;
    }
    uint32_t seq = imu_seq++;
    for (unsigned axis = 0; axis < 3; ++axis) {
        uint8_t low = frame[17 + axis];
        unsigned j = 1 + 2 * axis;
        /* Only the exact high-resolution sentinel 0x80000 is invalid. */
        if ((frame[j] == 0 && frame[j + 1] == 0x80 && !(low & 0xF0)) ||
            (frame[j + 6] == 0 && frame[j + 7] == 0x80 && !(low & 0x0F))) {
            fault_flags |= 16;
            return true;
        }
    }
    /* Remove the unused temperature in place. ADC and LTC can send between
     * batches, so use the emission time for the monotonically ordered packet
     * header. The original IMU sample timestamp moves into payload bytes 12–13. */
    for (unsigned j = 13; j < 18; ++j) frame[j] = frame[j + 2];
    frame[18] = 0;
    send_packet(1, seq, clock_us(), frame + 1);
    return true;
}

static void fast_imu_poll(void) {
    if (imu_dma_frames) return;
    /* Allow LTC and status work between groups of frames. Each queued DMA
     * frame is packed once, only while the ADC deadline guard is open. */
    for (unsigned n = 0; n < 2 && imu_batch_next < imu_batch_count && background_allowed(); ++n) {
        uint8_t *frame = imu_batch + 1 + imu_batch_next++ * IMU_FIFO_FRAME_BYTES;
        if (!imu_emit_frame(frame)) return;
    }
    imu_backlog = imu_batch_next < imu_batch_count || imu_cached_count != 0;
    if (imu_batch_next < imu_batch_count || !background_allowed()) return;
    if (!imu_cached_count && !imu_read_fifo_count()) return;

    uint32_t time_left = background_time_left_us();
    if (time_left <= IMU_GUARD_US + imu_fifo_dma_budget_us(IMU_FIFO_FRAME_BYTES)) return;
    unsigned frames = imu_cached_count;
    if (frames > IMU_FIFO_DMA_MAX_FRAMES) frames = IMU_FIFO_DMA_MAX_FRAMES;
    while (time_left <= IMU_GUARD_US + imu_fifo_dma_budget_us(frames * IMU_FIFO_FRAME_BYTES)) {
        --frames;
    }
    if (!imu_fifo_dma_start(imu_batch, frames * IMU_FIFO_FRAME_BYTES)) {
        imu_backlog = false;
        imu_alive = false;
        fault_flags |= 8;
        return;
    }
    imu_cached_count -= frames;
    imu_dma_frames = frames;
}
#else
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
#endif
