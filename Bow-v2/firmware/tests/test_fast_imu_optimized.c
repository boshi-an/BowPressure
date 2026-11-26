#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "../src/fast_protocol.h"

#define BOW_OPTIMIZED
enum { IMU_FIFO_CAPACITY = 8192 / 20, IMU_GUARD_US = 200 };
enum { IMU_FIFO_DMA_MAX_FRAMES = 8, IMU_FIFO_FRAME_BYTES = 20 };
typedef enum { IMU_FIFO_DMA_BUSY, IMU_FIFO_DMA_DONE, IMU_FIFO_DMA_ERROR } ImuFifoDmaResult;

static struct { uint32_t IDR; } gpio;
#define GPIOA (&gpio)
static bool imu_alive = true, imu_backlog;
static uint32_t fault_flags, imu_seq, imu_irq_count;
static uint32_t now, time_left = 300;
static unsigned count_reads, pending_count = 9, starts, aborted, emits;
static unsigned dma_bytes, fifo_read;
static uint8_t status, *dma_dest;
static ImuFifoDmaResult dma_result;
static uint32_t last_sent_us;
static unsigned last_sent_seq;
static uint8_t last_payload[18];

static uint32_t background_time_left_us(void) { return time_left; }
static bool background_allowed(void) { return background_time_left_us() > IMU_GUARD_US; }
static uint32_t clock_us(void) { return ++now; }
static void imu_service_begin(uint32_t observed) { (void)observed; imu_backlog = false; }
static bool rd(uint8_t addr, uint8_t *out) {
    assert(addr == 0x19);
    *out = status;
    return true;
}
static bool imu_transfer(uint8_t addr, uint8_t *out, unsigned n, bool read) {
    assert(addr == 0x12 && n == 2 && read);
    unsigned count = (++count_reads & 1) ? 0xF000 : pending_count;
    pack16(out, count);
    return true;
}
/* Convenient mock budget: one microsecond per byte and a 10 us margin. */
static unsigned imu_fifo_dma_budget_us(unsigned bytes) { return bytes + 10; }
static bool imu_fifo_dma_start(uint8_t *rx, unsigned bytes) {
    assert(!dma_dest && bytes > 0 && bytes % IMU_FIFO_FRAME_BYTES == 0);
    ++starts;
    dma_dest = rx;
    dma_bytes = bytes;
    dma_result = IMU_FIFO_DMA_BUSY;
    rx[0] = 0xFF;
    for (unsigned offset = 1; offset <= bytes; offset += IMU_FIFO_FRAME_BYTES) {
        ++fifo_read;
        rx[offset] = 0x78;
        for (unsigned n = 1; n < IMU_FIFO_FRAME_BYTES; ++n) rx[offset+n] = fifo_read + n;
    }
    return true;
}
static ImuFifoDmaResult imu_fifo_dma_poll(void) {
    assert(dma_dest);
    if (dma_result != IMU_FIFO_DMA_BUSY) dma_dest = NULL;
    return dma_result;
}
static void imu_fifo_dma_abort(void) {
    assert(dma_dest);
    dma_dest = NULL;
    ++aborted;
}
static void send_packet(uint8_t type, uint32_t seq, uint32_t stamp, const uint8_t *payload) {
    assert(stamp >= last_sent_us);
    last_sent_us = stamp;
    last_sent_seq = seq;
    memcpy(last_payload, payload, sizeof last_payload);
    if (type == 1) ++emits;
}
#include "../src/fast_imu.inc"

int main(void) {
    /* Use the second stable count and defer when even one DMA frame cannot fit. */
    time_left = 230;
    fast_imu_poll();
    assert(imu_alive && count_reads == 2 && imu_cached_count == 9 && imu_backlog && starts == 0);

    time_left = 290;
    fast_imu_poll();
    assert(starts == 1 && dma_bytes == 60 && imu_cached_count == 6 && emits == 0);
    assert(fast_imu_dma_service());
    fast_imu_poll();
    assert(count_reads == 2 && starts == 1 && emits == 0);

    /* Completion releases shared SPI without publishing ahead of a ready ADC. */
    dma_result = IMU_FIFO_DMA_DONE;
    assert(!fast_imu_dma_service() && !dma_dest && emits == 0);
    time_left = 0;
    fast_imu_poll();
    assert(emits == 0);
    last_sent_us = clock_us();  /* ADC packet sent before publishing staged IMU. */
    time_left = 1000;
    fast_imu_poll();
    assert(emits == 2 && last_sent_seq == 1 && starts == 1);
    for (unsigned j = 0; j < 12; ++j) assert(last_payload[j] == 2 + j + 1);
    for (unsigned j = 0; j < 5; ++j) assert(last_payload[12 + j] == 2 + 15 + j);
    assert(last_payload[17] == 0);
    fast_imu_poll();
    assert(emits == 3 && last_sent_seq == 2 && starts == 2 && dma_bytes == 120);
    assert(imu_cached_count == 0 && count_reads == 2 && imu_backlog);

    /* Unexpectedly long SPI work is stopped so the ADC regains ownership. */
    time_left = IMU_GUARD_US;
    assert(!fast_imu_dma_service() && aborted == 1 && !dma_dest);
    assert(!imu_alive && !imu_backlog && fault_flags == 8);

    imu_alive = true;
    fault_flags = 0;
    time_left = 1000;
    pending_count = 20;
    fast_imu_poll();
    assert(imu_alive && starts == 3 && dma_bytes == 8 * IMU_FIFO_FRAME_BYTES);
    assert(imu_cached_count == 12 && count_reads == 4);
    dma_result = IMU_FIFO_DMA_DONE;
    assert(!fast_imu_dma_service());
    /* Skip an exact invalid endpoint but keep the next valid packet's sequence. */
    imu_batch[1 + 1] = 0;
    imu_batch[1 + 2] = 0x80;
    imu_batch[1 + 17] &= 0x0F;
    unsigned second = 1 + IMU_FIFO_FRAME_BYTES;
    imu_batch[second + 1] = 0;
    imu_batch[second + 2] = 0x80;
    imu_batch[second + 17] |= 0x20;
    fast_imu_poll();
    assert(emits == 4 && last_sent_seq == 4 && fault_flags == 16 && imu_alive);

    /* Record full-queue diagnostics when the exact enlarged capacity is seen. */
    imu_batch_next = imu_batch_count;
    imu_cached_count = 0;
    pending_count = IMU_FIFO_CAPACITY;
    fast_imu_poll();
    assert(!imu_alive && (fault_flags & 4) && imu_peak_count == IMU_FIFO_CAPACITY);
    imu_send_diagnostics(clock_us());
    assert(last_payload[2] == (IMU_FIFO_CAPACITY & 0xFF));
    assert(last_payload[3] == (IMU_FIFO_CAPACITY >> 8));
    assert(fast_crc(last_payload, 0) == 0xFFFF);
    return 0;
}
