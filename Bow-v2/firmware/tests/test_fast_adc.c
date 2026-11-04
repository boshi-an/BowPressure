/* Exercise production ADC scheduler with simulated clock and SPI. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "../src/fast_protocol.h"
static struct { uint32_t IDR; } gpio;
#define GPIOA (&gpio)
static bool adc_alive = true;
static uint32_t fault_flags, now;
static char events[64];
static unsigned n, sent;
static bool fail_start;
static uint8_t last_payload[18];
static uint32_t last_read;
static uint32_t clock_us(void) { return now; }
static uint32_t timer_events, timer_due;
static void adc_timer_snapshot(uint32_t *events, uint32_t *due) {
    *events=timer_events;*due=timer_due;
}
static void fire(uint32_t deadline) {
    now=timer_due=deadline;++timer_events;
}
static bool adc_wr(uint8_t reg, uint8_t value) {
    (void)value;
    events[n++] = reg == 4 ? 'S' : 'M';
    now += 5;
    return !(fail_start && reg == 4);
}
static bool adc_frame(const uint8_t *tx, uint8_t *rx, unsigned size) {
    (void)tx; assert(size == 5);
    events[n++] = 'R'; now += 10;
    uint8_t data[5] = {0xFF,0x60,0xFF,0xFF,0xFE};
    memcpy(rx, data, 5); return true;
}
static void send_packet(uint8_t type, uint32_t seq, uint32_t time_us, const uint8_t *p) {
    assert(type == 2); assert(seq == sent++);
    events[n++] = 'P'; memcpy(last_payload,p,18); last_read = time_us;
    now += 70; /* Even an expensive CRC must occur AFTER next START. */
}
#include "../src/fast_adc.inc"
static void reset(void) {
    n=sent=now=fault_flags=channel=adc_events_used=start_adc=adc_seq=late_slots=max_late=0;
    timer_events=timer_due=0;
    pending=adc_result_ready=fail_start=false;adc_alive=true;gpio.IDR=8;
    memset(events,0,sizeof events);
}
int main(void) {
    reset();fast_adc_poll();assert(!pending); /* No timer event, no START. */
    fire(500);fast_adc_poll();assert(pending && start_adc==510);
    now=890;gpio.IDR=0;fast_adc_poll();
    assert(adc_result_ready && !pending && sent==0);
    fire(1000);fast_adc_poll();
    assert(!adc_result_ready && pending && start_adc==1010);
    assert(n==6 && memcmp(events,"MSRMSP",6)==0);
    assert(last_read==890 && last_payload[6]==0 && last_payload[7]==32);
    assert(last_payload[0]==254 && last_payload[3]==255);
    now=1390;fast_adc_poll();fire(1500);fast_adc_poll();
    assert(sent==2 && last_payload[6]==1 && late_slots==0);
    now=1890;fast_adc_poll();fail_start=true;fire(2000);fast_adc_poll();
    assert(!adc_alive && fault_flags==1 && sent==3 && !adc_result_ready);
    reset();fire(500);fast_adc_poll();now=20511;fast_adc_poll();
    assert(!adc_alive && fault_flags==1 && sent==0);
    reset();fire(UINT32_MAX-100);fast_adc_poll();
    now+=383;gpio.IDR=0;fast_adc_poll();fire(timer_due+500);fast_adc_poll();
    assert(sent==1 && pending && late_slots==0);
    reset();fire(500);fire(1000);fire(1500);now=1520;fast_adc_poll();
    assert(late_slots==2 && max_late==20 && adc_events_used==3);
    assert(fast_crc((const uint8_t *)"123456789",9)==0x29B1);
    return 0;
}
