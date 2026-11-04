#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "../src/fast_protocol.h"
static struct {uint32_t IDR;} gpio;
#define GPIOA (&gpio)
static bool imu_alive=true, imu_backlog, adc_result_ready;
static uint32_t fault_flags,imu_seq,imu_irq_count;
enum {IMU_FIFO_CAPACITY=102};
static unsigned queries, reads, emits, pending_count=3, steps=100;
static uint8_t status;
static uint32_t now;
static bool background_allowed(void) { return steps && --steps; }
static uint32_t clock_us(void) { return ++now; }
static void imu_service_begin(uint32_t t) {(void)t;imu_backlog=false;}
static bool rd(uint8_t addr,uint8_t *out) {assert(addr==0x19);*out=status;return true;}
static bool imu_transfer(uint8_t addr,uint8_t *out,unsigned n,bool read) {
    assert(read);
    memset(out,0,n);
    if(addr==0x12) {++queries;out[0]=pending_count;steps=1;}
    else {assert(addr==0x14 && n==20);++reads;out[0]=0x78;}
    return true;
}
static void send_packet(uint8_t t,uint32_t seq,uint32_t us,const uint8_t *p) {
    (void)seq;(void)us;(void)p;if(t==1)++emits;
}
static void fast_adc_poll(void) { adc_result_ready=true; }
static void tx_pump(void) {}
#include "../src/fast_imu.inc"
static uint16_t bit_crc(const uint8_t *p,unsigned n) {
    uint16_t c=0xffff;
    while(n--) {c^=(uint16_t)*p++<<8;for(unsigned j=0;j<8;++j)c=(c&0x8000)?(c<<1)^0x1021:c<<1;}
    return c;
}
int main(void) {
    /* Query uses remaining time; defer frame but retain count across slots. */
    fast_imu_poll();assert(queries==1 && reads==0 && imu_cached_count==3 && imu_backlog);
    for(unsigned i=0;i<3;++i) {
        steps=100;adc_result_ready=false;fast_imu_poll();
        assert(queries==1 && reads==i+1 && emits==i+1);
    }
    assert(!imu_backlog && !imu_cached_count && imu_alive);
    pending_count=102;steps=100;fast_imu_poll();assert(!imu_alive && fault_flags==4);
    imu_send_diagnostics(clock_us());
    uint8_t data[256];for(unsigned i=0;i<256;++i)data[i]=i;
    for(unsigned n=0;n<=256;++n)assert(fast_crc(data,n)==bit_crc(data,n));
    assert(fast_crc((const uint8_t*)"123456789",9)==0x29b1);
    return 0;
}
