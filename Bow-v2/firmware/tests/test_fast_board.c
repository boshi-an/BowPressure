#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#ifndef BOARD_REV
#define BOARD_REV 3
#endif
static struct { uint32_t IDR, BSRR; } gpio;
static struct { uint32_t RPR1,FPR1,IMR1,FTSR1,RTSR1,EXTICR[4]; } exti;
#define GPIOA (&gpio)
#define EXTI (&exti)
#define EXTI_EXTICR2_EXTI6 (7UL<<16)
#define EXTI4_15_IRQn 7
static uint32_t ticks;
static unsigned enabled, priority;
static void gpio_mode(void *p,unsigned pin,unsigned mode) { (void)p;assert(pin==6 && mode==0); }
static void NVIC_SetPriority(unsigned irq,unsigned p) { assert(irq==7);priority=p; }
static void NVIC_EnableIRQ(unsigned irq) { assert(irq==7);enabled=1; }
#include "../src/fast_board.inc"
int main(void) {
    imu_irq_init();assert(enabled && priority==2 && (exti.IMR1&64));
    assert((exti.RTSR1&64) && !(exti.FTSR1&64));
    assert(!imu_service_due(4999));assert(imu_service_due(5000));
    imu_service_begin(5000);assert(!imu_service_due(5001));
    exti.RPR1=64;EXTI4_15_IRQHandler();assert(imu_irq_count==1 && imu_service_due(5001));
    imu_service_begin(5001);
    /* A fresh edge during SPI must survive; the end of service must not clear it. */
    exti.RPR1=64;EXTI4_15_IRQHandler();assert(imu_irq_pending && imu_irq_count==2);
    imu_service_begin(6000);gpio.IDR=64;assert(imu_service_due(6001));
    gpio.IDR=0;imu_backlog=true;assert(imu_service_due(6001));
    imu_service_begin(UINT32_MAX-999);assert(!imu_service_due(3999));assert(imu_service_due(4000));
    ticks=0;status_led_poll(false);assert(gpio.BSRR==(1UL<<20));
    ticks=60;status_led_poll(false);assert(gpio.BSRR==(1UL<<4));
    ticks=250;status_led_poll(true);assert(gpio.BSRR==(1UL<<20));
    ticks=350;status_led_poll(true);assert(gpio.BSRR==(1UL<<4));
    return 0;
}
