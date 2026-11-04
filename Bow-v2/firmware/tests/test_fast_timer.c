#include <assert.h>
#include <stdint.h>
static struct { uint32_t SR,CNT,CCR1; } tim;
#define TIM3 (&tim)
#define TIM_SR_CC1IF 2U
static unsigned irq_mask;
static unsigned __get_PRIMASK(void) { return irq_mask; }
static void __disable_irq(void) { irq_mask=1; }
static void __set_PRIMASK(unsigned value) { irq_mask=value; }
#include "../src/fast_timer.inc"
static void tick(uint16_t counter) {
    tim.CNT=counter;tim.SR=TIM_SR_CC1IF;TIM3_IRQHandler();
}
int main(void) {
    tick(500);assert(adc_timer_events==1 && adc_timer_due==500 && tim.CCR1==1000);
    tick(1760);assert(adc_timer_events==3 && adc_timer_due==1500 && tim.CCR1==2000);
    adc_next_deadline=65500;tick(65500);assert(tim.CCR1==464 && adc_timer_due==65500);
    tick(464);assert(adc_timer_due==66000 && tim.CCR1==964);
    adc_next_deadline=UINT32_MAX-255;tick(65280);
    assert(adc_next_deadline==244 && tim.CCR1==244);
    tick(244);assert(adc_timer_due==244 && tim.CCR1==744);
    uint32_t events,due;irq_mask=1;adc_timer_snapshot(&events,&due);
    assert(events==adc_timer_events && due==244 && irq_mask==1);
    return 0;
}
