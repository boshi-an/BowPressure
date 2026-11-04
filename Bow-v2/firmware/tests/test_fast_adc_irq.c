/* Test production IRQ event handoff, capture reconstruction and level fallback. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define BOARD_REV 3
static struct { uint32_t FPR1,RPR1,IMR1,RTSR1,FTSR1,EXTICR[4]; } exti;
static struct { uint32_t CNT; } timer;
static struct { uint32_t IDR; } gpio;
#define EXTI (&exti)
#define TIM3 (&timer)
#define GPIOA (&gpio)
#define EXTI_EXTICR1_EXTI3 (255UL<<24)
#define EXTI2_3_IRQn 6
static uint32_t mask, now;
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask=1; }
static void __set_PRIMASK(uint32_t m) { mask=m; }
static void NVIC_ClearPendingIRQ(int n) { assert(n==6); }
static void NVIC_SetPriority(int n,int p) { assert(n==6 && p==0); }
static void NVIC_EnableIRQ(int n) { assert(n==6); }
static uint32_t clock_us(void) { return now; }
#include "../src/fast_adc_irq.inc"
static void edge(uint32_t t) {
    timer.CNT=t&65535;exti.FPR1=8;EXTI2_3_IRQHandler();
    exti.FPR1=0; /* Mock write-one-to-clear register. */
}
int main(void) {
    uint32_t ready;
    adc_drdy_init();gpio.IDR=8;
    assert(exti.IMR1&8);assert(exti.FTSR1&8);assert(!(exti.RTSR1&8));
    edge(100);assert(!adc_drdy_pending); /* Ignore before a START. */
    adc_drdy_prepare();assert(mask==0);
    assert(!adc_drdy_take(100,&ready));
    edge(483);edge(490);now=510;
    assert(adc_drdy_take(100,&ready)&&ready==483); /* First edge wins. */
    assert(!adc_drdy_pending&&!adc_drdy_armed);
    assert(!adc_drdy_take(100,&ready));
    adc_drdy_prepare();edge(65530);now=65560;
    assert(adc_drdy_take(65147,&ready)&&ready==65530);
    adc_drdy_prepare();edge(250);now=300;
    assert(adc_drdy_take(UINT32_MAX-132,&ready)&&ready==250);
    adc_drdy_prepare();gpio.IDR=0;now=900;
    assert(adc_drdy_take(510,&ready)&&ready==900); /* Missing edge fallback. */
    gpio.IDR=8;adc_drdy_prepare();edge(900);now=1400;
    assert(adc_drdy_take(1000,&ready)&&ready==1400); /* Reject stale timestamp. */
    mask=1;adc_drdy_prepare();assert(mask==1);
    edge(1783);now=1800;assert(adc_drdy_take(1400,&ready)&&mask==1);
    return 0;
}
