#include "ltc_decode.h"
/* EXTI owns producer, main owns consumer. Keep ISR short and never touch SPI. */
enum { LTC_RING = 128, LTC_MASK = LTC_RING - 1 };
static volatile uint16_t ltc_intervals[LTC_RING], ltc_edge_ticks[LTC_RING], ltc_edge_ms[LTC_RING];
static volatile unsigned ltc_in, ltc_out;
static volatile uint32_t ltc_edges, ltc_overflows;
static LtcDecoder ltc_decoder;
static uint32_t ltc_seq, ltc_last_good, ltc_next_report, ltc_seen_overflows;
void EXTI0_1_IRQHandler(void) {
    uint32_t rising=EXTI->RPR1 & 1, falling=EXTI->FPR1 & 1;
    if (!(rising | falling)) return;
    uint16_t now = TIM3->CNT;
    uint32_t ms = ticks;
    EXTI->RPR1 = 1; EXTI->FPR1 = 1;
    static uint16_t previous;
    static uint32_t previous_ms;
    static bool have_previous;
    unsigned dt = (uint16_t)(now - previous);
    if (!have_previous || ms - previous_ms > 2 || (rising && falling)) dt = 0;
    have_previous = true; previous = now; previous_ms = ms;
    ++ltc_edges;
    unsigned next = (ltc_in + 1) & LTC_MASK;
    if (next == ltc_out) { ++ltc_overflows; return; }
    ltc_intervals[ltc_in] = dt;
    ltc_edge_ticks[ltc_in] = now;
    ltc_edge_ms[ltc_in] = (uint16_t)ms;
    __DMB(); ltc_in = next;
}
static void ltc_init(void) {
    gpio_mode(GPIOA, 0, 0);
    GPIOA->PUPDR &= ~3UL;
    EXTI->EXTICR[0] &= ~EXTI_EXTICR1_EXTI0;
    EXTI->RPR1 = 1; EXTI->FPR1 = 1;
#ifdef DISABLE_LTC_IRQ
    /* Long-cable A/B test: leave PA0 electrically unchanged (input, no pull).
       Mask only LTC EXTI0; IMU EXTI6 and ADC TIM3 remain active. */
    EXTI->IMR1 &= ~1UL;
    EXTI->RTSR1 &= ~1UL; EXTI->FTSR1 &= ~1UL;
    NVIC_DisableIRQ(EXTI0_1_IRQn);
    NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
#else
    EXTI->RTSR1 |= 1; EXTI->FTSR1 |= 1; EXTI->IMR1 |= 1;
    NVIC_SetPriority(EXTI0_1_IRQn, 1); /* TIM3 ADC scheduler remains higher. */
    NVIC_EnableIRQ(EXTI0_1_IRQn);
#endif
}
static void ltc_send(uint32_t now, bool locked) {
    uint8_t p[18] = {0};
    for (unsigned j=0; j<4; ++j) p[j] = ltc_decoder.tc[j];
    p[4]=25; p[5]=locked;
    pack32(p+6, ltc_edges); pack32(p+10, ltc_decoder.errors);
    pack32(p+14, ltc_overflows);
    send_packet(4, ltc_seq++, now, p);
    ltc_next_report = now + 1000000;
}
/* A complete BMC bit is decoded at its END boundary. The final sync bit's
 * ending edge therefore marks the START of the NEXT 25fps frame. Keep the
 * packet header at main-loop time to preserve transport timestamp ordering. */
static void ltc_send_anchor(uint32_t now, uint16_t tick) {
    static uint32_t seq;
    uint8_t p[18] = {0};
    for (unsigned j=0; j<4; ++j) p[j]=ltc_decoder.tc[j];
    p[4]=25; p[5]=1; /* forward frame-end anchor, not frame-start */
    pack32(p+6, now-(uint16_t)((uint16_t)now-tick));
    send_packet(6, seq++, now, p);
}
static void fast_ltc_poll(void) {
    if (ltc_seen_overflows != ltc_overflows) {
        uint32_t irq=__get_PRIMASK(); __disable_irq();
        ltc_seen_overflows=ltc_overflows; ltc_out=ltc_in;
        __set_PRIMASK(irq); ltc_reset(&ltc_decoder);
    }
    /* Recheck ADC deadline for every edge, including frame validation work. */
    for (unsigned n=0; n<8 && ltc_out!=ltc_in && background_allowed(); ++n) {
        __DMB(); unsigned dt=ltc_intervals[ltc_out];
        uint16_t edge_tick=ltc_edge_ticks[ltc_out];
        uint16_t edge_ms=ltc_edge_ms[ltc_out];
        ltc_out=(ltc_out+1)&LTC_MASK;
        if ((uint16_t)((uint16_t)ticks-edge_ms)>50) { ltc_reset(&ltc_decoder); continue; }
        if (ltc_interval(&ltc_decoder, dt)) {
            ltc_last_good=clock_us(); ltc_send(ltc_last_good, true);
            ltc_send_anchor(ltc_last_good, edge_tick);
        }
    }
    uint32_t now=clock_us();
    if (ltc_decoder.locked && now-ltc_last_good>120000) ltc_reset(&ltc_decoder);
    if (background_allowed() && (int32_t)(now-ltc_next_report)>=0)
        ltc_send(now, ltc_decoder.locked);
}
