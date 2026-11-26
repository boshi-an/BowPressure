#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

static struct { uint32_t PUPDR; } gpio;
static struct { uint32_t CNT; } timer;
static struct { uint32_t RPR1, FPR1, IMR1, FTSR1, RTSR1, EXTICR[4]; } exti;
#define GPIOA (&gpio)
#define TIM3 (&timer)
#define EXTI (&exti)
#define EXTI_EXTICR1_EXTI0 7U
#define EXTI0_1_IRQn 5
#define BOW_OPTIMIZED
static uint32_t ticks, now_us;
static bool background_open;
static void __DMB(void) {}
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t value) { assert(value == 0); }
static void gpio_mode(void *port, unsigned pin, unsigned mode) {
    assert(port == GPIOA && pin == 0 && mode == 0);
}
static void NVIC_SetPriority(unsigned irq, unsigned priority) {
    assert(irq == EXTI0_1_IRQn && priority == 1);
}
static void NVIC_EnableIRQ(unsigned irq) { assert(irq == EXTI0_1_IRQn); }
static bool background_allowed(void) { return background_open; }
static uint32_t clock_us(void) { return now_us; }
static void pack32(uint8_t *p, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) p[i] = value >> (8 * i);
}
static void send_packet(uint8_t type, uint32_t seq, uint32_t time_us, const uint8_t *payload) {
    (void)seq;
    assert(type == 4 && time_us == now_us && payload[4] == 25);
}

#include "../src/fast_ltc.inc"

static void advance(unsigned elapsed) {
    now_us += elapsed;
    ticks = now_us / 1000;
    timer.CNT = (uint16_t)now_us;
}

static void edge(unsigned elapsed, uint32_t rising, uint32_t falling) {
    advance(elapsed);
    exti.RPR1 = rising;
    exti.FPR1 = falling;
    EXTI0_1_IRQHandler();
    /* Mock registers record the write-one-to-clear masks. */
    assert(exti.RPR1 == (rising & 1) && exti.FPR1 == (falling & 1));
}

int main(void) {
    ltc_init();
    assert((exti.IMR1 & exti.RTSR1 & exti.FTSR1) == 1);
    advance(32767900);
    edge(0, 1, 0);
    edge(500, 0, 1); /* Both the timer tick and packed millisecond clock wrap. */
    fast_ltc_poll();
    assert(ltc_out != ltc_in);
    background_open = true;
    fast_ltc_poll();
    assert(ltc_out == ltc_in && ltc_decoder.count == 1 && ltc_decoder.errors == 1);

    edge(250, 1, 0);
    edge(250, 0, 1);
    fast_ltc_poll();
    assert(ltc_decoder.count == 2 && ltc_decoder.errors == 1);
    edge(500, 1, 1); /* Coalesced flags break the current bit sequence. */
    fast_ltc_poll();
    assert(ltc_decoder.count == 0 && ltc_decoder.errors == 2);

    for (unsigned i = 0; i < 5; ++i) edge(500, 1, 0);
    fast_ltc_poll();
    assert(ltc_decoder.count == 4 && ltc_out != ltc_in);
    fast_ltc_poll();
    assert(ltc_decoder.count == 5 && ltc_out == ltc_in);
    edge(500, 0, 1);
    advance(51000);
    fast_ltc_poll();
    assert(!ltc_have_previous && ltc_decoder.count == 0);

    for (unsigned i = 0; i < LTC_RING; ++i) edge(250, 1, 0);
    assert(ltc_overflows == 1);
    fast_ltc_poll();
    assert(ltc_out == ltc_in && !ltc_have_previous);
    return 0;
}
