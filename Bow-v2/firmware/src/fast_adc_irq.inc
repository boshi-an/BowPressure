/* V3: PA3 / ADC DRDY falling edge. No SPI or clock_us() in the ISR.
 * Only the main loop owns the 32-bit clock extender and shared SPI bus. */
#if BOARD_REV >= 3
static volatile bool adc_drdy_pending, adc_drdy_armed;
static volatile uint16_t adc_drdy_tick;
#endif
void EXTI2_3_IRQHandler(void) {
#if BOARD_REV >= 3
    if (EXTI->FPR1 & (1UL << 3)) {
        uint16_t tick = TIM3->CNT;
        EXTI->FPR1 = 1UL << 3;
        if (adc_drdy_armed && !adc_drdy_pending) {
            adc_drdy_tick = tick;
            adc_drdy_pending = true;
        }
    }
#endif
}
static void adc_drdy_init(void) {
#if BOARD_REV >= 3
    EXTI->IMR1 &= ~(1UL << 3);
    EXTI->EXTICR[0] &= ~EXTI_EXTICR1_EXTI3; /* PA3 */
    EXTI->RTSR1 &= ~(1UL << 3);
    EXTI->FTSR1 |= 1UL << 3;
    EXTI->RPR1 = 1UL << 3; EXTI->FPR1 = 1UL << 3;
    adc_drdy_pending = adc_drdy_armed = false;
    NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
    NVIC_SetPriority(EXTI2_3_IRQn, 0); /* Same as TIM3; above LTC and IMU. */
    EXTI->IMR1 |= 1UL << 3;
    NVIC_EnableIRQ(EXTI2_3_IRQn);
#endif
}
static void adc_drdy_prepare(void) {
#if BOARD_REV >= 3
    /* Clear only before START, never after it: do not erase a fresh edge. */
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    adc_drdy_pending = false;
    EXTI->FPR1 = 1UL << 3;
    adc_drdy_armed = true;
    __set_PRIMASK(irq);
#endif
}
static bool adc_drdy_take(uint32_t start, uint32_t *ready_us) {
#if BOARD_REV >= 3
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    bool edge = adc_drdy_pending;
    uint16_t tick = adc_drdy_tick;
    bool low = !(GPIOA->IDR & (1UL << 3));
    if (edge || low) {
        adc_drdy_pending = false;
        adc_drdy_armed = false;
    }
    __set_PRIMASK(irq);
    if (!edge && !low) return false;
    uint32_t now = clock_us();
    /* The timeout is < one 16-bit timer wrap. Reject stale/pre-START captures. */
    uint32_t captured = now - (uint16_t)((uint16_t)now - tick);
    *ready_us = edge && (uint32_t)(captured - start) <= 20000 ? captured : now;
    return true;
#else
    (void)start;
    if (GPIOA->IDR & (1UL << 3)) return false;
    *ready_us = clock_us();
    return true;
#endif
}
