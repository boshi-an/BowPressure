/* V3 INT1 is U2.4 -> U3.13 / PA6. ISR never accesses shared SPI.
 * A level check and periodic poll recover missed edges or disconnected INT1. */
static volatile bool imu_irq_pending;
static volatile uint32_t imu_irq_count;
static uint32_t imu_check_us;
static bool imu_backlog;
void EXTI4_15_IRQHandler(void) {
    if (EXTI->RPR1 & (1UL << 6)) {
        EXTI->RPR1 = 1UL << 6;
        imu_irq_pending = true;
        ++imu_irq_count;
    }
}
static void imu_irq_init(void) {
#if BOARD_REV >= 3
    gpio_mode(GPIOA, 6, 0);
    EXTI->EXTICR[1] &= ~EXTI_EXTICR2_EXTI6;
    EXTI->IMR1 &= ~(1UL << 6);
    EXTI->FTSR1 &= ~(1UL << 6);
    EXTI->RTSR1 |= 1UL << 6;
    EXTI->RPR1 = 1UL << 6; EXTI->FPR1 = 1UL << 6;
    NVIC_SetPriority(EXTI4_15_IRQn, 2); /* Below ADC timer and LTC. */
    EXTI->IMR1 |= 1UL << 6;
    NVIC_EnableIRQ(EXTI4_15_IRQn);
#endif
}
static bool imu_service_due(uint32_t now) {
#if BOARD_REV >= 3
    return imu_backlog || imu_irq_pending || (GPIOA->IDR & (1UL << 6)) ||
           (uint32_t)(now - imu_check_us) >= 5000;
#else
    return imu_backlog || (uint32_t)(now - imu_check_us) >= 200;
#endif
}
static void imu_service_begin(uint32_t now) {
    /* Clear before SPI; an interrupt arriving during the read remains pending. */
    imu_irq_pending = false;
    imu_backlog = false;
    imu_check_us = now;
}
static void status_led_poll(bool error) {
#if BOARD_REV >= 3
    /* Avoid software division on every acquisition-loop iteration (Cortex-M0+).
       Update at most every 10ms; ADC/SPI retain the rest of the CPU budget. */
    static bool initialized, previous_error;
    static uint32_t last_update;
    uint32_t now = ticks;
    if (initialized && error == previous_error && (uint32_t)(now - last_update) < 10) return;
    initialized = true; previous_error = error; last_update = now;
    /* Healthy: short heartbeat each second. Fault: 5Hz. No delay or PWM. */
    bool on = error ? (now % 200 < 100) : (now % 1000 < 60);
    GPIOA->BSRR = on ? (1UL << 20) : (1UL << 4);
#else
    (void)error;
#endif
}
