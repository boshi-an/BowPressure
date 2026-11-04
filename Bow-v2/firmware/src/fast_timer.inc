/* TIM3 compare deadlines remain phase-locked at 500us. ISR never touches
 * shared SPI or the main-loop clock extender. Snapshot protects paired fields. */
static volatile uint32_t adc_timer_events, adc_timer_due;
static volatile uint32_t adc_next_deadline = 500;
void TIM3_IRQHandler(void) {
    if (!(TIM3->SR & TIM_SR_CC1IF)) return;
    TIM3->SR = ~TIM_SR_CC1IF;
    do {
        adc_timer_due = adc_next_deadline;
        ++adc_timer_events;
        adc_next_deadline += 500;
    } while ((int16_t)((uint16_t)adc_next_deadline - (uint16_t)TIM3->CNT) <= 0);
    TIM3->CCR1 = (uint16_t)adc_next_deadline;
}
static void adc_timer_snapshot(uint32_t *events, uint32_t *due) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    *events = adc_timer_events; *due = adc_timer_due;
    __set_PRIMASK(irq);
}
