/* Main-loop-owned ring; DMA owns only the active contiguous span.
 * STM32C011 RM0490: DMAMUX channel0 -> DMA1 channel1, USART1_TX request51.
 * Completion is polled, so neither byte nor DMA interrupts are required. */
enum { TX_SIZE = 2048, TX_MASK = TX_SIZE - 1, PACKET_SIZE = 32 };
static uint8_t tx_ring[TX_SIZE];
static unsigned tx_in, tx_out, tx_active;
static uint32_t tx_drops, tx_errors;
static void tx_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* Also clocks DMAMUX on STM32C011. */
    (void)RCC->AHBENR;
    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMAMUX1_Channel0->CCR = 51;
    DMA1_Channel1->CPAR = (uintptr_t)&USART1->TDR;
    USART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    USART1->CR3 |= USART_CR3_DMAT;
}
static void tx_pump(void) {
    if (tx_active) {
        uint32_t flags = DMA1->ISR;
        if (!(flags & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1))) return;
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF1;
        if (flags & DMA_ISR_TEIF1) {
            ++tx_errors;
            /* Abandon the affected whole span; partial bytes are rejected by
               the receiver CRC. Conservative count, includes frames already sent. */
            tx_drops += tx_active / PACKET_SIZE;
        }
        tx_out = (tx_out + tx_active) & TX_MASK;
        tx_active = 0;
    }
    if (tx_out == tx_in) return;
    tx_active = tx_in > tx_out ? tx_in - tx_out : TX_SIZE - tx_out;
    DMA1_Channel1->CMAR = (uintptr_t)(tx_ring + tx_out);
    DMA1_Channel1->CNDTR = tx_active;
    __DMB();
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_EN;
}
static void send_packet(uint8_t type, uint32_t seq, uint32_t time_us, const uint8_t *data) {
    tx_pump();
    if (((tx_out - tx_in - 1) & TX_MASK) < PACKET_SIZE) { ++tx_drops; return; }
    /* Every record and wrap boundary is aligned to 32 bytes. Encode directly
       into free ring memory; do not copy a stack packet a second time. */
    uint8_t *p = tx_ring + tx_in;
    p[0] = 0xA5; p[1] = 0x5A; p[2] = 2; p[3] = type;
    pack32(p + 4, seq); pack32(p + 8, time_us);
    for (unsigned i = 0; i < 18; ++i) p[12 + i] = data[i];
    pack16(p + 30, fast_crc(p, 30));
    __DMB();
    tx_in = (tx_in + PACKET_SIZE) & TX_MASK;
    tx_pump();
}
