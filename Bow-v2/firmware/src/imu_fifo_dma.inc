/* DMA1 ch1 owns UART TX. RM0490 DMAMUX requests 16/17 are SPI1 RX/TX. */
enum {
    IMU_FIFO_SPI_BR = 2, /* PCLK / 8 = 6 MHz for FIFO bursts only. */
    IMU_FIFO_FRAME_BYTES = 20,
    IMU_FIFO_DMA_MAX_FRAMES = 8,
    IMU_FIFO_DMA_PADDING_US = 24,
};

typedef enum {
    IMU_FIFO_DMA_BUSY,
    IMU_FIFO_DMA_DONE,
    IMU_FIFO_DMA_ERROR,
} ImuFifoDmaResult;

/* Clock a FIFO_DATA read command and the required dummy transmit bytes.
 * Received byte zero is the response during the command and is ignored. */
static const uint8_t imu_fifo_command[1 + IMU_FIFO_FRAME_BYTES * IMU_FIFO_DMA_MAX_FRAMES] = {0x94};
static uint16_t imu_dma_started_tick, imu_dma_budget_us;

static unsigned imu_fifo_dma_budget_us(unsigned payload_bytes) {
    /* A byte at 6 MHz takes 4/3 us. Round up to 11/8 us per byte so
     * checking the budget needs no integer division on Cortex-M0+. */
    return ((1 + payload_bytes) * 11 + 7) / 8 + IMU_FIFO_DMA_PADDING_US;
}

static void imu_fifo_dma_init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    (void)RCC->AHBENR;
    DMA1_Channel2->CCR = 0;
    DMA1_Channel3->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    DMAMUX1_Channel1->CCR = 16;
    DMAMUX1_Channel2->CCR = 17;
    DMA1_Channel2->CPAR = (uintptr_t)&SPI1->DR;
    DMA1_Channel3->CPAR = (uintptr_t)&SPI1->DR;
    DMA1_Channel3->CMAR = (uintptr_t)imu_fifo_command;
}

static bool imu_fifo_dma_start(uint8_t *rx, unsigned payload_bytes) {
    if (!payload_bytes || payload_bytes >= sizeof(imu_fifo_command) ||
        imu_dma_budget_us || (SPI1->SR & (SPI_SR_BSY | SPI_SR_FRLVL | SPI_SR_FTLVL)))
        return false;

    imu_dma_started_tick = TIM3->CNT;
    imu_dma_budget_us = imu_fifo_dma_budget_us(payload_bytes);
    GPIOB->BSRR = IMU_CS;
    GPIOC->BSRR = ADC_CS;
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                (IMU_FIFO_SPI_BR << SPI_CR1_BR_Pos);
    DMA1_Channel2->CMAR = (uintptr_t)rx;
    DMA1_Channel2->CNDTR = 1 + payload_bytes;
    DMA1_Channel3->CNDTR = 1 + payload_bytes;
    SPI1->CR2 |= SPI_CR2_RXDMAEN;
    __DMB();
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_EN;
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_PL_0 | DMA_CCR_EN;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    GPIOB->BSRR = IMU_CS << 16;
    SPI1->CR1 |= SPI_CR1_SPE;
    return true;
}

static void imu_fifo_dma_stop_channels(void) {
    DMA1_Channel2->CCR = 0;
    DMA1_Channel3->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
    imu_dma_budget_us = 0;
}

static void imu_fifo_dma_abort(void) {
    imu_fifo_dma_stop_channels();
    GPIOB->BSRR = IMU_CS;
    /* The transfer may have stopped mid-byte. Reset only SPI1 to clear
     * queued shifter/FIFO state before the ADC uses the shared bus. */
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                (SPI_BR << SPI_CR1_BR_Pos);
    SPI1->CR2 = (7UL << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
}

static ImuFifoDmaResult imu_fifo_dma_poll(void) {
    const uint32_t complete = DMA_ISR_TCIF2 | DMA_ISR_TCIF3;
    uint32_t flags = DMA1->ISR;
    uint32_t status = SPI1->SR;
    if ((flags & (DMA_ISR_TEIF2 | DMA_ISR_TEIF3)) ||
        (status & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_FRE))) {
        imu_fifo_dma_abort();
        return IMU_FIFO_DMA_ERROR;
    }
    if ((flags & complete) == complete &&
        !(status & (SPI_SR_FRLVL | SPI_SR_FTLVL | SPI_SR_BSY))) {
        imu_fifo_dma_stop_channels();
        GPIOB->BSRR = IMU_CS;
        SPI1->CR1 &= ~SPI_CR1_SPE;
        SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        __DMB();
        return IMU_FIFO_DMA_DONE;
    }
    if ((uint16_t)(TIM3->CNT - imu_dma_started_tick) <= imu_dma_budget_us)
        return IMU_FIFO_DMA_BUSY;
    imu_fifo_dma_abort();
    return IMU_FIFO_DMA_ERROR;
}
