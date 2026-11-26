/* Verify DMA ownership and timeout paths against mock memory-mapped registers. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

static struct { uint32_t AHBENR, APBRSTR2; } rcc;
static struct { uint32_t CCR, CNDTR; uintptr_t CPAR, CMAR; } dma_rx, dma_tx;
static struct { uint32_t ISR, IFCR; } dma;
static struct { uint32_t CCR; } mux_rx, mux_tx;
static struct { uint32_t DR, SR, CR1, CR2; } spi;
static struct { uint32_t BSRR; } gpio_b, gpio_c;
static struct { uint16_t CNT; } tim3;
#define RCC (&rcc)
#define DMA1_Channel2 (&dma_rx)
#define DMA1_Channel3 (&dma_tx)
#define DMA1 (&dma)
#define DMAMUX1_Channel1 (&mux_rx)
#define DMAMUX1_Channel2 (&mux_tx)
#define SPI1 (&spi)
#define GPIOB (&gpio_b)
#define GPIOC (&gpio_c)
#define TIM3 (&tim3)
#define RCC_AHBENR_DMA1EN 1
#define RCC_APBRSTR2_SPI1RST (1UL << 12)
#define DMA_IFCR_CGIF2 (1UL << 4)
#define DMA_IFCR_CGIF3 (1UL << 8)
#define DMA_ISR_TCIF2 (1UL << 5)
#define DMA_ISR_TEIF2 (1UL << 7)
#define DMA_ISR_TCIF3 (1UL << 9)
#define DMA_ISR_TEIF3 (1UL << 11)
#define DMA_CCR_EN 1
#define DMA_CCR_DIR (1UL << 4)
#define DMA_CCR_MINC (1UL << 7)
#define DMA_CCR_PL_0 (1UL << 12)
#define DMA_CCR_PL_1 (1UL << 13)
#define SPI_CR1_MSTR (1UL << 2)
#define SPI_CR1_BR_Pos 3
#define SPI_CR1_SPE (1UL << 6)
#define SPI_CR1_SSI (1UL << 8)
#define SPI_CR1_SSM (1UL << 9)
#define SPI_CR2_RXDMAEN 1
#define SPI_CR2_TXDMAEN 2
#define SPI_CR2_DS_Pos 8
#define SPI_CR2_FRXTH (1UL << 12)
#define SPI_SR_MODF (1UL << 5)
#define SPI_SR_OVR (1UL << 6)
#define SPI_SR_BSY (1UL << 7)
#define SPI_SR_FRE (1UL << 8)
#define SPI_SR_FRLVL (3UL << 9)
#define SPI_SR_FTLVL (3UL << 11)
#define SPI_BR 3
#define IMU_CS (1UL << 7)
#define ADC_CS (1UL << 15)
#define __DMB() ((void)0)
#include "../src/imu_fifo_dma.inc"

int main(void) {
    const unsigned dma_requests = SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    const unsigned transfer_done = DMA_ISR_TCIF2 | DMA_ISR_TCIF3;
    uint8_t rx[1 + IMU_FIFO_FRAME_BYTES * IMU_FIFO_DMA_MAX_FRAMES];
    bool started;

    imu_fifo_dma_init();
    assert(rcc.AHBENR == RCC_AHBENR_DMA1EN && mux_rx.CCR == 16 && mux_tx.CCR == 17);
    assert(dma_rx.CPAR == (uintptr_t)&spi.DR && dma_tx.CPAR == (uintptr_t)&spi.DR);
    assert(imu_fifo_command[0] == 0x94 && imu_fifo_command[sizeof(rx) - 1] == 0);
    assert(imu_fifo_dma_budget_us(IMU_FIFO_FRAME_BYTES) == 53);
    assert(imu_fifo_dma_budget_us(sizeof(rx) - 1) == 246);
    assert(!imu_fifo_dma_start(rx, 0) && !imu_fifo_dma_start(rx, sizeof(rx)));
    spi.SR = SPI_SR_BSY;
    assert(!imu_fifo_dma_start(rx, sizeof(rx) - 1));
    spi.SR = 0;

    started = imu_fifo_dma_start(rx, sizeof(rx) - 1);
    assert(started && dma_rx.CMAR == (uintptr_t)rx);
    assert(dma_rx.CNDTR == sizeof(rx) && dma_tx.CNDTR == sizeof(rx));
    assert((dma_rx.CCR & DMA_CCR_EN) && (dma_tx.CCR & DMA_CCR_EN));
    assert(gpio_b.BSRR == (IMU_CS << 16) && gpio_c.BSRR == ADC_CS);
    assert((spi.CR1 & SPI_CR1_SPE) && ((spi.CR2 & dma_requests) == dma_requests));
    assert(!imu_fifo_dma_start(rx, IMU_FIFO_FRAME_BYTES));
    dma.ISR = transfer_done;
    spi.SR = SPI_SR_BSY;
    assert(imu_fifo_dma_poll() == IMU_FIFO_DMA_BUSY);
    assert(gpio_b.BSRR == (IMU_CS << 16));
    spi.SR = 0;
    assert(imu_fifo_dma_poll() == IMU_FIFO_DMA_DONE);
    assert(gpio_b.BSRR == IMU_CS && !(spi.CR2 & dma_requests) && !(spi.CR1 & SPI_CR1_SPE));
    assert(!dma_rx.CCR && !dma_tx.CCR);

    /* A DMA error cleans up the shared bus. A new read can start afterward. */
    dma.ISR = 0; /* Emulate the flags cleared by IFCR. */
    started = imu_fifo_dma_start(rx, IMU_FIFO_FRAME_BYTES);
    assert(started);
    dma.ISR = DMA_ISR_TEIF2;
    assert(imu_fifo_dma_poll() == IMU_FIFO_DMA_ERROR);
    assert(gpio_b.BSRR == IMU_CS && !(spi.CR2 & dma_requests));

    /* Timeout uses a difference so the 1 MHz timer can wrap in flight. */
    dma.ISR = 0;
    tim3.CNT = 65510;
    started = imu_fifo_dma_start(rx, IMU_FIFO_FRAME_BYTES);
    assert(started);
    tim3.CNT = 23;
    assert(imu_fifo_dma_poll() == IMU_FIFO_DMA_BUSY);
    tim3.CNT = 32;
    assert(imu_fifo_dma_poll() == IMU_FIFO_DMA_ERROR);
    assert(gpio_b.BSRR == IMU_CS && !dma_rx.CCR && !dma_tx.CCR);
    return 0;
}
