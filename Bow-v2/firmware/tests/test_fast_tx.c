/* Production DMA ring with mocked registers: no target timing claims. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "../src/fast_protocol.h"
static struct { uint32_t AHBENR; } rcc;
static struct { uint32_t CCR,CNDTR; uintptr_t CPAR,CMAR; } dma_ch;
static struct { uint32_t ISR,IFCR; } dma;
static struct { uint32_t CCR; } mux;
static struct { uint32_t TDR,CR1,CR3; } uart;
#define RCC (&rcc)
#define DMA1_Channel1 (&dma_ch)
#define DMA1 (&dma)
#define DMAMUX1_Channel0 (&mux)
#define USART1 (&uart)
#define RCC_AHBENR_DMA1EN 1
#define DMA_IFCR_CGIF1 1
#define DMA_ISR_TCIF1 2
#define DMA_ISR_TEIF1 8
#define DMA_CCR_EN 1
#define DMA_CCR_DIR 16
#define DMA_CCR_MINC 128
#define USART_CR1_TXEIE_TXFNFIE 128
#define USART_CR3_DMAT 128
#define __DMB() ((void)0)
#include "../src/fast_tx.inc"
static unsigned packets;
static void complete(void) {
    assert(tx_active && dma_ch.CNDTR==tx_active);
    const uint8_t *p=(const uint8_t *)dma_ch.CMAR;
    for(unsigned j=0;j<tx_active;j+=32) {
        assert(p[j]==0xA5 && p[j+1]==0x5A);
        assert((p[j+4]|((unsigned)p[j+5]<<8))==packets++);
        assert(fast_crc(p+j,30)==(uint16_t)(p[j+30]|((unsigned)p[j+31]<<8)));
    }
    dma.ISR=DMA_ISR_TCIF1;tx_pump();dma.ISR=0; /* emulate write-to-clear */
}
int main(void) {
    tx_init();assert(mux.CCR==51 && (uart.CR3&128));
    uint8_t payload[18]={0};
    /* Delayed completions, multi-packet spans and many ring wraps. */
    for(unsigned seq=0;seq<300;++seq) {
        send_packet(1,seq,seq*1250,payload);
        if(seq%17==16) while(tx_active) complete();
    }
    while(tx_active)complete();
    assert(packets==300 && tx_in==tx_out && !tx_drops);
    /* A stalled DMA cannot have its in-flight storage overwritten. */
    for(unsigned seq=300;seq<363;++seq)send_packet(1,seq,0,payload);
    uint8_t snapshot[TX_SIZE];memcpy(snapshot,tx_ring,TX_SIZE);
    send_packet(1,363,0,payload);
    assert(tx_drops==1 && memcmp(snapshot,tx_ring,TX_SIZE)==0);
    while(tx_active)complete();assert(packets==363);
    /* A DMA bus error releases the abandoned span and enables recovery. */
    send_packet(1,364,0,payload);unsigned abandoned=tx_active/32;
    dma.ISR=DMA_ISR_TEIF1;tx_pump();dma.ISR=0;
    assert(tx_errors==1 && tx_drops==1+abandoned && !tx_active);
    send_packet(1,365,0,payload);packets=365;complete();assert(!tx_active);
    return 0;
}
