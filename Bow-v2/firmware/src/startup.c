#include <stdint.h>
extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
extern void SysTick_Handler(void);
extern void TIM3_IRQHandler(void);
extern void EXTI0_1_IRQHandler(void);
extern void EXTI4_15_IRQHandler(void);
extern void EXTI2_3_IRQHandler(void);
void Default_Handler(void) { for (;;) __asm volatile ("wfi"); }
void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    for (uint32_t *p = &_sdata; p < &_edata;) *p++ = *src++;
    for (uint32_t *p = &_sbss; p < &_ebss;) *p++ = 0;
    /* Flash vector base, including when launched by a debugger. */
    *(volatile uint32_t *)0xE000ED08 = 0x08000000;
    (void)main();
    Default_Handler();
}
__attribute__((section(".isr_vector"), used))
void (*const vectors[48])(void) = {
    [0] = (void (*)(void))(&_estack), [1] = Reset_Handler,
    [2] = Default_Handler, [3] = Default_Handler,
    [11] = Default_Handler, [14] = Default_Handler, [15] = SysTick_Handler,
    [16 ... 20] = Default_Handler, [21] = EXTI0_1_IRQHandler,
    [22] = EXTI2_3_IRQHandler, [23] = EXTI4_15_IRQHandler,
    [24 ... 31] = Default_Handler, [32] = TIM3_IRQHandler,
    [33 ... 47] = Default_Handler
};
