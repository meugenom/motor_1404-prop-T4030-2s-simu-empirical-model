#include <stdint.h>

extern uint32_t _estack;
extern uint32_t _sdata, _edata, _sidata;
extern uint32_t _sbss, _ebss;
extern int main(void);

// Caching handler for crtical faults
void HardFault_Handler(void) {
    while(1); 
}

void Reset_Handler(void) {
    // 1. ENABLE FPU (Floating Point Unit) - without it vsnprintf crashes!
    // Coprocessor Access Control Register (CPACR)
    volatile uint32_t *cpacr = (volatile uint32_t *)0xE000ED88;
    *cpacr |= (0xF << 20); // Allow full access to coprocessors 10 and 11

    // 2. Copy variables from Flash to RAM
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }
    
    // 3. Zero initialize the BSS section
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }
    
    // 4. Start the test!
    main();
    
    while (1);
}

// Full vector table
__attribute__((section(".isr_vector"), used))
const uint32_t vector_table[] = {
    (uint32_t)&_estack,        // 0: Stack pointer
    (uint32_t)Reset_Handler,   // 1: Reset handler
    0,                         // 2: NMI Handler (empty)
    (uint32_t)HardFault_Handler// 3: HardFault Handler
};