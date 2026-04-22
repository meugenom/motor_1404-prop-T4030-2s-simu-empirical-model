ENTRY(Reset_Handler)

MEMORY {
    /* Physcal Memory STM32F407 */
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM); /* Конец RAM - стартовая точка для стека */

SECTIONS {
    /* Table of interrupts on start of FLASH */
    .isr_vector : {
        KEEP(*(.isr_vector))
    } > FLASH

    /* Programm's code */
    .text : {
        *(.text*)
        *(.rodata*)
        _sidata = LOADADDR(.data);
    } > FLASH

    /* Initial Values */
    .data : {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT > FLASH

    /* Empty Arrays */
    .bss : {
        _sbss = .;
        __bss_start__ = .;   /* <--- for standart library C */
        *(.bss*)
        *(COMMON)
        _ebss = .;
        __bss_end__ = .;     /* <--- for standart library C */
    } > RAM
    
    /* Start point for Heap, that need for function printf */
    . = ALIGN(4);
    end = .;                 /* <--- fpr standart library C */
}