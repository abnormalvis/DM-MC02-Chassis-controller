/*
 * linker script for STM32H723RBTx
 * Flash: 128KB, RAM: 32KB + 64KB DTCM
 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 32K
    DTCMRAM (rwx) : ORIGIN = 0x20020000, LENGTH = 64K
}

ENTRY(Reset_Handler)

/* Stack and Heap sizes */
_estack = 0x20030000;  /* End of DTCM RAM */
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)         /* ARM/Thumb interworking */
        *(.glue_7t)
        *(.gcc_except_table)
        *(.ARM.exidx*)
        *(.ARM.extab*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } >FLASH

    . = ALIGN(4);
    _sidata = .;

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT>FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* DTCM for faster access */
    .dtcm :
    {
        . = ALIGN(4);
        *(.dtcm*)
        . = ALIGN(4);
    } >DTCMRAM

    /* Heap and Stack sections */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( _end = . );
        PROVIDE ( end = . );
        . = ALIGN(8);
        . = . + _Min_Heap_Size;
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >RAM

    /* Discard C++ exceptions and unwind tables */
    /DISCARD/ :
    {
        libc.a (*)
        libm.a (*)
        libgcc.a (*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}