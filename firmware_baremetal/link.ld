/*
 * Baremetal STM32F405xG memory layout.
 * Matches the existing ChibiOS firmware layout exactly so the
 * bootloader can flash and jump to this firmware unchanged.
 */

__main_stack_size__     = 0x0400;
__process_stack_size__  = 0x0400;

MEMORY
{
    flash_bl  : org = 0x08000000,        len = 32k       /* Bootloader (Sector 0) */
    flash_id  : org = 0x08000000 + 32k,  len = 16k       /* Board ID (Sector 1) */
    flash_nvp : org = 0x08000000 + 48k,  len = 16k       /* Calibrations (Sector 2) */
    flash_fw  : org = 0x08010000,        len = 1M - 64k  /* Firmware (Sectors 4-11) */
    ram       : org = 0x20000000,        len = 112k
    ethram    : org = 0x2001C000,        len = 16k
    ccmram    : org = 0x10000000,        len = 64k
}

__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;

ENTRY(ResetHandler)

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(vectors))
    } > flash_fw

    constructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash_fw

    destructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash_fw

    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash_fw

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash_fw

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash_fw

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash_fw

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash_fw

    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > flash_fw

    . = ALIGN(4);
    _etext = .;
    _textdata = _etext;

    .stacks :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > ram

    .data ALIGN(4) : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > flash_fw

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
    } > ram
}

PROVIDE(end = .);
_end            = .;

__heap_base__   = _end;
__heap_end__    = __ram_end__;
