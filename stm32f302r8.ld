/*
** stm32f302r8-minimal.ld -- Load file for stm32f302r8
**
** Copyright (c) 2015 - Randy Stockberger
**
** --
**
**   STM32F302R8T6
**   16K RAM @ 0x20000000
**   64K ROM @ 0x08000000
*/
ENTRY(_reset)

MEMORY
{
  ROM (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 16K
}

_stacktop = 0x20004000;		/* Allocate stack at top of RAM */

SECTIONS				/* Sections Definitions */
{
    .text :
    {
		. = ALIGN(4);
        KEEP(*(.isr_vector)) /* Interrupt vector table, MUST be first */
		. = ALIGN(4);
        *(.text)			/* normal code */
        *(.text.*)			/* -ffunction-sections code */
        *(.rodata)			/* read-only data (constants) */
        /* *(.rodata*)		/* -fdata-sections read only data */
		. = ALIGN(4);
        _etext = .;
        _sidata = _etext;	/* Used by startup to initialize the .data section */
    } >ROM
    
    .data : AT ( _sidata )	/* Copied from _sidata(FLASH) to _sdata(RAM) at runtime */
    {
        /* _sdata = . ;		/* Used by startup to initialize the .data secion */
		. = ALIGN(4);
        _sdata = . ;
        *(.data)
        *(.data.*)
        /* *(.RAMtext) */
		. = ALIGN(4);
		 _edata = . ;		/* Used by startup to initialize the .data secion */
    } >RAM
    
	.bss :		/* Uninitialized data section */
	{
		_sbss = .;
		. = ALIGN(4);
		*(.bss)
		*(.bss.*)
		/* *(COMMON) */
		. = ALIGN(4);
		_ebss = .;
	} >RAM
		
/*
	PROVIDE ( end = _ebss );
	PROVIDE ( _end = _ebss );
*/
}
