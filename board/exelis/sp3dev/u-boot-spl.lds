/*
 * Copyright (c) 2014 Exelis Inc.
 *
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
MEMORY { .flash1 : ORIGIN = CONFIG_SPL_TEXT_BASE,\
		LENGTH = 0x2000 }
MEMORY { .flash2 : ORIGIN = 0x2400,\
		LENGTH = CONFIG_SPL_MAX_SIZE }
MEMORY { .porconfig : ORIGIN = 0x2000,\
		LENGTH = 0x400}

MEMORY { .ocm : ORIGIN = CONFIG_SPL_BSS_START_ADDR, \
		LENGTH = CONFIG_SPL_BSS_MAX_SIZE }

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)

SECTIONS
{
	.text1 :
	{
	  __image_copy_start = .;
  	  *(.vectors)
  	  arch/arm/cpu/armv7/start.o (.text*)
  	  arch/arm/cpu/armv7*(.text*)
	  arch/arm/lib*(.text*)  	  
      drivers/serial*(.text*)
      BOARDDIR*(.text*)
	  drivers/gpio*(.text*)
	  common*(.text*)
/*	  
	  common*(.text.puts*)
	  common*(.text.fputs*)
	  common*(.text.printf*)
*/	  
	  lib*(.text.memset*)	        
	} > .flash1
	
	.por :
	{
		*(.por_config*)
	} > .porconfig
	
	.text2 :
	{
      *(.text*)
	} > .flash2

	. = ALIGN(4);
	.rodata : { 
	    *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) 
	    } > .flash2

	.data : {
		*(.data*)
	} > .flash2


	. = ALIGN(4);

	. = .;

	__image_copy_end = .;

	_end = .;


	 . = ALIGN(4);
	.bss (NOLOAD) : {
		__bss_start = .;
		*(.bss*)
		 . = ALIGN(4);
		__bss_end = .;
	} > .ocm

	/DISCARD/ : { *(.dynsym) }
	/DISCARD/ : { *(.dynstr*) }
	/DISCARD/ : { *(.dynamic*) }
	/DISCARD/ : { *(.plt*) }
	/DISCARD/ : { *(.interp*) }
	/DISCARD/ : { *(.gnu*) }
}


