/*
 * Copyright (c) 2014 Exelis Inc.
 *
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
	. = CONFIG_SPL_TEXT_BASE;

	. = ALIGN(4);
	.text :
	{
		*(.__image_copy_start)
		*(.vectors)
         arch/arm/cpu/armv7/start.o (.text*)
         arch/arm/cpu/armv7*(.text*)
         arch/arm/lib*(.text*)
         drivers/serial*(.text*)
         BOARDDIR*(.text*)
         drivers/gpio*(.text*)
         common*(.text*)
         lib*(.text.memset*) 

		. = 8K;
		*(.por_config*)
		
		*(.text*)
	
	}	
    
	. = ALIGN(4);

	.data : 
	{
		*(.data*)
	}



	. = ALIGN(4);
	.rodata : { *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) }


	. = ALIGN(4);

	_image_binary_end = .;
	_end = _image_binary_end;
	
	. =  CONFIG_SPL_BSS_START_ADDR;
 
 .bss (NOLOAD) : {
   . = ALIGN(4);
  __bss_start = .;
  *(.bss*)
   . = ALIGN(4);
  __bss_end = .;
 }
	

 /DISCARD/ : { *(.dynsym) }
 /DISCARD/ : { *(.dynstr*) }
 /DISCARD/ : { *(.dynamic*) }
 /DISCARD/ : { *(.plt*) }
 /DISCARD/ : { *(.interp*) }
 /DISCARD/ : { *(.gnu*) }

}
