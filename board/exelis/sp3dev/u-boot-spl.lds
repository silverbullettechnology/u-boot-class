/*
 * Copyright (c) 2004-2008 Texas Instruments
 *
 * (C) Copyright 2002
 * Gary Jennejohn, DENX Software Engineering, <garyj@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
	. = 0x00000000;

	. = ALIGN(4);
	.text :
	{
		*(.__image_copy_start)
		*(.vectors)
         CPUDIR/start.o (.text*)
         CPUDIR*(.text*)
         ARCH/lib*(.text*)
         drivers/serial*(.text*)
         BOARDDIR*(.text*)
         drivers/gpio*(.text*)
		 common*(.text.puts)     
         common*(.text.printf)   
		 common*(.text.sbrk)     
         common*(.text.mem_malloc_init)
         lib*(.text*) 
		 common/spl*(.text*)
		*(.text*)
		
		. = 8K;
		*(.por_config*)


	}

	. = ALIGN(4);
	
	_rodata_src_addr = . ;
	
	. = CONFIG_SPL_RAM_START_ADDR;
	
	_rodata_dst_addr = . ;

	.rodata : AT (_rodata_src_addr) 
	{ 
	    *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) 
	    . = ALIGN(4);
	}
	
	_rodata_size = SIZEOF(.rodata);
	

	_data_src_addr = _rodata_src_addr + _rodata_size ;

	_data_dst_addr = . ;
	
	.data : AT( _data_src_addr )
	{
		*(.data*)
	    . = ALIGN(4);
		
	}

	_data_size = SIZEOF(.data);
	
	. = ALIGN(4);

	. = .;

	.bss : {
		__bss_start = . ;
		*(.bss*)
		 . = ALIGN(4);
		 __bss_limit = . ;
		 __bss_end = . ;
	}

	_image_binary_end = _data_src_addr + _data_size;




/DISCARD/  : { *(.dynsym) }
/DISCARD/  : { *(.dynbss) }
/DISCARD/  : { *(.dynstr*) }
/DISCARD/  : { *(.dynamic*) }
/DISCARD/  : { *(.plt*) }
/DISCARD/  : { *(.interp*) }
/DISCARD/  : { *(.gnu.hash) }
/DISCARD/  : { *(.gnu*) }
/DISCARD/  : { *(.ARM.exidx*) }
/DISCARD/  : { *(.gnu.linkonce.armexidx.*) }
}
