/*
 * Copyright (c) 2014 Exelis Inc.
 *
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
 MEMORY 
 {
 	flash	  (RX)  : ORIGIN = CONFIG_SPL_TEXT_BASE, LENGTH = 0x2000
 	por_flash (R)   : ORIGIN = 0x2000, LENGTH = 0x400
 	ocm_bss	  (RWX) : ORIGIN = CONFIG_SPL_BSS_START_ADDR, LENGTH = CONFIG_SPL_BSS_MAX_SIZE
 	ocm_d 	  (R)   : ORIGIN = CONFIG_SPL_DATA_START_ADDR, LENGTH = CONFIG_SPL_BSS_MAX_SIZE
 }

SECTIONS
{
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
#if 0
         common*(.text*)
#else
		 common*(.text.puts)     
               
         common*(.text.printf)   
               
		 common*(.text.sbrk)     
               
         common*(.text.mem_malloc_init)
#endif         
         lib*(.text*) 
		 
		 common/spl*(.text*)

		*(.text*)
     
	} > flash =0x00

#if 0
	.fill  : 
	{
		FILL(0x00);
	 . = ORIGIN(flash) + LENGTH(flash) - 1;
		BYTE(0xAA);
	} > flash

#endif

   .por : 
   {
	 *(.por_config*)
   } > por_flash
   
	_big_data_src = ORIGIN(por_flash) + LENGTH(por_flash );
		
	.big_data : AT ( _big_data_src  )  
	{
	   . = ALIGN(4);
    	_big_data_dst = . ; 
    	  *(.data*)
    	  . = ALIGN(4);
    	  *(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*))) 
	   . = ALIGN(4);
	} > ocm_d
	
	_big_data_size = SIZEOF( .big_data );

	_image_binary_end = _big_data_src + _big_data_size ;
	_end = _image_binary_end;
		
 
 .bss  : {
   . = ALIGN(4);
  __bss_start = .;
  *(.bss*)
   . = ALIGN(4);
  __bss_end = .;
 } > ocm_bss
	

#if 1
 /DISCARD/ : { *(.dynsym) }
 /DISCARD/ : { *(.dynstr*) }
 /DISCARD/ : { *(.dynamic*) }
 /DISCARD/ : { *(.plt*) }
 /DISCARD/ : { *(.interp*) }
 /DISCARD/ : { *(.gnu*) }
#endif
}
