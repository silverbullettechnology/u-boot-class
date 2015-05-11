MEMORY { .sram : ORIGIN = CONFIG_SPL_TEXT_BASE,\
		LENGTH = 8*1024 }
MEMORY { .sdram : ORIGIN = CONFIG_SPL_BSS_START_ADDR, \
		LENGTH =  16*1024}
MEMORY { .por : ORIGIN = 0x2000,\
		LENGTH = 1024 }

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_start)
SECTIONS
{
	. = ALIGN(4);
	.text :
	{
		__image_copy_start = .;
		CPUDIR/start.o (.text*)
		*(.text*)
	} > .sram

	. = ALIGN(4);
	.rodata : {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
	} > .sram

	. = ALIGN(4);
	.data : {
		*(.data*)
	} > .sram

	. = ALIGN(4);

	. = .;

	__image_copy_end = .;

	_end = .;

	/* Move BSS section to RAM because of FAT */
	.bss : {
		__bss_start = .;
		*(.bss*)
		 . = ALIGN(4);
		__bss_end = .;
	} > .sdram

	.por_config : {
		*(.por_config*)
	} > .por
	/DISCARD/ : { *(.dynsym) }
	/DISCARD/ : { *(.dynstr*) }
	/DISCARD/ : { *(.dynamic*) }
	/DISCARD/ : { *(.plt*) }
	/DISCARD/ : { *(.interp*) }
	/DISCARD/ : { *(.gnu*) }
}
