/*
 * (C) Copyright 2014 Xilinx, Inc. Michal Simek
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <spl.h>
#include <spi_flash.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/sys_proto.h>

#define		HIGH	1
#define		LOW		0
#define		RESET_REQUEST_LEVEL		HIGH
#define		PLL_BYPASS_MODE_LEVEL	HIGH
#define		NORMAL_MODE_LEVEL	    LOW
#define		RESET_REQUEST_GPIO		GPIO18_16
#define		PLL_BYPASS_MODE_GPIO	GPIO18_20

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SPL_SPI_FLASH_SUPPORT
static
int s3ma_load_spi_image(void)
{
	struct spi_flash *flash;
	struct image_header *header;
	int	res = 1;
	/*
	 * Try loading U-Boot image from SPI data flash into RAM
	 */
	s3ma_gpio33_set_value(CONFIG_SPL_SPI_CS, 1);


	flash = spi_flash_probe(CONFIG_SPL_SPI_BUS, CONFIG_SPL_SPI_CS,
			CONFIG_SPL_SPI_DEFAULT_SPEED, CONFIG_SPL_SPI_DEFAULT_MODE);


	if (!flash) {
		puts("SPI probe failed.\n");
	}
	else
	{

		printf("SF: Detected %s with page size ", flash->name);
		print_size(flash->page_size, ", erase size ");
		print_size(flash->erase_size, ", total ");
		print_size(flash->size, "");

		if (flash->memory_map)
			printf(", mapped at %p", flash->memory_map);
		puts("\n");

		/* use CONFIG_SYS_TEXT_BASE as temporary storage area */
		header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

		/* Load u-boot, mkimage header is 64 bytes. */
		spi_flash_read(flash, CONFIG_SYS_SPI_U_BOOT_OFFS, 0x40,
				       (void *)header);

		print_buffer((uint32_t)header, header, 4, 16, 4 );
		puts("\n");

		if(image_get_magic(header) == IH_MAGIC)
		{
			printf("Valid image found\n");

			spl_parse_image_header(header);

			printf("Reading image %.s \n", spl_image.name);

			spi_flash_read(flash, CONFIG_SYS_SPI_U_BOOT_OFFS,
					       spl_image.size, (void *)spl_image.load_addr);


			printf("Done... \n");

			res = 0;
		}


	}

	return res;
}
#endif

#ifdef CONFIG_SPL_MMC_SUPPORT
int s3ma_load_mmc_image(void)
{
	int	res = 1;


	return res;
}
#endif

extern char _image_binary_end[0];
static
int	s3ma_load_nor_flash_image(void)
{
	struct image_header *header = (struct image_header*)_image_binary_end;
	int res = 1;


	if(image_get_magic(header) == IH_MAGIC)
	{
		spl_parse_image_header(header);
		memcpy((void*)spl_image.load_addr, (void*)header, spl_image.size);
		res = 0;

	}

	return res;
}

extern char _rodata_dst_addr[0];
extern char _rodata_src_addr[0];
extern char _rodata_size[0];
extern char _data_dst_addr[0];
extern char _data_src_addr[0];
extern char _data_size[0];


void board_init_f(ulong dummy)
{
//	ps7_init();


	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);
	/* copy data sections */
	memcpy((void*)_rodata_dst_addr, (void*)_rodata_src_addr, (uint32_t)_rodata_size);
	memcpy((void*)_data_dst_addr, (void*)_data_src_addr, (uint32_t)_data_size);


	/* Set global data pointer. */
//	gd = &gdata;
    /* Clear global data */
	memset((void*)gd, 0, sizeof(gd_t));

#ifdef CONFIG_SPL_SERIAL_SUPPORT
	preloader_console_init();

#endif
	arch_cpu_init();
	board_init_r(NULL, 0);
}

u32 spl_boot_device(void)
{
	u32 val;
#ifndef CONFIG_SPL_PLL_BYPASS
	const struct image_header *header = (struct image_header *)
											(CONFIG_SYS_TEXT_BASE -	sizeof(struct image_header));
#endif
	/* Check if we are running in PLL_BYPASS or NORMAL mode */
#ifdef CONFIG_SPL_PLL_BYPASS
	val = gpio_get_value(PLL_BYPASS_MODE_GPIO);
#else
	val = (image_get_magic(header) == IH_MAGIC) ? NORMAL_MODE_LEVEL : PLL_BYPASS_MODE_LEVEL;
#endif
	if(val == PLL_BYPASS_MODE_LEVEL)
	{
		do
		{
#ifdef CONFIG_SPL_SPI_FLASH_SUPPORT
			printf("Locating image in data flash...\n");
			val = s3ma_load_spi_image();
			if(0 == val)
			{
				break;
			}
			else
			{
				printf("No image in SPI data flash\n");
			}
#endif
			printf("Locating image in QSPI NOR flash...\n");
			val = s3ma_load_nor_flash_image();
			if(0 == val)
			{
				break;
			}
			else
			{
				printf("No image in QSPI NOR flash\n");
			}

			//				val = s3ma_load_mmc_image();

		}while(0);


		if(0 == val)
		{
			printf("Booting image ...\n");
#ifdef CONFIG_SPL_PLL_BYPASS
			gpio_set_value(RESET_REQUEST_GPIO, RESET_REQUEST_LEVEL);
			while(1);
#else
			reset_cpu(0);
#endif
		}
		else
		{
			printf("No bootable image found\n ");
			hang();

		}
	}

	return BOOT_DEVICE_RAM;
}

#ifdef CONFIG_SPL_MMC_SUPPORT
u32 spl_boot_mode(void)
{
	return MMCSD_MODE_FAT;
}
#endif

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* boot linux */
	return 0;
}
#endif


