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
#define		RESET_REQUEST_GPIO		GPIO18_16
#define		PLL_BYPASS_MODE_GPIO	GPIO18_17

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SPL_SPI_FLASH_SUPPORT
int s3ma_load_spi_image(void)
{
	struct spi_flash *flash;
	struct image_header *header;
	int	res = 1;
	/*
	 * Try loading U-Boot image from SPI data flash into RAM
	 */

	flash = spi_flash_probe(CONFIG_SPL_SPI_BUS, CONFIG_SPL_SPI_CS,
				CONFIG_SF_DEFAULT_SPEED, SPI_MODE_3);
	if (!flash) {
		puts("SPI probe failed.\n");
	}
	else
	{
		/* use CONFIG_SYS_TEXT_BASE as temporary storage area */
		header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

			/* Load u-boot, mkimage header is 64 bytes. */
			spi_flash_read(flash, CONFIG_SYS_SPI_U_BOOT_OFFS, 0x40,
				       (void *)header);
			spl_parse_image_header(header);

			spi_flash_read(flash, CONFIG_SYS_SPI_U_BOOT_OFFS,
				       spl_image.size, (void *)spl_image.load_addr);

			res = 0;
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

extern uint32_t _end[];

int	s3ma_load_nor_flash_image(void)
{
	struct image_header *header = (struct image_header*)_end;
	int res = 1;


	if(image_get_ep(header) == CONFIG_SYS_TEXT_BASE)
	{
		spl_parse_image_header(header);
		memcpy((void*)spl_image.load_addr, (void*)header, spl_image.size);
		res = 0;

	}

	return res;
}

extern uint32_t _rodata_dst_addr[];
extern uint32_t _rodata_src_addr[];
extern uint32_t _rodata_size[];
extern uint32_t _data_dst_addr[];
extern uint32_t _data_src_addr[];
extern uint32_t _data_size[];


void board_init_f(ulong dummy)
{
//	ps7_init();

	red_led_off();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);
	/* copy data sections */
	memcpy(_rodata_dst_addr, _rodata_src_addr, _rodata_size);
	memcpy(_data_dst_addr, _data_src_addr, _data_size);


	/* Set global data pointer. */
//	gd = &gdata;
    /* Clear global data */
	memset(gd, 0, sizeof(gd_t));

#ifdef CONFIG_SPL_SERIAL_SUPPORT
	preloader_console_init();

#endif
	arch_cpu_init();
	board_init_r(NULL, 0);
}

u32 spl_boot_device(void)
{
	u32 val;
	struct image_header *header;

	/* Check if we are running in PLL_BYPASS or NORMAL mode */
#if 0
	val = gpio_get_value(PLL_BYPASS_MODE_GPIO);
#else
	val = PLL_BYPASS_MODE_LEVEL;

	header = (struct image_header *)
			(CONFIG_SYS_TEXT_BASE -	sizeof(struct image_header));


	if(image_get_ep(header) == CONFIG_SYS_TEXT_BASE)
	{
		val = LOW;
	}

#endif
	if(PLL_BYPASS_MODE_LEVEL == val)
	{
		printf("Locating image in data flash...\n");
#ifdef CONFIG_SPL_SPI_FLASH_SUPPORT
		val = s3ma_load_spi_image();
		if(0 != val)
#endif
		{
//			val = s3ma_load_mmc_image();
			printf("No image in SPI data flash\n");
			printf("Locating image in nor flash...\n");
			val = s3ma_load_nor_flash_image();
		}
		if(0 == val)
		{
			printf("Booting image ...\n");
			gpio_set_value(RESET_REQUEST_GPIO, RESET_REQUEST_LEVEL);
			while(1);
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


