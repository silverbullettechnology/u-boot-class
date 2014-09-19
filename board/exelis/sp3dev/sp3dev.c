/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013, Boundary Devices <info@boundarydevices.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/s3ma-regs.h>
#include <malloc.h>
#include <asm/errno.h>
#include <mmc.h>
#include <sdhci.h>
#include <micrel.h>
#include <miiphy.h>
#include <netdev.h>
#include <i2c.h>
#include <spi.h>
#ifdef CONFIG_USB_GADGET_S3C_UDC_OTG
#include <usb/s3c_udc.h>
#endif
#ifdef CONFIG_POST
#include <post.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/* Declare Power on reset configuration space data
 * used by hardware bootloader
 */

int dram_init(void)
{
#ifndef CONFIG_RUN_ON_QEMU
	s3ma_ddr_clock_enable();
	s3ma_ddr_setup();
#endif
	gd->ram_size = ((ulong)CONFIG_S3MA_RAM_SIZE);

	return 0;
}

#ifdef CONFIG_CMD_MMC
int board_mmc_init(bd_t *bis)
{
	struct sdhci_host *host = NULL;

	host = (struct sdhci_host *)malloc(sizeof(*host));
	if (!host) {
		printf("s3ma_sdhci_init: sdhci_host malloc fail\n");
		return 1;
	}
/* Enable SDHC controller clock  */
	host->name = "s3ma_sdhci";
	host->ioaddr = (void *)(SD_S_ABSOLUTE_BASE);
#ifdef CONFIG_RUN_ON_QEMU
	add_sdhci(host, 48000000, 48000000);
#else
	s3ma_sdhc_clk_enable();
	add_sdhci(host, 0, 0);
#endif
	return 0;


}
#endif

#if 0
int board_mmc_getcd(struct mmc *mmc)
{
	int ret = 0;
	/*TODO: Detect if card is present(probably there is a GPIO for it) */

	return ret;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	imx_iomux_v3_setup_multiple_pads(usb_pads, ARRAY_SIZE(usb_pads));

	/* Reset USB hub */
	gpio_direction_output(IMX_GPIO_NR(7, 12), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(7, 12), 1);

	return 0;
}

int board_ehci_power(int port, int on)
{
	if (port)
		return 0;
	gpio_set_value(GP_USB_OTG_PWR, on);
	return 0;
}

#endif

#ifdef CONFIG_USB_GADGET_S3C_UDC_OTG

static int s3ma_phy_control(int on)
{
	return 0;
}

struct s3c_plat_otg_data s3ma_otg_data = {
	.phy_control	= s3ma_phy_control,
	//.regs_phy	= EXYNOS4X12_USBPHY_BASE,
	.regs_otg	= (USB0_S_ABSOLUTE_BASE),
	//.usb_phy_ctrl	= EXYNOS4X12_USBPHY_CONTROL,
	//.usb_flags	= PHY0_SLEEP,
};
#endif

int board_eth_init(bd_t *bis)
{

#ifdef CONFIG_USB_GADGET_S3C_UDC_OTG

	/* For otg ethernet*/
	usb_eth_initialize(bis);
	return s3c_udc_probe(&s3ma_otg_data);
#endif
	return 0;
}




int board_early_init_f(void)
{
	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
	return 0;
}

int checkboard(void)
{
	return 0;
}



int misc_init_r(void)
{
#ifdef CONFIG_POST
	writel(0, (CONFIG_SYS_POST_WORD_ADDR));
#endif
	return 0;
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	/* TODO: Add Chip select check code */

	return 1;
}

void spi_cs_activate(struct spi_slave *slave)
{
	/* TODO: Add Chip select assertion code */
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	/* TODO: Add Chip select de-assertion code */
}

#if (CONFIG_POST & CONFIG_SYS_POST_MEMORY)

int arch_memory_test_prepare(u32 *vstart, u32 *size, phys_addr_t *phys_offset)
{
//	bd_t *bd = gd->bd;

	*vstart = CONFIG_SYS_SDRAM_BASE;
	*size = gd->ram_size;
	gd->post_log_res |= CONFIG_SYS_POST_MEMORY;

	return 0;
}

void arch_memory_failure_handle(void)
{
	/* Save result in global data structure */
	gd->post_log_res &= ~CONFIG_SYS_POST_MEMORY;
	return;
}
#endif

