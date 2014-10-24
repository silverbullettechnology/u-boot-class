/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Boundary Devices Nitrogen6X
 * and Freescale i.MX6Q Sabre Lite boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "s3ma_common.h"

/***********************/

/* Custom build options for debugging  */

#define CONFIG_DEBUG_BUILD
#define CONFIG_RUN_ON_QEMU

/***********************/

#define CONFIG_S3MA


/*This one has to be defined by
 * http://www.arm.linux.org.uk/developer/machines/
 * We don't have the number assigned, so just pick something for now
 */
#define CONFIG_MACH_TYPE	5000

#include <asm/arch/s3ma-regs.h>
#include <asm/arch/gpio.h>

#if 0
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#endif

#define CONFIG_SYS_GENERIC_BOARD

/* System clock rates */
#define CONFIG_CPU_CLK_HZ			768000000
#define CONFIG_PERIPHCLK_HZ			(CONFIG_CPU_CLK_HZ/4)

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN				(1024 * 1024)
#define CONFIG_SYS_FALLBACK_MALLOC_LEN		(32 * 1024)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R

#define CONFIG_CMD_GPIO
#define CONFIG_PL061_GPIO
#define CONFIG_GPIO_BASE	(GPIO0_APB_ABSOLUTE_BASE)
#define CONFIG_GPIO_BASE1	(GPIO1_APB_ABSOLUTE_BASE)
#define CONFIG_GPIO_BASE2	(GPIO2_APB_ABSOLUTE_BASE)


#if 0
#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif
#endif

#if 0
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	       UART2_BASE
#else
#define CONFIG_PL011_SERIAL
#define CONFIG_PL011_CLOCK	24000000
#define CONFIG_PL01x_PORTS				\
			{(void *)CONFIG_SYS_SERIAL0,	\
			 (void *)CONFIG_SYS_SERIAL1 }

#define	CONFIG_SYS_SERIAL0	UART0_APB_ABSOLUTE_BASE
#define	CONFIG_SYS_SERIAL1	UART1_APB_ABSOLUTE_BASE

#define CONFIG_CONS_INDEX	0

#define CONFIG_BAUDRATE			115200
#define CONFIG_PL011_SERIAL_FLUSH_ON_INIT
#endif


/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_DW_I2C
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_BASE			I2C_APB_ABSOLUTE_BASE
#define	CONFIG_SYS_I2C_SLAVE		0x90

/* MMC Configs */
#define CONFIG_SDHCI
#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_MMC_SDMA
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION
#define	CONFIG_SYS_MMC_MAX_DEVICE	1

/* SPI bus configs */
#define CONFIG_CMD_SPI
#define CONFIG_PL022_SPI
#define	CONFIG_SYS_SPI_CLK		38400000
#define CONFIG_SYS_SPI_BASE		(SPI0_APB_ABSOLUTE_BASE)
#define CONFIG_SYS_SPI_BASE1	(SPI1_APB_ABSOLUTE_BASE)
#define CONFIG_SYS_SPI_BASE2	(SPI2_APB_ABSOLUTE_BASE)
#define CONFIG_SYS_SPI_BASE3	(SPI3_APB_ABSOLUTE_BASE)
#define CONFIG_SYS_SPI_BASE4	(SPI4_APB_ABSOLUTE_BASE)

/*
 * SATA Configs
 */
#if 0

#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		6
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ9021
#endif

/* USB Configs */
#if 1
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_S3C_UDC_OTG
#define CONFIG_USB_DEVICE
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_USB_ETH_RNDIS
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#else
#define CONFIG_CMD_USB
#define CONFIG_USB_DEVICE
#define CONFIG_USB_ETH_CDC
#define CONFIG_USB_TTY
#define CONFIG_USBD_HS
#define CONFIG_USB_ULPI
#define CONFIG_USB_ULPI_VIEWPORT
#define CONFIG_ULPI_REF_CLK		60000000
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#endif

/* Miscellaneous commands */
#if 0
#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR
#endif

#define CONFIG_SYS_CACHELINE_SIZE	32

/* Framebuffer and LCD */
#if 0
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_CMD_HDMIDETECT
#define CONFIG_CONSOLE_MUX
#define CONFIG_IMX_HDMI
#define CONFIG_IMX_VIDEO_SKIP
#endif

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#if 0
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200
#endif

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	       1



#if 0
#define CONFIG_SYS_TEXT_BASE	       0x17800000
#else
#define CONFIG_SYS_TEXT_BASE	       0x00000000
#endif

#if 0
#ifdef CONFIG_CMD_SATA
#define CONFIG_DRIVE_SATA "sata "
#else
#define CONFIG_DRIVE_SATA
#endif
#endif

#ifdef CONFIG_CMD_MMC
#define CONFIG_DRIVE_MMC "mmc "
#else
#define CONFIG_DRIVE_MMC
#endif

#define CONFIG_DRIVE_TYPES CONFIG_DRIVE_SATA CONFIG_DRIVE_MMC

#if 0
#if defined(CONFIG_SABRELITE)
#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"uimage=uImage\0" \
	"console=ttymxc1\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=imx6q-sabrelite.dtb\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"mmcdev=0\0" \
	"mmcpart=1\0" \
	"mmcroot=/dev/mmcblk0p2 rootwait rw\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loaduimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${uimage}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${uimage}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootm ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootm; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootm; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run loaduimage; then " \
				   "run mmcboot; " \
			   "else run netboot; " \
			   "fi; " \
		   "fi; " \
	   "else run netboot; fi"
#else
#define CONFIG_EXTRA_ENV_SETTINGS \
	"console=ttymxc1\0" \
	"clearenv=if sf probe || sf probe || sf probe 1 ; then " \
		"sf erase 0xc0000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	"bootcmd=for dtype in " CONFIG_DRIVE_TYPES \
		"; do " \
			"for disk in 0 1 ; do ${dtype} dev ${disk} ;" \
				"for fs in fat ext2 ; do " \
					"${fs}load " \
						"${dtype} ${disk}:1 " \
						"10008000 " \
						"/6x_bootscript" \
						"&& source 10008000 ; " \
				"done ; " \
			"done ; " \
		"done; " \
		"setenv stdout serial,vga ; " \
		"echo ; echo 6x_bootscript not found ; " \
		"echo ; echo serial console at 115200, 8N1 ; echo ; " \
		"echo details at http://boundarydevices.com/6q_bootscript ; " \
		"setenv stdout serial\0" \
	"upgradeu=for dtype in " CONFIG_DRIVE_TYPES \
		"; do " \
		"for disk in 0 1 ; do ${dtype} dev ${disk} ;" \
		     "for fs in fat ext2 ; do " \
				"${fs}load ${dtype} ${disk}:1 10008000 " \
					"/6x_upgrade " \
					"&& source 10008000 ; " \
			"done ; " \
		"done ; " \
	"done\0" \

#endif
#endif

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE	       1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE


#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       		   (DMC_S_ABSOLUTE_BASE)

#define CONFIG_DDR_SIZE					1024*1024*1024
#define CONFIG_S3MA_OCM_RAM_BASE		(OCM_S_ABSOLUTE_BASE)
#define CONFIG_S3MA_OCM_RAM_SIZE		(OCM_S_SIZE)
#define CONFIG_S3MA_RAM_SIZE			(CONFIG_DDR_SIZE)
#if 0
#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE
#else
//#define CONFIG_SYS_INIT_RAM_ADDR       OCM_S_ABSOLUTE_BASE
//#define CONFIG_SYS_INIT_RAM_SIZE       0x8000
//#define CONFIG_SYS_SDRAM_BASE	       (OCM_S_ABSOLUTE_BASE+CONFIG_SYS_INIT_RAM_SIZE)
#define CONFIG_SYS_INIT_RAM_ADDR       0x500//SYS_INIT_RAM_BASE
#define CONFIG_SYS_INIT_RAM_SIZE       (0x2000 - CONFIG_SYS_INIT_RAM_ADDR) //SYS_INIT_RAM_SIZE
//#define CONFIG_SYS_SDRAM_BASE	       (OCM_S_ABSOLUTE_BASE)
#define CONFIG_SYS_SDRAM_BASE	       (PHYS_SDRAM)
#endif

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define S3MA_L2_PL310_BASE		(PL310_L2_ABSOLUTE_BASE)
#define L2_PL310_BASE			(S3MA_L2_PL310_BASE)



#define CONFIG_LOADADDR			       CONFIG_SYS_SDRAM_BASE

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_CMD_SF
#define CONFIG_CMD_SF_TEST
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SST

# define CONFIG_SF_DEFAULT_BUS		(4)
# define CONFIG_SF_DEFAULT_CS		(GPIO33_3) //GPIO33_3
# define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
# define CONFIG_SF_DEFAULT_SPEED	(1000000)

#if defined(CONFIG_RUN_ON_QEMU)
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_SIZE			(16 * 1024)
#else
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SIZE			(16 * 1024)
#endif

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(0)
#define CONFIG_ENV_SECT_SIZE	(16 * 1024)
#define CONFIG_ENV_SPI_BUS		(CONFIG_SF_DEFAULT_BUS)
#define CONFIG_ENV_SPI_CS		(CONFIG_SF_DEFAULT_CS)
#define CONFIG_ENV_SPI_MODE		(CONFIG_SF_DEFAULT_MODE)
#define CONFIG_ENV_SPI_MAX_HZ	(CONFIG_SF_DEFAULT_SPEED)
#endif

#if 0
#define CONFIG_OF_LIBFDT
#endif

#if 0
#define CONFIG_CMD_BOOTZ
#endif

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#if 0
#define CONFIG_CMD_BMP
#endif

#define CONFIG_CMD_TIME
#define CONFIG_SYS_ALT_MEMTEST

#if 0
#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD
#endif
#define CONFIG_CMD_FS_GENERIC

/*
 * PCI express
 */
#if 0
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif
#endif

/*
 * AD9361
 */
#define CONFIG_AD9361
#define CONFIG_AD9361_MAX_DEVICE 4
/*
 * Standalone applications
 */

#define CONFIG_STANDALONE_LOAD_ADDR		0x8000000
//#define CONFIG_API

/*  Power-on self test */
#define CONFIG_POST	(CONFIG_SYS_POST_MEMORY)
#define CONFIG_SYS_POST_WORD_ADDR	(CONFIG_S3MA_OCM_RAM_BASE)
//#define CONFIG_POST_STD_LIST

/* Memory test commands */
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START       (CONFIG_SYS_SDRAM_BASE)
#define CONFIG_SYS_MEMTEST_END	       (CONFIG_SYS_MEMTEST_START+CONFIG_S3MA_RAM_SIZE - 8*1024*1024)
#define CONFIG_SYS_MEMTEST_SCRATCH     0

/* DAC configs */
#define CONFIG_CMD_DAC
#define CONFIG_LT2640_DAC
#define CONFIG_LT2640_DAC_SPEED		(1000000)
#define CONFIG_LT2640_DAC_MODE		SPI_MODE_0
#define CONFIG_LT2640_DAC_CS		(GPIO33_7)
#define CONFIG_LT2640_DAC_BUS		(4)
#define CONFIG_LT2640_DAC_DEFAULT	(0)

#endif	       /* __CONFIG_H */
