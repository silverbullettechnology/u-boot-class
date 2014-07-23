/*
 * clock.c
 *   Exelis inc   2014
 *
 *  Created on: Jun 18, 2014
 *      Author: Dmitriy Goldyuk
 *   Exelis inc
 */

#include <common.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/s3ma-regs.h>
#include <asm/arch/boot_mode.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>

/* Place holder for pll config */
static uint32_t	pll_config[3];

DECLARE_GLOBAL_DATA_PTR;

void s3ma_setup_pll(void)
{
	/* TODO: Configure PLL settings */
	/* Read por data and modify if needed
	 * For now just reference por record,
	 * so linker keeps por structure in the image
	 */
	int i;
	for(i = 0; i < sizeof(pll_config)/sizeof(pll_config[0]); i++)
	{
		pll_config[i] =  por_config[i];
	}


	/* Initialize values for clock rates */
	gd->cpu_clk = CONFIG_CPU_CLK_HZ;
	gd->bus_clk = CONFIG_PERIPHCLK_HZ;
}

void s3ma_ddr_clock_enable(void)
{
	clrbits_le32(CRT_CLK_DIS, DMC_DIS_BITMASK);
}

void s3ma_ddr_phy_clk_sel_384(void)
{
	//mon_puts("BEGIN CHANGE FREQUENCY OF DDR I/F: 192->384\n");

	// mon_puts("Disable clock to DDR I/F\n");
	setbits_le32(CRT_CLK_DIS, (uint32_t)0x1 << (DMC_DIS_SHIFT + 2));// "ddr_phy_clk"
	__udelay(1);

	// mon_puts("Select 384MHz DDR I/F clock\n");
	setbits_le32(DDR_CFG, (uint32_t)0x1 << DDR_PHY_CLK_SEL_SHIFT);// use "clk_384"
	__udelay(1);

	clrbits_le32(CRT_CLK_DIS, (uint32_t)0x1 << (DMC_DIS_SHIFT + 2));// "ddr_phy_clk"
	__udelay(1);

	// mon_puts("  END CHANGE FREQUENCY OF DDR I/F: 192->384\n");
}

void s3ma_ddr_phy_clk_sel_192(void)
{

	setbits_le32(CRT_CLK_DIS,(uint32_t)0x1 << (DMC_DIS_SHIFT + 2)); // "ddr_phy_clk"
	__udelay(1);

	// mon_puts("Select 192MHz DDR I/F clock\n");
	clrbits_le32(DDR_CFG,(uint32_t)0x1 << DDR_PHY_CLK_SEL_SHIFT); // use "clk_192"
	__udelay(1);

	// mon_puts("Enable clock to DDR I/F\n");
	clrbits_le32(CRT_CLK_DIS, (uint32_t)0x1 << (DMC_DIS_SHIFT + 2)); // "ddr_phy_clk"
	__udelay(1);

//	mon_puts("  END CHANGE FREQUENCY OF DDR I/F: 384->192\n");
}

void s3ma_sdhc_clk_enable(void)
{
	clrbits_le32(CRT_CLK_DIS, SD_DIS_BITMASK);

}
void s3ma_sdhc_clk_disable(void)
{
	setbits_le32(CRT_CLK_DIS, SD_DIS_BITMASK);

}

