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
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>


void s3ma_setup_pll(void)
{
	/* TODO: Configure PLL settings */
}

void s3ma_ddr_clock_enable(void)
{
	u32 mru_clk_dis_reg;

	mru_clk_dis_reg = readl(CRT_CLK_DIS);

	mru_clk_dis_reg &= ~DMC_DIS_BITMASK;

	mru_clk_dis_reg |= ((u32)DMC_DIS_RESET << DMC_DIS_SHIFT) & (u32)DMC_DIS_BITMASK;

	writel(mru_clk_dis_reg, CRT_CLK_DIS);
}

void s3ma_ddr_phy_clk_sel_384(void)
{
	u32 reg;

	//mon_puts("BEGIN CHANGE FREQUENCY OF DDR I/F: 192->384\n");

	// mon_puts("Disable clock to DDR I/F\n");
	reg = readl(CRT_CLK_DIS);
	reg |= (u32)0x0 |((u32)0x1 << (DMC_DIS_SHIFT + 2));// "ddr_phy_clk"
	writel(reg,CRT_CLK_DIS);

	//REG32(DDR_CFG);					// use as delay
	__udelay(1);

	// mon_puts("Select 384MHz DDR I/F clock\n");
	reg = readl(DDR_CFG);
	reg |= (u32)0x0 | ((u32)0x1 << DDR_PHY_CLK_SEL_SHIFT);
	writel(reg,DDR_CFG);		// use "clk_384"

	// mon_puts("Enable clock to DDR I/F\n");
	reg = readl(CRT_CLK_DIS);
	reg &= ~((u32)0x0 |((u32)0x1 << (DMC_DIS_SHIFT + 2)));// "ddr_phy_clk"

	writel(reg,CRT_CLK_DIS);	// "ddr_phy_clk"
	// REG32(DDR_CFG);					// use as delay
	__udelay(1);

	// mon_puts("  END CHANGE FREQUENCY OF DDR I/F: 192->384\n");
}

void s3ma_ddr_phy_clk_sel_192(void)
{
	u32 reg;
	// mon_puts("BEGIN CHANGE FREQUENCY OF DDR I/F: 384->192\n");

	// mon_puts("Disable clock to DDR I/F\n");
	reg = readl(CRT_CLK_DIS);
	reg |= (u32)0x0 |((u32)0x1 << (DMC_DIS_SHIFT + 2));// "ddr_phy_clk"
	writel(reg,CRT_CLK_DIS);
	// REG32(DDR_CFG);					// use as delay
	__udelay(1);

	// mon_puts("Select 192MHz DDR I/F clock\n");
	reg = readl(DDR_CFG);
	reg &= ~((u32)0x0 | ((u32)0x1 << DDR_PHY_CLK_SEL_SHIFT));
	writel(reg,DDR_CFG);// use "clk_192"
	__udelay(1);

	// mon_puts("Enable clock to DDR I/F\n");
	reg = readl(CRT_CLK_DIS);
	reg &= ~((u32)0x0 |((u32)0x1 << (DMC_DIS_SHIFT + 2)));// "ddr_phy_clk"

	writel(reg,CRT_CLK_DIS);	// "ddr_phy_clk"
	__udelay(1);

//	mon_puts("  END CHANGE FREQUENCY OF DDR I/F: 384->192\n");
}

