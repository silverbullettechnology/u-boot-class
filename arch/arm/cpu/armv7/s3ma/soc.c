/*
 * (C) Copyright 2007
 * Sascha Hauer, Pengutronix
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/armv7.h>
#include <asm/pl310.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/s3ma-regs.h>
#include <asm/arch/boot_mode.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/sys_proto.h>

#if 0
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/dma.h>
#include <stdbool.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#endif

enum ldo_reg {
	LDO_ARM,
	LDO_SOC,
	LDO_PU,
};

struct scu_regs {
	u32	ctrl;
	u32	config;
	u32	status;
	u32	invalidate;
	u32	fpga_rev;
};

const uint32_t por_config[POR_CONFIG_LEN] __attribute__ ((section (".por_config"))) = {POR_CONFIG};



u32 get_cpu_rev(void)
{
#if 0
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	u32 reg = readl(&anatop->digprog_sololite);
	u32 type = ((reg >> 16) & 0xff);

	if (type != MXC_CPU_MX6SL) {
		reg = readl(&anatop->digprog);
		struct scu_regs *scu = (struct scu_regs *)SCU_BASE_ADDR;
		u32 cfg = readl(&scu->config) & 3;
		type = ((reg >> 16) & 0xff);
		if (type == MXC_CPU_MX6DL) {
			if (!cfg)
				type = MXC_CPU_MX6SOLO;
		}

		if (type == MXC_CPU_MX6Q) {
			if (cfg == 1)
				type = MXC_CPU_MX6D;
		}

	}
	reg &= 0xff;		/* mx6 silicon revision */
	return (type << 12) | (reg + 0x10);
#else
/* This may not be needed at all, but...
 * TODO -Figure out how to get silicon revision
*/

	return 0;
#endif
}

#if 0

#ifdef CONFIG_REVISION_TAG
u32 __weak get_board_rev(void)
{
	u32 cpurev = get_cpu_rev();
	u32 type = ((cpurev >> 12) & 0xff);
	if (type == MXC_CPU_MX6SOLO)
		cpurev = (MXC_CPU_MX6DL) << 12 | (cpurev & 0xFFF);

	if (type == MXC_CPU_MX6D)
		cpurev = (MXC_CPU_MX6Q) << 12 | (cpurev & 0xFFF);

	return cpurev;
}
#endif

#endif

#if 0
void init_aips(void)
{
	struct aipstz_regs *aips1, *aips2;

	aips1 = (struct aipstz_regs *)AIPS1_BASE_ADDR;
	aips2 = (struct aipstz_regs *)AIPS2_BASE_ADDR;

	/*
	 * Set all MPROTx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	writel(0x77777777, &aips1->mprot0);
	writel(0x77777777, &aips1->mprot1);
	writel(0x77777777, &aips2->mprot0);
	writel(0x77777777, &aips2->mprot1);

	/*
	 * Set all OPACRx to be non-bufferable, not require
	 * supervisor privilege level for access,allow for
	 * write access and untrusted master access.
	 */
	writel(0x00000000, &aips1->opacr0);
	writel(0x00000000, &aips1->opacr1);
	writel(0x00000000, &aips1->opacr2);
	writel(0x00000000, &aips1->opacr3);
	writel(0x00000000, &aips1->opacr4);
	writel(0x00000000, &aips2->opacr0);
	writel(0x00000000, &aips2->opacr1);
	writel(0x00000000, &aips2->opacr2);
	writel(0x00000000, &aips2->opacr3);
	writel(0x00000000, &aips2->opacr4);
}
#endif

#if 0
static void clear_ldo_ramp(void)
{
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	int reg;

	/* ROM may modify LDO ramp up time according to fuse setting, so in
	 * order to be in the safe side we neeed to reset these settings to
	 * match the reset value: 0'b00
	 */
	reg = readl(&anatop->ana_misc2);
	reg &= ~(0x3f << 24);
	writel(reg, &anatop->ana_misc2);
}

/*
 * Set the VDDSOC
 *
 * Mask out the REG_CORE[22:18] bits (REG2_TRIG) and set
 * them to the specified millivolt level.
 * Possible values are from 0.725V to 1.450V in steps of
 * 0.025V (25mV).
 */
static int set_ldo_voltage(enum ldo_reg ldo, u32 mv)
{
	struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
	u32 val, step, old, reg = readl(&anatop->reg_core);
	u8 shift;

	if (mv < 725)
		val = 0x00;	/* Power gated off */
	else if (mv > 1450)
		val = 0x1F;	/* Power FET switched full on. No regulation */
	else
		val = (mv - 700) / 25;

	clear_ldo_ramp();

	switch (ldo) {
	case LDO_SOC:
		shift = 18;
		break;
	case LDO_PU:
		shift = 9;
		break;
	case LDO_ARM:
		shift = 0;
		break;
	default:
		return -EINVAL;
	}

	old = (reg & (0x1F << shift)) >> shift;
	step = abs(val - old);
	if (step == 0)
		return 0;

	reg = (reg & ~(0x1F << shift)) | (val << shift);
	writel(reg, &anatop->reg_core);

	/*
	 * The LDO ramp-up is based on 64 clock cycles of 24 MHz = 2.6 us per
	 * step
	 */
	udelay(3 * step);

	return 0;
}
#endif

#if 0
static void imx_set_wdog_powerdown(bool enable)
{
	struct wdog_regs *wdog1 = (struct wdog_regs *)WDOG1_BASE_ADDR;
	struct wdog_regs *wdog2 = (struct wdog_regs *)WDOG2_BASE_ADDR;

	/* Write to the PDE (Power Down Enable) bit */
	writew(enable, &wdog1->wmcr);
	writew(enable, &wdog2->wmcr);
}
#endif

#if 0
static void set_ahb_rate(u32 val)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	u32 reg, div;

	div = get_periph_clk() / val - 1;
	reg = readl(&mxc_ccm->cbcdr);

	writel((reg & (~MXC_CCM_CBCDR_AHB_PODF_MASK)) |
		(div << MXC_CCM_CBCDR_AHB_PODF_OFFSET), &mxc_ccm->cbcdr);
}
#endif

#if 0
static void clear_mmdc_ch_mask(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* Clear MMDC channel mask */
	writel(0, &mxc_ccm->ccdr);
}
#endif

int arch_cpu_init(void)
{
	/* Initialize Boot QSPI flash drive strength to max*/
	setbits_le32(IO_CTL_E1_CFG,(uint32_t)0x00020000);
	setbits_le32(IO_CTL_E2_CFG,(uint32_t)0x00020000);
	/* Initialize Boot QSPI flash Slew Rate to fast */
	setbits_le32(IO_CTL_SR_CFG,(uint32_t)0x00020000);

	return 0;
}

int board_postclk_init(void)
{
#if 0
	set_ldo_voltage(LDO_SOC, 1175);	/* Set VDDSOC to 1.175V */

#endif
	return 0;
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Avoid random hang when download by usb */
	invalidate_dcache_all();
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif


void s_init(void)
{
	/* Disable clocks not needed */
	setbits_le32(CRT_CLK_DIS, (DMA_DIS_BITMASK | USB_DIS_BITMASK | SD_DIS_BITMASK));

	setbits_le32(CRT_WFE_DIS, (WFE_AHB_DIS_BITMASK | WFE_APB_DIS_BITMASK | \
			WFE_TX_DIS_BITMASK |WFE_RX_DIS_BITMASK |WFE_ADC_DAC_DIS_BITMASK));

	setbits_le32(CRT_LLM_DIS, (LLM_MAIN_DIS_BITMASK | LLM_AHB_DIS_BITMASK | \
			LLM_APB_DIS_BITMASK | LLM_SAMPLE_DIS_BITMASK));

	setbits_le32(CRT_DICEU_DIS, (DICEU_AHB_DIS_BITMASK | DICEU_APB_DIS_BITMASK | \
			DICEU_IQ_DIS_BITMASK | DICEU_OFDM_DIS_BITMASK | DICEU_SYS_DIS_BITMASK));

	setbits_le32(CRT_RF_DIS, WFE_DIS_BITMASK);
	setbits_le32(CRT_MDM_DIS, MDM_DIS_BITMASK);

	s3ma_setup_pll();
//	s3ma_ddr_clock_enable();
//	s3ma_ddr_setup();
}


#if 0

#ifdef CONFIG_IMX_HDMI
void imx_enable_hdmi_phy(void)
{
	struct hdmi_regs *hdmi = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	u8 reg;
	reg = readb(&hdmi->phy_conf0);
	reg |= HDMI_PHY_CONF0_PDZ_MASK;
	writeb(reg, &hdmi->phy_conf0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_ENTMDS_MASK;
	writeb(reg, &hdmi->phy_conf0);
	udelay(3000);
	reg |= HDMI_PHY_CONF0_GEN2_TXPWRON_MASK;
	writeb(reg, &hdmi->phy_conf0);
	writeb(HDMI_MC_PHYRSTZ_ASSERT, &hdmi->mc_phyrstz);
}

void imx_setup_hdmi(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	int reg;

	/* Turn on HDMI PHY clock */
	reg = readl(&mxc_ccm->CCGR2);
	reg |=  MXC_CCM_CCGR2_HDMI_TX_IAHBCLK_MASK|
		 MXC_CCM_CCGR2_HDMI_TX_ISFRCLK_MASK;
	writel(reg, &mxc_ccm->CCGR2);
	writeb(HDMI_MC_PHYRSTZ_DEASSERT, &hdmi->mc_phyrstz);
	reg = readl(&mxc_ccm->chsccdr);
	reg &= ~(MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_MASK|
		 MXC_CCM_CHSCCDR_IPU1_DI0_PODF_MASK|
		 MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_MASK);
	reg |= (CHSCCDR_PODF_DIVIDE_BY_3
		 << MXC_CCM_CHSCCDR_IPU1_DI0_PODF_OFFSET)
		 |(CHSCCDR_IPU_PRE_CLK_540M_PFD
		 << MXC_CCM_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);
}
#endif

#endif

#ifndef CONFIG_SYS_L2CACHE_OFF
#define IOMUXC_GPR11_L2CACHE_AS_OCRAM 0x00000002
void v7_outer_cache_enable(void)
{
	struct pl310_regs *const pl310 = (struct pl310_regs *)L2_PL310_BASE;
	unsigned int val;

#if defined CONFIG_MX6SL
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	val = readl(&iomux->gpr[11]);
	if (val & IOMUXC_GPR11_L2CACHE_AS_OCRAM) {
		/* L2 cache configured as OCRAM, reset it */
		val &= ~IOMUXC_GPR11_L2CACHE_AS_OCRAM;
		writel(val, &iomux->gpr[11]);
	}
#endif

	writel(0x132, &pl310->pl310_tag_latency_ctrl);
	writel(0x132, &pl310->pl310_data_latency_ctrl);

	val = readl(&pl310->pl310_prefetch_ctrl);

	/* Turn on the L2 I/D prefetch */
	val |= 0x30000000;

	/*
	 * The L2 cache controller(PL310) version on the i.MX6D/Q is r3p1-50rel0
	 * The L2 cache controller(PL310) version on the i.MX6DL/SOLO/SL is r3p2
	 * But according to ARM PL310 errata: 752271
	 * ID: 752271: Double linefill feature can cause data corruption
	 * Fault Status: Present in: r3p0, r3p1, r3p1-50rel0. Fixed in r3p2
	 * Workaround: The only workaround to this erratum is to disable the
	 * double linefill feature. This is the default behavior.
	 */

#ifndef CONFIG_MX6Q
	val |= 0x40800000;
#endif
	writel(val, &pl310->pl310_prefetch_ctrl);

	val = readl(&pl310->pl310_power_ctrl);
	val |= L2X0_DYNAMIC_CLK_GATING_EN;
	val |= L2X0_STNDBY_MODE_EN;
	writel(val, &pl310->pl310_power_ctrl);

	setbits_le32(&pl310->pl310_ctrl, L2X0_CTRL_EN);
}

void v7_outer_cache_disable(void)
{
	struct pl310_regs *const pl310 = (struct pl310_regs *)L2_PL310_BASE;

	clrbits_le32(&pl310->pl310_ctrl, L2X0_CTRL_EN);
}
#endif /* !CONFIG_SYS_L2CACHE_OFF */

void reset_cpu(ulong addr)
{
#if 0
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;

	writew(WCR_WDE, &wdog->wcr);
	writew(0x5555, &wdog->wsr);
	writew(0xaaaa, &wdog->wsr);	/* load minimum 1/2 second timeout */
	while (1) {
		/*
		 * spin for .5 seconds before reset
		 */
	}
#endif
}

