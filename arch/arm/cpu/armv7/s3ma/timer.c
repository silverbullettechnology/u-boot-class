/*
 * (C) Copyright 2007
 * Sascha Hauer, Pengutronix
 *
 * (C) Copyright 2009 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <div64.h>
#include <asm/arch/clock.h>

/* MPCore Global timer registers */
typedef struct globaltimer {
	u32 cnt_l; /* 0x00 */
	u32 cnt_h;
	u32 ctl;
	u32 stat;
	u32 cmp_l; /* 0x10 */
	u32 cmp_h;
	u32 inc;
}GTIMER, * GTIMER_PTR;

//static struct globaltimer * gt = (struct globaltimer *)GT_CNTR_L;


DECLARE_GLOBAL_DATA_PTR;

static inline unsigned long long tick_to_time(unsigned long long tick)
{
#if 0
	tick *= CONFIG_SYS_HZ;
	do_div(tick, MXC_CLK32);
#endif
	return tick;
}

static inline unsigned long long us_to_tick(unsigned long long usec)
{
#if 0
	usec = usec * MXC_CLK32 + 999999;
	do_div(usec, 1000000);
#endif
	return usec;
}

static inline void read_timer_count(uint32_t* high_word_p, uint32_t* low_word_p)
{
	/* Per ARM spec
	 * To get the value from the Global Timer Counter register proceed as follows:
	 * 1. Read the upper 32-bit timer counter register
	 * 2. Read the lower 32-bit timer counter register
	 * 3. Read the upper 32-bit timer counter register again.
	 * If the value is different to the 32-bit upper value read previously,
	 * go back to step 2. Otherwise the 64-bit timer counter value is correct.
	 */
	GTIMER_PTR gt = (GTIMER_PTR)(GT_CNTR_L);
	uint32_t high;
	uint32_t low;

	do{
		high = readl(&gt->cnt_h);
		low = readl(&gt->cnt_l);
	}while(high != readl(&gt->cnt_h));

	*high_word_p = high;
	*low_word_p = low;
}

int timer_init(void)
{
#if 0
	int i;

	/* setup GP Timer 1 */
	__raw_writel(GPTCR_SWR, &cur_gpt->control);

	/* We have no udelay by now */
	for (i = 0; i < 100; i++)
		__raw_writel(0, &cur_gpt->control);

	__raw_writel(0, &cur_gpt->prescaler); /* 32Khz */

	/* Freerun Mode, PERCLK1 input */
	i = __raw_readl(&cur_gpt->control);
	__raw_writel(i | GPTCR_CLKSOURCE_32 | GPTCR_TEN, &cur_gpt->control);

	gd->arch.tbl = __raw_readl(&cur_gpt->counter);
	gd->arch.tbu = 0;
#endif
	uint32_t upper,lower;
	GTIMER_PTR gt = (GTIMER_PTR)(GT_CNTR_L);

	/* Reset and turn Global Timer off */
	writel(GT_CNTL_REG_RESET, &gt->ctl);
	/* Make timer tick at 1000*CONFIG_SYS_HZ rate */
	clrbits_le32(&gt->ctl,PRESCALER_BITMASK);
	setbits_le32(&gt->ctl,(lldiv((uint64_t)gd->bus_clk, (uint32_t)1000*CONFIG_SYS_HZ) - 1)<< PRESCALER_SHIFT);
	/* Disable interrupts */
	clrbits_le32(&gt->ctl, IRQ_EN_BITMASK);
	/* Clear counter value */
	writel(0,&gt->cnt_l);

	writel(0,&gt->cnt_h);
	/* Enable timer */
	setbits_le32(&gt->ctl, TIMER_EN_BITMASK);

	read_timer_count(&upper,&lower);

	gd->arch.tbu = (ulong)upper;
	gd->arch.tbl = (ulong)lower;

	return 0;
}

unsigned long long get_ticks(void)
{
#if 0
	ulong now = __raw_readl(&cur_gpt->counter); /* current tick value */

	/* increment tbu if tbl has rolled over */
	if (now < gd->arch.tlbl)
		gd->arch.tbu++;
	gd->arch.tbl = now;
#else
	uint32_t upper,lower;

	read_timer_count(&upper,&lower);
	gd->arch.tbu = upper;
	gd->arch.tbl = lower;
#endif
	return (((unsigned long long)gd->arch.tbu) << 32) | gd->arch.tbl;
}

#if 0
ulong get_timer_masked(void)
{
	/*
	 * get_ticks() returns a long long (64 bit), it wraps in
	 * 2^64 / MXC_CLK32 = 2^64 / 2^15 = 2^49 ~ 5 * 10^14 (s) ~
	 * 5 * 10^9 days... and get_ticks() * CONFIG_SYS_HZ wraps in
	 * 5 * 10^6 days - long enough.
	 */
	return tick_to_time(get_ticks());
}
#endif

ulong get_timer(ulong base)
{
#if 0
	return get_timer_masked() - base;
#else
	return tick_to_time(get_ticks()) - base;
#endif
}

/* delay x useconds AND preserve advance timstamp value */
void __udelay(unsigned long usec)
{
	unsigned long long tmp;
	ulong tmo;

	tmo = us_to_tick(usec);
	tmp = get_ticks() + tmo;	/* get current timestamp */

	while (get_ticks() < tmp)	/* loop till event */
		 /*NOP*/;
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
#if 0
	return MXC_CLK32;
#endif
	return 1000;
}
