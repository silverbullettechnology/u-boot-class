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

#if 1
DECLARE_GLOBAL_DATA_PTR;

struct scu_timer {
	u32 load; /* Timer Load Register */
	u32 counter; /* Timer Counter Register */
	u32 control; /* Timer Control Register */
};

static struct scu_timer *timer_base =
			      (struct scu_timer *)(PTIMER0_ABSOLUTE_BASE);

#define SCUTIMER_CONTROL_PRESCALER_MASK	0x0000FF00 /* Prescaler */
#define SCUTIMER_CONTROL_PRESCALER_SHIFT	8
#define SCUTIMER_CONTROL_AUTO_RELOAD_MASK	0x00000002 /* Auto-reload */
#define SCUTIMER_CONTROL_ENABLE_MASK		0x00000001 /* Timer enable */

#define TIMER_LOAD_VAL 0xFFFFFFFF
#define TIMER_PRESCALE 255

int timer_init(void)
{
	const u32 emask = SCUTIMER_CONTROL_AUTO_RELOAD_MASK |
			(TIMER_PRESCALE << SCUTIMER_CONTROL_PRESCALER_SHIFT) |
			SCUTIMER_CONTROL_ENABLE_MASK;

	gd->arch.timer_rate_hz = (gd->cpu_clk / 2) / (TIMER_PRESCALE + 1);
	/* Load the timer counter register */
	writel(0xFFFFFFFF, &timer_base->load);

	/*
	 * Start the A9Timer device
	 * Enable Auto reload mode, Clear prescaler control bits
	 * Set prescaler value, Enable the decrementer
	 */
	clrsetbits_le32(&timer_base->control, SCUTIMER_CONTROL_PRESCALER_MASK,
								emask);

	/* Reset time */
	gd->arch.lastinc = readl(&timer_base->counter) /
				(gd->arch.timer_rate_hz / CONFIG_SYS_HZ);
	gd->arch.tbl = 0;

	return 0;
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
ulong get_timer_masked(void)
{
	ulong now;

	now = readl(&timer_base->counter) /
			(gd->arch.timer_rate_hz / CONFIG_SYS_HZ);

	if (gd->arch.lastinc >= now) {
		/* Normal mode */
		gd->arch.tbl += gd->arch.lastinc - now;
	} else {
		/* We have an overflow ... */
		gd->arch.tbl += gd->arch.lastinc + TIMER_LOAD_VAL - now + 1;
	}
	gd->arch.lastinc = now;

	return gd->arch.tbl;
}

void __udelay(unsigned long usec)
{
	u32 countticks;
	u32 timeend;
	u32 timediff;
	u32 timenow;

	if (usec == 0)
		return;
	countticks = lldiv(((unsigned long long)gd->arch.timer_rate_hz * usec),
			   1000);

	/* decrementing timer */
	timeend = readl(&timer_base->counter) - countticks;

#if TIMER_LOAD_VAL != 0xFFFFFFFF
	/* do not manage multiple overflow */
	if (countticks >= TIMER_LOAD_VAL)
		countticks = TIMER_LOAD_VAL - 1;
#endif

	do {
		timenow = readl(&timer_base->counter);

		if (timenow >= timeend) {
			/* normal case */
			timediff = timenow - timeend;
		} else {
			if ((TIMER_LOAD_VAL - timeend + timenow) <=
								countticks) {
				/* overflow */
				timediff = TIMER_LOAD_VAL - timeend + timenow;
			} else {
				/* missed the exact match */
				break;
			}
		}
	} while (timediff > 0);
}

/* Timer without interrupts */
ulong get_timer(ulong base)
{
	return get_timer_masked() - base;
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return CONFIG_SYS_HZ;
}

#else
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
	return CONFIG_SYS_HZ;
}

static inline unsigned long long us_to_tick(unsigned long long usec)
{
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
	GTIMER_PTR gt = (GTIMER_PTR)(GTIMER0_ABSOLUTE_BASE);
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
	GTIMER_PTR gt = (GTIMER_PTR)(GTIMER0_ABSOLUTE_BASE);

	/* Reset and turn Global Timer off */
	writel(GT_CNTL_REG_RESET, &gt->ctl);
	/* Make timer tick at 1000*CONFIG_SYS_HZ rate */
	clrbits_le32(&gt->ctl,PRESCALER_BITMASK);
	setbits_le32(&gt->ctl,(lldiv((uint64_t)gd->cpu_clk/2, (uint32_t)1000*CONFIG_SYS_HZ) - 1)<< PRESCALER_SHIFT);
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
#endif
