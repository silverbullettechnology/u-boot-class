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
#include <asm/arch/s3ma-regs.h>

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
			   1000000);

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
