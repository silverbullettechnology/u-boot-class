/*
 * Copyright (C) 2012 Stefan Roese <sr@denx.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Driver for SPEAr600 GPIO controller
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/pl061.h>
#include <asm/io.h>
#include <errno.h>

static int gpio_direction(unsigned gpio,
			  enum gpio_direction direction)
{
	struct gpio_regs *regs = (struct gpio_regs *)CONFIG_GPIO_BASE;
	u32 val,addr;

	addr = (u32)&regs->gpiodir;
	val = readl(addr);

	if (direction == GPIO_DIRECTION_OUT)
		val |= 1 << gpio;
	else
		val &= ~(1 << gpio);
	addr = (u32)&regs->gpiodir;
	writel(val, addr);

	return 0;
}

int gpio_set_value(unsigned gpio, int value)
{
	struct gpio_regs *regs = (struct gpio_regs *)CONFIG_GPIO_BASE;
	u32 addr = (u32)&regs->gpiodata[DATA_REG_ADDR(gpio)];
	writel(1 << gpio, addr);

	return 0;
}

int gpio_get_value(unsigned gpio)
{
	struct gpio_regs *regs = (struct gpio_regs *)CONFIG_GPIO_BASE;
	u32 val, addr;

	addr = (u32)&regs->gpiodata[DATA_REG_ADDR(gpio)];
	val = readl(addr);

	return !!val;
}

int gpio_request(unsigned gpio, const char *label)
{
	int ret, bank;
//	int io;

//	io = gpio % PINS_PER_BANK;
	bank = gpio / PINS_PER_BANK;

	switch(bank){
#ifdef CONFIG_GPIO_BASE
	case 0:
		ret = 0;
		break;
#endif
#ifdef CONFIG_GPIO_BASE1
	case 1:
		ret = 0;
		break;
#endif
#ifdef CONFIG_GPIO_BASE2
	case 2:
		ret = 0;
		break;
#endif
#ifdef CONFIG_GPIO_BASE3
	case 3:
		ret = 0;
		break;
#endif

	default:
		ret = -EINVAL;

	}

	return ret;
}

int gpio_free(unsigned gpio)
{
	return 0;
}

void gpio_toggle_value(unsigned gpio)
{
	gpio_set_value(gpio, !gpio_get_value(gpio));
}

int gpio_direction_input(unsigned gpio)
{
	return gpio_direction(gpio, GPIO_DIRECTION_IN);
}

int gpio_direction_output(unsigned gpio, int value)
{
	int ret = gpio_direction(gpio, GPIO_DIRECTION_OUT);

	if (ret < 0)
		return ret;

	gpio_set_value(gpio, value);
	return 0;
}
