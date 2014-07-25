#ifndef __PL061_GPIO_H
#define __PL061_GPIO_H

#define PINS_PER_BANK	8

enum gpio_direction {
	GPIO_DIRECTION_IN,
	GPIO_DIRECTION_OUT,
};

struct gpio_regs {
	uint8_t gpiodata[0x400];	/* 0x000 ... 0x3fc */
	uint8_t gpiodir;			/* 0x400 */
};

#define DATA_REG_ADDR(gpio)		(1 << (gpio + 2))

#endif	/* __PL061_GPIO_H */
