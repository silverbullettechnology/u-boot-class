/*
 * Copyright 2014 Exelis , Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <serial.h>
#include <asm/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

static int do_timer(cmd_tbl_t *cmdtp, int flag, int argc,
			 char * const argv[])
{
	char  *cp = 0;
	uint32_t gpio, value = 0;

	if (argc < 2){
		return CMD_RET_USAGE;
	}

	gpio = simple_strtoul(argv[1], &cp, 10);

	gpio_direction_output(gpio, value);

	debug("CPU rate is %ld HZ \n", gd->cpu_clk);
	debug("Timer rate per HZ is %ld\n", gd->arch.timer_rate_hz);

	while(!ctrlc()) {
		value = !value;
		gpio_direction_output(gpio, value);
		mdelay(1000);
	}
	
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	timer, 2, 0, do_timer,
	"toggle <pin> every 1 second",
	"<pin>"
);

