/*
 * Copyright 2014 Exelis , Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <config.h>
#include <command.h>
#include <asm/io.h>
#include <serial.h>
#include <linux/compiler.h>
#include <asm/arch/clock.h>
#include <asm/arch/ddr.h>
#include <asm/arch/sys_proto.h>

static int do_ddr_init(cmd_tbl_t *cmdtp, int flag, int argc,
			 char * const argv[])
{
	char* str_cmd = NULL;

	if (argc < 2){
		return CMD_RET_USAGE;
	}

	str_cmd = argv[1];

	if (!strcmp(str_cmd, "on")) {
		s3ma_ddr_clock_enable();
		s3ma_ddr_setup();

	}
	else if(!strcmp(str_cmd, "off")){
		s3ma_ddr_clock_disable();
	}
	else{
		return CMD_RET_USAGE;

	}
	
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	ddr, 2, 0, do_ddr_init,
	"turn DDR interface on/off ",
	"on | off"
);

