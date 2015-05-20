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

static int do_uart_test(cmd_tbl_t *cmdtp, int flag, int argc,
			 char * const argv[])
{
	struct serial_device * serial_dev = secondary_serial_console();
	char c;

	serial_dev->start();
	printf("Connect to other UART and begin typing\n");
	printf("Hit Enter key to end this test\n");
	for(;;){
		while(!serial_dev->tstc());
		c = serial_dev->getc();
		if(('\r' == c) || ('\n' == c))
			break;
		else
			serial_dev->putc(c);
	};
	
	return 0;
}

U_BOOT_CMD(
	uartrx, 1, 0, do_uart_test,
	"test S3MA secondary uart ",
	""
);

