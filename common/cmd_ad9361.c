/*
 * (C) Copyright 2002
 * Gerald Van Baren, Custom IDEAS, vanbaren@cideas.com
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * SPI Read/Write Utilities
 */

#include <common.h>
#include <command.h>
#include <spi.h>
#include <malloc.h>
#include <linux/string.h>
#include <ad9361/ad9361.h>
#include <ad9361/ad9361_api.h>
#include <ad9361/command.h>
#include <ad9361/console.h>

/*-----------------------------------------------------------------------
 * Definitions
 */

#ifndef MAX_SPI_BYTES
#   define MAX_SPI_BYTES 32	/* Maximum number of bytes we can handle */
#endif

#ifndef CONFIG_DEFAULT_SPI_BUS
#   define CONFIG_DEFAULT_SPI_BUS	0
#endif
#ifndef CONFIG_DEFAULT_SPI_MODE
#   define CONFIG_DEFAULT_SPI_MODE	SPI_MODE_0
#endif

/*
 * Values from last command.
 */
static unsigned int bus = 0;
static unsigned int cs = 0;
static unsigned int mode = 0;
//static int   		bitlen;
//static uchar 		dout[MAX_SPI_BYTES];
//static uchar 		din[MAX_SPI_BYTES];

static struct ad9361_rf_phy* ad9361_phy_table[CONFIG_AD9361_MAX_DEVICE] = {
NULL,
NULL,
NULL,
NULL };

/*
 * SPI read/write
 *
 * Syntax:
 *   spi {dev} {num_bits} {dout}
 *     {dev} is the device number for controlling chip select (see TBD)
 *     {num_bits} is the number of bits to send & receive (base 10)
 *     {dout} is a hexadecimal string of data to send
 * The command prints out the hexadecimal string received via SPI.
 */

int do_ad9361(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
	struct spi_slave *slave;
	char *cp = 0;
	uchar tmp;
	char *command_line = NULL;
	int i, len;
	int rcode = 0;
	int cmd = 0;
	int invalid_cmd = 0;
	double param[5] = { 0, 0, 0, 0, 0 };
	char param_no = 0;
	int cmd_type = -1;

	/*
	 * We use the last specified parameters, unless new ones are
	 * entered.
	 */

	if ((flag & CMD_FLAG_REPEAT) == 0) {

		if (argc > 2) {
			mode = CONFIG_DEFAULT_SPI_MODE;
			bus = simple_strtoul(argv[1], &cp, 10);
			/*
			 * Allocate space for command line
			 */
			len = 0;

			for (i = 2; i < argc; i++) {
				len += strlen(argv[i]);
			}

			command_line = calloc(len + argc, sizeof(char));
			if (!command_line) {
				printf("%s - Memory allocation failed!!!\n", argv[0]);
				return 1;
			}

			len = 0;
			for (i = 2; i < argc; i++) {
				strcat(command_line, argv[i]);
				strcat(command_line, " ");
			}

		} else if (argc == 2) {
			len = strlen(argv[1]);
			command_line = calloc(len + argc, sizeof(char));
			if (!command_line) {
				printf("%s - Memory allocation failed!!!\n", argv[0]);
				return 1;
			}
			strcat(command_line, argv[1]);

		} else {
			len = strlen("help?");
			command_line = calloc(len + argc, sizeof(char));
			if (!command_line) {
				printf("%s - Memory allocation failed!!!\n", argv[0]);
				return 1;
			}
			strcat(command_line, "help?");

		}

		strcat(command_line, "\n");

		/*
		 * Init AD9361 chip if needed
		 */

		if (bus >= CONFIG_AD9361_MAX_DEVICE) {
			printf("Invalid RFIC: %d\n", bus);
			rcode = 1;
		} else {
			invalid_cmd = 0;
			for (cmd = 0; cmd < cmd_no; cmd++) {
				param_no = 0;
				cmd_type = console_check_commands(command_line,
						cmd_list[cmd].name, param, &param_no);
				if (cmd_type == UNKNOWN_CMD) {
					invalid_cmd++;
				} else {
					break;
				}
			}

			if (invalid_cmd == cmd_no) {
				printf("Invalid ad9361 command: %s\n", command_line);
				rcode = 1;
			}

		}
	}

	if (0 == rcode) {

		slave = spi_setup_slave(bus, cs, 1000000, mode);
		if (!slave) {
			printf("Invalid device %d:%d\n", bus, cs);
			rcode = 1;
		} else {

			spi_claim_bus(slave);
			/*
			 * Init RFIC if needed
			 */
			if (NULL != ad9361_phy_table[bus]) {
				ad9361_phy_table[bus] = ad9361_init(&default_init_param,
						(struct spi_device*) slave);
				/* Initialize RESETB pin */
				ad9361_phy_table[bus]->pdata->gpio_resetb = RESETB0_BITMASK << (bus * 8);

			}

			/* Initialize spi device pointer */
			ad9361_phy_table[bus]->spi = (struct spi_device*) slave;

			cmd_list[cmd].function(param, param_no);

			spi_release_bus(slave);
			spi_free_slave(slave);

		}

	}

	if (command_line)
		free(command_line);
	return rcode;
}

/***************************************************/

U_BOOT_CMD(ad9361, 2, 1, do_ad9361, "AD9361 utility command",
		"[<RFIC no>] <command> - Send and receive commands\n" "<RFIC no>             - Identifies the AD9361 IC number\n" "<command>             - Identifies the command\n" "Type ad9361 help!       for available commands\n");
