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
#include <asm/arch/s3ma-regs.h>
#include <ad9361/ad9361.h>
#include <ad9361/ad9361_api.h>
#include <ad9361/command.h>
#include <ad9361/console.h>
#include <ad9361/platform.h>


/*-----------------------------------------------------------------------
 * Definitions
 */






int do_ad9361(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
	char *cp = 0;
	char *command_line = NULL;
	int i, len;
	int rcode = 0;
	int cmd = 0;
	int invalid_cmd = 0;
	double param[5] = { 0, 0, 0, 0, 0 };
	char param_no = 0;
	int cmd_type = -1;
	unsigned int bus = 0;
	AD9361_InitParam * init_param_ptr = NULL;
	/*
	 * We use the last specified parameters, unless new ones are
	 * entered.
	 */

	if ((flag & CMD_FLAG_REPEAT) == 0) {

		if (argc > 2) {
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

			/*
			 * Init RFIC if needed
			 */
#ifdef	CONFIG_SP3DTC
		i = bus;
		bus = 0;
		while(bus < CONFIG_AD9361_MAX_DEVICE)
		{

#endif
			if (NULL == ad9361_phy_table[bus]) {

				init_param_ptr = malloc(sizeof(AD9361_InitParam));

				if(init_param_ptr)
				{
					memcpy(init_param_ptr, &default_init_param, sizeof(AD9361_InitParam));

					/* Save bus number */
					init_param_ptr->id_no = bus;
					/* Initialize RESETB pin */
					init_param_ptr->gpio_resetb = RESETB0_BITMASK << (bus* 8);

#ifdef CONFIG_SP3DTC
					/* Initialize RESETB pin */
					init_param_ptr->gpio_resetb = RESETB0_BITMASK << (((bus+1) & 0x1) * 8);
					if(1 == bus)
					{
						ulong env;
						/* Initialize the DAC setting from saved env variable */
						env = getenv_ulong("dac", 10, 0xFFFFFFFF);
						if(env == 0xFFFFFFFF)
						{
							env = 0;
							setenv_ulong("dac", env);
							saveenv();
						}

						init_param_ptr->aux_dac1_default_value_mV = env;
					 }
#endif
					ad9361_init(&ad9361_phy_table[bus],init_param_ptr);
					free(init_param_ptr);


				}
				else{
					rcode = 1;
#ifdef CONFIG_SP3DTC
					break;
#endif
				}
			}
#ifdef CONFIG_SP3DTC
			bus++;
		}

		bus = i;
#endif
		if(0 == rcode)
		{
			if(bus < CONFIG_AD9361_MAX_DEVICE){
				if(NULL != ad9361_phy_table[bus]){
					ad9361_phy = ad9361_phy_table[bus];
	#ifdef DEBUG
					debug("Param_no = %d\n", param_no);
					for(i = 0; i < param_no; i++)
					{
						debug("Param %d = %d\n", i, (int32_t)param[i]);
					}
	#endif
					cmd_list[cmd].function(param, param_no);
				}

			}
			else{
				rcode = 1;
			}

		}

	}

	if (command_line)
		free(command_line);

	return rcode;
}

/***************************************************/

U_BOOT_CMD(ad9361, 10, 1, do_ad9361, "AD9361 utility command",
		"[<RFIC no>] <command> - Send and receive commands\n" "<RFIC no>             - Identifies the AD9361 IC number\n" "<command>             - Identifies the command\n" "Type ad9361 help?       for available commands\n");
