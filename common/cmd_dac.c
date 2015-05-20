/*
 * Command for controlling external DAC
 *
 * Copyright 2014 Exelis , Inc.
 *
 */

#include <common.h>
#include <command.h>
#include <dac.h>

static int do_spi_dac(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[])
{
	const char *cmd;
	int ret = -1;
	ulong value;
	char *env;

	/* need at least two arguments */
	if (argc > 1)
	{
		cmd = argv[1];

		if (strcmp(cmd, "setenv") == 0)
		{
			ret = setenv_ulong("dac", spi_dac_get());
			if(ret == 0)
			{
				ret = saveenv();
			}
		}
		else if (strcmp(cmd, "getenv") == 0)
		{
			env = getenv("dac");
			if(env)
			{
				printf("%s\n", env);
				ret = 0;
			}
			else
			{
				printf("Not found\n");
				ret = 1;
			}
		}
		else if (strcmp(cmd, "set") == 0)
		{
			if (argc > 2)
			{
				value = simple_strtoul(argv[2], NULL, 10);
				ret = spi_dac_set(value);
			}
		}
		else if (strcmp(cmd, "get") == 0)
		{
			value = spi_dac_get();
			printf("%ld\n", value);
			ret = 0;
		}

	}

	return ret;
}

U_BOOT_CMD(
	dac, 3, 1, do_spi_dac,
	"SPI DAC control",
	"set <output> - Set DAC register <output> (0 - 1023)\n"
	"dac get - Return DAC register output\n"
	"dac setenv - Save DAC output to env variable\n"
	"dac getenv - Return DAC output from env variable\n"
);
