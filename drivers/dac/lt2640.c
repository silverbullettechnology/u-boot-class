/*
 * Driver for LT2640 SPI DAC
 */

#include <common.h>
#include <spi.h>
#include <dac.h>

#ifndef CONFIG_LT2640_DAC_SPEED
#define CONFIG_LT2640_DAC_SPEED		(1000000)
#endif
#ifndef CONFIG_LT2640_DAC_MODE
#define CONFIG_LT2640_DAC_MODE		SPI_MODE_0
#endif
#ifndef CONFIG_LT2640_DAC_CS
#define CONFIG_LT2640_DAC_CS		(23)
#endif
#ifndef CONFIG_LT2640_DAC_BUS
#define CONFIG_LT2640_DAC_BUS		(4)
#endif
#ifndef CONFIG_LT2640_DAC_DEFAULT
#define CONFIG_LT2640_DAC_DEFAULT	(0)
#endif

#define DAC_CMD_CODE_WRITE			0x00
#define DAC_CMD_CODE_UPDATE			0x10
#define DAC_CMD_CODE_WRITE_UPDATE	0x30
#define DAC_CMD_CODE_POWER_DOWN		0x40
#define DAC_CMD_CODE_INT_REF		0x60
#define DAC_CMD_CODE_EXT_REF		0x70

#define DAC_DATA_MAX_OUTPUT			1023

static ulong dacOutput = 0;

int spi_dac_set(ulong output)
{
	struct spi_slave *slave;
	int ret = 0;
	u8 cmd[3];

	if (output > DAC_DATA_MAX_OUTPUT)
	{
		ret = -1;
	}
	else
	{
		cmd[2] = (u8)((output << 6) & 0xFF);
		cmd[1] = (u8)((output >> 8) & 0xFF);
		cmd[0] = DAC_CMD_CODE_WRITE_UPDATE;

		slave = spi_setup_slave(CONFIG_LT2640_DAC_BUS,
								CONFIG_LT2640_DAC_CS,
								CONFIG_LT2640_DAC_SPEED,
								CONFIG_LT2640_DAC_MODE);

		if (!slave)
		{
			ret = 1;
		}
		else
		{
			spi_claim_bus(slave);
			if (spi_xfer(slave, 24, cmd, NULL, SPI_XFER_BEGIN | SPI_XFER_END) != 0)
			{
				ret = 1;
			}
			else
			{
				dacOutput = output;
				ret = 0;
			}
			spi_release_bus(slave);
			spi_free_slave(slave);
		}
	}

	return ret;
}

ulong spi_dac_get(void)
{
	return dacOutput;
}
