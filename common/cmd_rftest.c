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
#include <ad9361/platform.h>

/*-----------------------------------------------------------------------
 * Definitions
 */




int do_rf_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *cp = 0;
	uint32_t 	bus = 0;
	uint8_t	    *test_buf = (uint8_t*)CONFIG_AD9361_RAM_BUFFER_ADDR;
	uint32_t	num_samples = 8*32;
	uint32_t	i,j;
	uint32_t	adc_rotate = 0;
	//	uint16_t	pattern[] = {0x0000, 0xffff, 0xaaaa, 0x5555};
	uint16_t	pattern[] = {0xaaaa, 0xaaaa, 0xaaaa, 0xaaaa};
	//	uint16_t	pattern[] = {0xffff, 0xffff, 0xffff, 0xffff};
	int32_t		status = 0;
	uint32_t    val;
	int	rcode = 0;

	if ((flag & CMD_FLAG_REPEAT) == 0) {

		if (argc > 1) {
			bus = simple_strtoul(argv[1], &cp, 10);
			platform_axiadc_init(NULL);
#if 1
			adc_rotate = platform_axiadc_read(NULL, (AD_FORMAT));
			debug("original adc_rotate value = %x\n", adc_rotate);
			adc_rotate = (adc_rotate >> (8 * bus)) & ROTATE0_BITMASK;
			debug("adc_rotate value after shift= %x\n", adc_rotate);
#endif
			do{
				for (i = 0; i < ARRAY_SIZE((pattern)); i++)
				{
					/* Fill TX buffer with pattern
					 * TODO:
					 * It's unclear from ASIC spec which way TX samples are going to be shifted
					 * ADC samples are rotated left, so it would be logical to assume that
					 * TX samples are shifted right
					 */
					uint16_t	*tx_ptr = (uint16_t*)&test_buf[0];
					uint16_t	*rx_ptr = (uint16_t*)&test_buf[CONFIG_AD9361_RAM_BUFFER_SIZE];
#if 1
					memset(&test_buf[0], 0xbb, CONFIG_S3MA_OCM_RAM_SIZE);

					for(j = 0; j < num_samples*2; j++)
					{
						tx_ptr[j] = pattern[i];
					}
					console_print("Trying pattern %x\n", pattern[i]);
#else
					memset(&test_buf[0], 0x0, CONFIG_S3MA_OCM_RAM_SIZE);
					memset(&test_buf[0], 0xaa, CONFIG_AD9361_RAM_BUFFER_SIZE);
#endif

					/* Setup TX DMA registers */
					val = (uint32_t)tx_ptr;
					val -= CONFIG_AD9361_RAM_BUFFER_ADDR;
					debug("RF_READ_BASE setting to %x\n",val);
					platform_axiadc_write(NULL, RF_READ_BASE, val);
					debug("RF_READ_BASE is %x\n",platform_axiadc_read(NULL, RF_READ_BASE));

					val += num_samples*sizeof(uint32_t);
					debug("RF_READ_TOP setting to %x\n",val);
					platform_axiadc_write(NULL, RF_READ_TOP, val);
					debug("RF_READ_TOP is %x\n",platform_axiadc_read(NULL, RF_READ_TOP));

					platform_axiadc_write(NULL, RF_READ_COUNT, 0xffffffff);
					debug("RF_READ_COUNT is %x\n",platform_axiadc_read(NULL, RF_READ_COUNT));

					/* Setup RX DMA registers */
					val = (uint32_t)rx_ptr;
					val -= CONFIG_AD9361_RAM_BUFFER_ADDR;

					debug("RF_WRITE_BASE setting to %x\n",val);
					platform_axiadc_write(NULL, RF_WRITE_BASE, (uint32_t)val);
					debug("RF_WRITE_BASE is %x\n",platform_axiadc_read(NULL,RF_WRITE_BASE));

					val += num_samples*sizeof(uint32_t);
					debug("RF_WRITE_TOP setting to %x\n",val);
					platform_axiadc_write(NULL, RF_WRITE_TOP, val);
					debug("RF_WRITE_TOP is %x\n",platform_axiadc_read(NULL,RF_WRITE_TOP));

					platform_axiadc_write(NULL, RF_WRITE_COUNT, 8*num_samples);
					debug("RF_WRITE_COUNT is %x\n",platform_axiadc_read(NULL,RF_WRITE_COUNT));

					/* Select both channels for TX and RX*/
#if 0
					platform_axiadc_write(NULL, RF_CHANNEL_EN, ((0x3 << RX_CH_ENABLE_SHIFT)|(0x3 << TX_CH_ENABLE_SHIFT)) << (2*bus));
#else
					platform_axiadc_write(NULL, RF_CHANNEL_EN, 0xffff);
#endif
					debug("RF_CHANNEL_EN = 0x%x\n",platform_axiadc_read(NULL,RF_CHANNEL_EN));

					/* Select TX source */
					platform_axiadc_write(NULL, TX_SOURCE, 0x55555555);
					debug("TX_SOURCE = 0x%x\n",platform_axiadc_read(NULL,TX_SOURCE));

					/* Enable transfer */
					platform_axiadc_write(NULL, RF_CONFIG, RF_CONFIG_RX_ENABLE_BITMASK|RF_CONFIG_TX_ENABLE_BITMASK);

					/* Wait for transfer to complete  */
					while(platform_axiadc_read(NULL,RF_WRITE_COUNT_AXI) > 0x0)
					{
						/* TODO:May want to add a timeout check here in case transfer never completes */
					}

					/* Disable transfer */
					platform_axiadc_write(NULL, RF_CONFIG, 0);
					platform_axiadc_write(NULL,(RF_CHANNEL_EN),0);

					status = 0;

#if 1
					for(j = 0; j < num_samples*2; j++)
					{
						if((tx_ptr[j] & 0x0fff) != (rx_ptr[j] >> adc_rotate))
						{
							//    				console_print("Error in sample %d: tx_sample= 0x%x rx_sample= 0x%x\n", j/2, tx_ptr[j] >> adc_rotate, rx_ptr[j] >> adc_rotate);
							status = 1;
						}
					}
					if(status)
					{
						console_print("Pattern %x test: FAIL\n", pattern[i]);
					}
					else
					{
						console_print("Pattern %x test: PASS\n", pattern[i]);

					}
#endif
				}

//			}while(!ctrlc());
			}while(0);

		}
		else{
			rcode = CMD_RET_USAGE;
		}

	}
	return rcode;
}
	/***************************************************/

U_BOOT_CMD( rftest, 2, 1, do_rf_test,
			"RF interface digital loopback",
			"[<RFIC no>]  Digital loopback on interface <RFIC no>\n "
			"\n");
