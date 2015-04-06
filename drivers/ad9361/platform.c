/***************************************************************************//**
 *   @file   Platform.c
 *   @brief  Implementation of Platform Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
//#include "stdint.h"
#include <common.h>
#include <spi.h>
#include <asm/io.h>
#include <ad9361/platform.h>
#include <ad9361/ad9361_api.h>
#include <ad9361/util.h>


/***************************************************************************//**
 * @brief usleep
*******************************************************************************/
static inline void usleep(unsigned long usleep)
{

}

/***************************************************************************//**
 * @brief spi_init
*******************************************************************************/
int32_t platform_spi_init(uint32_t device_id,
				 uint8_t  clk_pha,
				 uint8_t  clk_pol)
{
	return 0;
}

/***************************************************************************//**
 * @brief spi_read
*******************************************************************************/
int32_t platform_spi_read(uint8_t *data,
				 uint8_t bytes_number)
{
	return 0;
}

/***************************************************************************//**
 * @brief spi_write_then_read
*******************************************************************************/
int platform_spi_write_then_read(struct spi_device *spi,
		const unsigned char *txbuf, unsigned n_tx,
		unsigned char *rxbuf, unsigned n_rx)
{
	int rcode = 0;
	struct spi_slave * slave = (struct spi_slave *)spi;

	if(0 != spi_xfer(slave, n_tx*sizeof(*txbuf), txbuf, rxbuf,
					SPI_XFER_BEGIN | SPI_XFER_END))
	{
		rcode = 1;
	}

	return rcode;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
void platform_gpio_init(uint32_t device_id)
{

}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
void platform_gpio_direction(uint8_t pin, uint8_t direction)
{

}

/***************************************************************************//**
 * @brief gpio_is_valid
*******************************************************************************/
bool platform_gpio_is_valid(int number)
{
	return 1;
}
/***************************************************************************//**
 * @brief gpio_data
*******************************************************************************/
void platform_gpio_data(uint8_t pin, uint8_t data)
{
}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
void platform_gpio_set_value(unsigned gpio, int value)
{
	if(0 == value){
		writel(gpio, (RF_CONTROL_RESET));
	}
	else{
		writel(gpio, (RF_CONTROL_SET));

	}
}

/***************************************************************************//**
 * @brief gpio_set_sync_value
*******************************************************************************/
void platform_gpio_set_sync_value(int value)
{
	if(0 == value)
	{
		writel(TRIGGER_RESET, RF_SYNC);
	}
	else
	{
		writel(TRIGGER_BITMASK, RF_SYNC);
	}

}

/***************************************************************************//**
 * @brief init_sync_pulse_shape
*******************************************************************************/
void platform_init_sync_pulse_shape(void)
{
	writel(0xffffffff, RF_SYNCPULSESHAPE);
}
/***************************************************************************//**
 * @brief udelay
*******************************************************************************/
void platform_udelay(unsigned long usecs)
{
	udelay(usecs);
}

/***************************************************************************//**
 * @brief mdelay
*******************************************************************************/
void platform_mdelay(unsigned long msecs)
{
	mdelay(msecs);
}

/***************************************************************************//**
 * @brief msleep_interruptible
*******************************************************************************/
unsigned long platform_msleep_interruptible(unsigned int msecs)
{

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
void platform_axiadc_init(struct ad9361_rf_phy *phy)
{
	struct spi_slave *slave = (struct spi_slave *)phy->spi;
	uint32_t bus = slave->bus;
	uint32_t val = 0;
	uint32_t addr = 0;
/*
 *	Enable RFIC interface I/O pads
 */
	/*
	 * RF_IO_CTLx allows you to assert the PWRDN  and EXT_REF pins on the IO cells for each of the RFIC busses
	 * Neither the RF_DriveX nor RF_IO_CTLx registers are initialized on a software reset.
	 */

	/*TODO: There is a discrepancy in the ASIC specification and include file. Correct bit names ?
	 * Does _ENB mean we need to write 0 to activate or it's just a signal name?
	 */

	addr = (RF_IO_CTL0) + bus*sizeof(val);
	val  = (uint32_t)1 << RX_ENB_SHIFT;
	val |= (uint32_t)1 << RX_OEB_SHIFT;
	val |= (uint32_t)1 << RX_REB_SHIFT;
	val |= (uint32_t)1 << RX_CM_EMF_SHIFT;
	platform_axiadc_write(NULL,addr,val);

	/*TODO: There is nothing in the ASIC specification about how to control the drive strength
	 * or what all fields mean. Let's just write all zeros
	 */
	addr = (RF_DRIVE0);
	addr += bus*sizeof(val);
	val = 0;
	platform_axiadc_write(NULL,addr,val);

/*
 *	Turn off RFIC RX/TX by driving control pins low
 */

	val = (ENABLE0_BITMASK | TXNRX0_BITMASK) << (ENABLE1_SHIFT - ENABLE0_SHIFT)*bus;
	platform_axiadc_write(NULL,(RF_CONTROL_RESET),val);


/*
 * 	Turn off RX/TX in RF_CONFIG register
 */
	val = ~(RF_CONFIG_RX_ENABLE_BITMASK|RF_CONFIG_TX_ENABLE_BITMASK|RX_INTERRUPT_EN_BITMASK|TX_INTERRUPT_EN_BITMASK);

	platform_axiadc_write(NULL,val,(RF_CONFIG));

/*
 *	clear enable bits for associated RX/TX Channels
 */
	val = ((1<< 2*bus)|(1 << (2*bus+1))) << RX_CH_ENABLE_SHIFT;
	val |= val << TX_CH_ENABLE_SHIFT;
	val = ~val;
	platform_axiadc_write(NULL,(RF_CHANNEL_EN),val);
/*
 * 	Initialize ADC sign bit location and shift
 * 	Keep the defaults for now
 */
	val = (12 << SIGN_BIT0_SHIFT)|(4 << ROTATE0_SHIFT);
	val <<= (SIGN_BIT1_SHIFT - SIGN_BIT0_SHIFT)*bus;
	platform_axiadc_write(NULL,(AD_FORMAT),val);

}

/***************************************************************************//**
 * @brief axiadc_read
*******************************************************************************/
unsigned int platform_axiadc_read(struct axiadc_state *st, unsigned long reg)
{
	return readl(reg);
}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
void platform_axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	writel(val, reg);
}

#ifdef CONFIG_SP3DTC

/***************************************************************************//**
 * @brief platform_en_pa_bias
*******************************************************************************/
void platform_pa_bias_en(uint32_t mask)
{
	if(ASFE_AD1_TX1_PA_BIAS & mask)
		gpio_set_value(GPIO_AD1_TX1_PA_EN, 1);
	else if(ASFE_AD1_TX2_PA_BIAS & mask)
		gpio_set_value(GPIO_AD1_TX2_PA_EN, 1);
	else if(ASFE_AD2_TX1_PA_BIAS & mask)
		gpio_set_value(GPIO_AD2_TX1_PA_EN, 1);
	else if(ASFE_AD2_TX2_PA_BIAS & mask)
		gpio_set_value(GPIO_AD2_TX2_PA_EN, 1);
	else
		printf("%s: Invalid PA control IO\n", __func__);
}
/***************************************************************************//**
 * @brief platform_dis_pa_bias
*******************************************************************************/
void platform_pa_bias_dis(uint32_t mask)
{
	if(ASFE_AD1_TX1_PA_BIAS & mask)
		gpio_set_value(GPIO_AD1_TX1_PA_EN, 0);
	else if(ASFE_AD1_TX2_PA_BIAS & mask)
		gpio_set_value(GPIO_AD1_TX2_PA_EN, 0);
	else if(ASFE_AD2_TX1_PA_BIAS & mask)
		gpio_set_value(GPIO_AD2_TX1_PA_EN, 0);
	else if(ASFE_AD2_TX2_PA_BIAS & mask)
		gpio_set_value(GPIO_AD2_TX2_PA_EN, 0);
	else
		printf("%s: Invalid PA control IO\n", __func__);
}

/***************************************************************************//**
 * @brief platform_lna_dis
*******************************************************************************/
void platform_lna_dis(uint32_t mask)
{
	if((ASFE_AD1_RX1_LNA|ASFE_AD2_RX1_LNA) & mask)
		ad9361_gpo_set(ad9361_phy,GPO_ADX_RX1_LNA_BYPASS);
	else if((ASFE_AD1_RX2_LNA|ASFE_AD2_RX2_LNA) & mask)
		ad9361_gpo_set(ad9361_phy,GPO_ADX_RX2_LNA_BYPASS);
	else
		printf("%s: Invalid LNA control IO\n", __func__);
}

/***************************************************************************//**
 * @brief platform_lna_en
*******************************************************************************/
void platform_lna_en(uint32_t mask)
{
	if((ASFE_AD1_RX1_LNA|ASFE_AD2_RX1_LNA) & mask)
		ad9361_gpo_clear(ad9361_phy,GPO_ADX_RX1_LNA_BYPASS);
	else if((ASFE_AD1_RX2_LNA|ASFE_AD2_RX2_LNA) & mask)
		ad9361_gpo_clear(ad9361_phy,GPO_ADX_RX2_LNA_BYPASS);
	else
		printf("%s: Invalid LNA control IO\n", __func__);
}

/***************************************************************************//**
 * @brief platform_tr_rx_en
*******************************************************************************/
void platform_tr_rx_en(uint32_t mask)
{
	if(ASFE_AD2_TR_SWITCH & mask)
		ad9361_gpo_clear(ad9361_phy,GPO_AD2_TR_N);
	else if(ASFE_AD1_TR_SWITCH & mask)
		ad9361_gpo_clear(ad9361_phy,GPO_AD2_TR_N);
	else
		printf("%s: Invalid TR switch control IO\n", __func__);
}

/***************************************************************************//**
 * @brief platform_tr_tx_en
*******************************************************************************/
void platform_tr_tx_en(uint32_t mask)
{
	if(ASFE_AD2_TR_SWITCH & mask)
		ad9361_gpo_set(ad9361_phy,GPO_AD2_TR_N);
	else if(ASFE_AD1_TR_SWITCH & mask)
		ad9361_gpo_set(ad9361_phy,GPO_AD2_TR_N);
	else
		printf("%s: Invalid TR switch control IO\n", __func__);
}

void platform_asfe_init(void)
{
	platform_pa_bias_dis(0|ASFE_AD1_TX1_PA_BIAS|ASFE_AD1_TX2_PA_BIAS|ASFE_AD2_TX1_PA_BIAS|ASFE_AD2_TX2_PA_BIAS);
	platform_lna_dis(0|ASFE_AD1_RX1_LNA|ASFE_AD1_RX2_LNA|ASFE_AD2_RX1_LNA|ASFE_AD2_RX2_LNA);

}

#else
void platform_pa_bias_en(uint32_t mask){}
void platform_pa_bias_dis(uint32_t mask){}
void platform_lna_dis(uint32_t mask){}
void platform_lna_en(uint32_t mask){}
void platform_tr_rx_en(uint32_t mask){}
void platform_tr_tx_en(uint32_t mask){}
void platform_asfe_init(void){}

#endif /* CONFIG_SP3DTC */

