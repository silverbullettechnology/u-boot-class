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
#include <asm/arch/s3ma-regs.h>
#include <asm/arch/sys_proto.h>
#include <ad9361/platform.h>
#include <ad9361/ad9361_api.h>
#include <ad9361/util.h>

#ifndef MAX_SPI_BYTES
#   define MAX_SPI_BYTES 32	/* Maximum number of bytes we can handle */
#endif

#ifndef CONFIG_DEFAULT_SPI_BUS
#   define CONFIG_DEFAULT_SPI_BUS	0
#endif
#ifndef CONFIG_DEFAULT_SPI_MODE
#   define CONFIG_DEFAULT_SPI_MODE	SPI_MODE_1
#endif

struct ad9361_rf_phy* ad9361_phy_table[CONFIG_AD9361_MAX_DEVICE] = {
NULL,
NULL,
#ifndef CONFIG_SP3DTC
NULL,
NULL
#endif
};



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
	struct spi_slave * slave;
	uint32_t bus = spi->dev.bus, cs = 0, mode = CONFIG_DEFAULT_SPI_MODE;

	debug("%s:%d: SPI setup: bus = %d, cs = %d, mode = %d\n", __func__, __LINE__, bus, cs, mode);

	slave = spi_setup_slave(bus, cs, 1000000, mode);
	spi_claim_bus(slave);

    if( (txbuf != NULL) && (n_tx > 0) ){
     rcode = spi_xfer(slave, n_tx*sizeof(*txbuf)*8, txbuf, NULL,
    					SPI_XFER_BEGIN | SPI_XFER_END);
    }

    if( (rxbuf != NULL) && (n_rx > 0) && (rcode == 0) ){
        rcode = spi_xfer(slave, n_rx*sizeof(*rxbuf)*8,NULL, rxbuf,
       					SPI_XFER_BEGIN | SPI_XFER_END);
    }

	spi_release_bus(slave);
	spi_free_slave(slave);

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
	platform_mdelay(msecs);
	return 0;
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
void platform_axiadc_init(struct ad9361_rf_phy *phy)
{

	uint32_t bus = phy->spi->dev.bus;

	uint32_t val = 0;
	uint32_t addr = 0;

	/*
	 * Enable RF interface clocks
	 */
	clrbits_le32(CRT_RF_DIS, WFE_DIS_BITMASK);

	/*
	 *	Enable RFIC interface I/O pads
	 */
	/*
	 * RF_IO_CTLx allows you to assert the PWRDN  and EXT_REF pins on the IO cells for each of the RFIC busses
	 * Neither the RF_DriveX nor RF_IO_CTLx registers are initialized on a software reset.
	 */
	/*
	 * Bit0: RX_CM_EMF: LVDS receiver active high common mode enforcement'
	 * Bit1: RX_REB:    LVDS receiver active high power down'
	 * Bit2: TX_OEB:    LVDS driver active low output enable'
	 * Bit3: TX_ENB:    LVDS driver active high power down'
	 *
	 */
	debug("%s: Bus # 0x%x\n", __func__, bus);

	addr = (RF_IO_CTL0);

	addr += 4*bus;
	val = platform_axiadc_read(NULL,addr);
	val &= ~(0|RX_REB_BITMASK|RX_OEB_BITMASK|RX_ENB_BITMASK);
	val |= RX_CM_EMF_BITMASK;
	platform_axiadc_write(NULL,addr,val);


	/*
	 * For RF_DriveX LVDS drive strength mode: 0 = low current, 1 = high current
	 */
	addr = (RF_DRIVE0);

	addr += 4*bus;
	platform_axiadc_write(NULL,addr,0);

/*
 *	Turn off RFIC RX/TX by driving control pins low
 */
	val = (ENABLE0_BITMASK | TXNRX0_BITMASK) << (ENABLE1_SHIFT - ENABLE0_SHIFT)*bus;
	platform_axiadc_write(NULL, (RF_CONTROL_RESET), val);

/*
 * 	Turn off RX/TX in RF_CONFIG register
 */

	platform_axiadc_write(NULL,(RF_CONFIG),0);

/*
 *	clear all enable bits for RX/TX time slots
 */

	platform_axiadc_write(NULL,(RF_CHANNEL_EN),0);

	/* Multiplex time slots sequentially onto LVDS ports*/
	platform_axiadc_write(NULL, (TX_SEL),0x76543210);

	/* Make sure TX and RX shifts are initialized*/
	platform_axiadc_write(NULL, (AD_FORMAT),0xb4b4b4b4);
	platform_axiadc_write(NULL, (TX_TDM_FORMAT),0x44444444);
	/* Select TX source for all time slots*/
	platform_axiadc_write(NULL, TX_SOURCE, 0x55555555);

}
/***************************************************************************//**
 * @brief axiadc_post_setup
*******************************************************************************/
int platform_axiadc_post_setup(struct ad9361_rf_phy *phy)
{
	return 0;
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
		s3ma_gpio33_set_value(GPIO_AD1_TX1_PA_EN, 1);

	if(ASFE_AD1_TX2_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD1_TX2_PA_EN, 1);

	if(ASFE_AD2_TX1_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD2_TX1_PA_EN, 1);

	if(ASFE_AD2_TX2_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD2_TX2_PA_EN, 1);
}
/***************************************************************************//**
 * @brief platform_dis_pa_bias
*******************************************************************************/
void platform_pa_bias_dis(uint32_t mask)
{
	if(ASFE_AD1_TX1_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD1_TX1_PA_EN, 0);

	if(ASFE_AD1_TX2_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD1_TX2_PA_EN, 0);

	if(ASFE_AD2_TX1_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD2_TX1_PA_EN, 0);

	if(ASFE_AD2_TX2_PA_BIAS & mask)
		s3ma_gpio33_set_value(GPIO_AD2_TX2_PA_EN, 0);
}

/***************************************************************************//**
* @brief axiadc_set_pnsel
*******************************************************************************/
int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	return 0;
}


/***************************************************************************//**
 * @brief platform_lna_dis
*******************************************************************************/
void platform_lna_dis(uint32_t mask)
{
	struct ad9361_rf_phy *phy;

	phy = ad9361_phy_table[0];

	if(ASFE_AD1_RX1_LNA & mask)
		ad9361_gpo_set(phy, GPO_ADX_RX1_LNA_BYPASS);

	if(ASFE_AD1_RX2_LNA & mask)
		ad9361_gpo_set(phy, GPO_ADX_RX2_LNA_BYPASS);

	phy = ad9361_phy_table[1];

	if(ASFE_AD2_RX1_LNA & mask)
		ad9361_gpo_set(phy, GPO_ADX_RX1_LNA_BYPASS);

	if(ASFE_AD2_RX2_LNA & mask)
		ad9361_gpo_set(phy,GPO_ADX_RX2_LNA_BYPASS);
}

/***************************************************************************//**
 * @brief platform_lna_en
*******************************************************************************/
void platform_lna_en(uint32_t mask)
{
	struct ad9361_rf_phy *phy;

	phy = ad9361_phy_table[0];

	if(ASFE_AD1_RX1_LNA & mask)
		ad9361_gpo_clear(phy,GPO_ADX_RX1_LNA_BYPASS);

	if(ASFE_AD1_RX2_LNA & mask)
		ad9361_gpo_clear(phy,GPO_ADX_RX2_LNA_BYPASS);

	phy = ad9361_phy_table[1];

	if(ASFE_AD2_RX1_LNA & mask)
		ad9361_gpo_clear(phy,GPO_ADX_RX1_LNA_BYPASS);

	if(ASFE_AD2_RX2_LNA & mask)
		ad9361_gpo_clear(phy,GPO_ADX_RX2_LNA_BYPASS);
}

/***************************************************************************//**
 * @brief platform_tr_rx_en
*******************************************************************************/
void platform_tr_rx_en(uint32_t mask)
{
	struct ad9361_rf_phy *phy;

	phy = ad9361_phy_table[0];

	if(ASFE_AD1_TR_SWITCH & mask)
		ad9361_gpo_clear(phy,GPO_ADX_TR_N);

	phy = ad9361_phy_table[1];

	if(ASFE_AD2_TR_SWITCH & mask)
		ad9361_gpo_clear(phy,GPO_ADX_TR_N);
}

/***************************************************************************//**
 * @brief platform_tr_tx_en
*******************************************************************************/
void platform_tr_tx_en(uint32_t mask)
{
	struct ad9361_rf_phy *phy;

	phy = ad9361_phy_table[0];

	if(ASFE_AD1_TR_SWITCH & mask)
		ad9361_gpo_set(phy, GPO_ADX_TR_N);

	phy = ad9361_phy_table[1];

	if(ASFE_AD2_TR_SWITCH & mask)
		ad9361_gpo_set(phy, GPO_ADX_TR_N);
}

void platform_asfe_init(void)
{
	/* Reset ASFE */
	s3ma_gpio33_set_value(GPIO_ASFE0_RESET,0);
	s3ma_gpio33_set_value(GPIO_ASFE1_RESET,0);
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

