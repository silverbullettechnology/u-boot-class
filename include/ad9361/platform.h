/***************************************************************************//**
 *   @file   platform.h
 *   @brief  Header file of Platform driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2014(c) Analog Devices, Inc.
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
#ifndef PLATFORM_H_
#define PLATFORM_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
//#include "stdint.h"
#include "util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define ADI_REG_CNTRL			0x0044
#define ADI_R1_MODE				(1 << 2)
#define ADI_DDR_EDGESEL			(1 << 1)
#define ADI_PIN_MODE			(1 << 0)

#define ADI_REG_STATUS			0x005C
#define ADI_MUX_PN_ERR			(1 << 3)
#define ADI_MUX_PN_OOS			(1 << 2)
#define ADI_MUX_OVER_RANGE		(1 << 1)
#define ADI_STATUS				(1 << 0)

#define ADI_REG_CHAN_CNTRL(c)	(0x0400 + (c) * 0x40)
#define ADI_PN_SEL				(1 << 10)
#define ADI_IQCOR_ENB			(1 << 9)
#define ADI_DCFILT_ENB			(1 << 8)
#define ADI_FORMAT_SIGNEXT		(1 << 6)
#define ADI_FORMAT_TYPE			(1 << 5)
#define ADI_FORMAT_ENABLE		(1 << 4)
#define ADI_PN23_TYPE			(1 << 1)
#define ADI_ENABLE				(1 << 0)

#define ADI_REG_CHAN_STATUS(c)	(0x0404 + (c) * 0x40)
#define ADI_PN_ERR				(1 << 2)
#define ADI_PN_OOS				(1 << 1)
#define ADI_OVER_RANGE			(1 << 0)

#define ADI_REG_CHAN_CNTRL_1(c)		(0x0410 + (c) * 0x40)
#define ADI_DCFILT_OFFSET(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_DCFILT_OFFSET(x)		(((x) >> 16) & 0xFFFF)
#define ADI_DCFILT_COEFF(x)			(((x) & 0xFFFF) << 0)
#define ADI_TO_DCFILT_COEFF(x)		(((x) >> 0) & 0xFFFF)

#define ADI_REG_CHAN_CNTRL_2(c)		(0x0414 + (c) * 0x40)
#define ADI_IQCOR_COEFF_1(x)		(((x) & 0xFFFF) << 16)
#define ADI_TO_IQCOR_COEFF_1(x)		(((x) >> 16) & 0xFFFF)
#define ADI_IQCOR_COEFF_2(x)		(((x) & 0xFFFF) << 0)
#define ADI_TO_IQCOR_COEFF_2(x)		(((x) >> 0) & 0xFFFF)

enum adc_pn_sel {
	ADC_PN9 = 0,
	ADC_PN23A = 1,
	ADC_PN7 = 4,
	ADC_PN15 = 5,
	ADC_PN23 = 6,
	ADC_PN31 = 7,
	ADC_PN_CUSTOM = 9,
	ADC_PN_END = 10,
};

enum adc_data_sel {
	ADC_DATA_SEL_NORM,
	ADC_DATA_SEL_LB, /* DAC loopback */
	ADC_DATA_SEL_RAMP, /* TBD */
};

#define ASFE_AD1_TX1_PA_BIAS		0x00000001
#define ASFE_AD1_TX2_PA_BIAS		0x00000002
#define ASFE_AD2_TX1_PA_BIAS		0x00000004
#define ASFE_AD2_TX2_PA_BIAS		0x00000008

#define ASFE_AD1_RX1_LNA			0x00000010
#define ASFE_AD1_RX2_LNA			0x00000020
#define ASFE_AD2_RX1_LNA			0x00000040
#define ASFE_AD2_RX2_LNA			0x00000080

#define ASFE_AD1_TR_SWITCH			0x00000100
#define ASFE_AD2_TR_SWITCH			0x00000200

#define GPIO_AD1_TX1_PA_EN			GPIO33_6
#define GPIO_AD1_TX2_PA_EN			GPIO33_2
#define GPIO_AD2_TX1_PA_EN			GPIO33_4
#define GPIO_AD2_TX2_PA_EN			GPIO33_1



#define GPO_ADX_RX1_LNA_BYPASS		GPO_0
#define GPO_ADX_RX2_LNA_BYPASS		GPO_1
#define GPO_AD2_TR_N				GPO_3

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
int32_t platform_spi_init(uint32_t device_id,
				 	 	 uint8_t  clk_pha,
				 	 	 uint8_t  clk_pol);
int32_t platform_spi_read(uint8_t *data,
				 	 	 uint8_t bytes_number);
int platform_spi_write_then_read(struct spi_device *spi,
				const unsigned char *txbuf, unsigned n_tx,
				unsigned char *rxbuf, unsigned n_rx);
void platform_gpio_init(uint32_t device_id);
void platform_gpio_direction(uint8_t pin, uint8_t direction);
bool platform_gpio_is_valid(int number);
void platform_gpio_set_value(unsigned gpio, int value);
void platform_gpio_set_sync_value(int value);
void platform_init_sync_pulse_shape(void);

void platform_udelay(unsigned long usecs);
void platform_mdelay(unsigned long msecs);
unsigned long platform_msleep_interruptible(unsigned int msecs);
void platform_axiadc_init(struct ad9361_rf_phy *phy);
unsigned int platform_axiadc_read(struct axiadc_state *st, unsigned long reg);
void platform_axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val);
void platform_pa_bias_en(uint32_t mask);
void platform_pa_bias_dis(uint32_t mask);
void platform_lna_dis(uint32_t mask);
void platform_lna_en(uint32_t mask);
void platform_tr_rx_en(uint32_t mask);
void platform_tr_tx_en(uint32_t mask);

#endif
