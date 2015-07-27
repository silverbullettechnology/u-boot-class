/**************************************************************************//**
 *   @file   command.c
 *   @brief  Implementation of AD9361 Command Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 *******************************************************************************
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
#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <asm/arch/s3ma-regs.h>
#include <ad9361/command.h>
#include <ad9361/console.h>
#include <ad9361/ad9361.h>
#include <ad9361/ad9361_api.h>
#include <ad9361/platform.h>

/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/
command cmd_list[] = {
	{"help?", "Displays all available commands.", "", get_help},
	{"register?", "Gets the specified register value.", "", get_register},
	{"register=", "Sets the specified register value.", "", set_register},
	{"aux_adc?", "Gets the AUX ADC value.", "", get_aux_adc},
	{"aux_dac?", "Gets the AUX DAC1 or DAC2 value [mV].", "", get_aux_dac},
	{"aux_dac=", "Sets the AUX DAC1 or DAC2 value [mV].", "", set_aux_dac},
	{"tx_lo_freq?", "Gets current TX LO frequency [MHz].", "", get_tx_lo_freq},
	{"tx_lo_freq=", "Sets the TX LO frequency [MHz].", "", set_tx_lo_freq},
	{"tx_samp_freq?", "Gets current TX sampling frequency [Hz].", "", get_tx_samp_freq},
	{"tx_samp_freq=", "Sets the TX sampling frequency [Hz].", "", set_tx_samp_freq},
	{"tx_rf_bandwidth?", "Gets current TX RF bandwidth [Hz].", "", get_tx_rf_bandwidth},
	{"tx_rf_bandwidth=", "Sets the TX RF bandwidth [Hz].", "", set_tx_rf_bandwidth},
	{"tx1_attenuation?", "Gets current TX1 attenuation [mdB].", "", get_tx1_attenuation},
	{"tx1_attenuation=", "Sets the TX1 attenuation [mdB].", "", set_tx1_attenuation},
	{"tx2_attenuation?", "Gets current TX2 attenuation [mdB].", "", get_tx2_attenuation},
	{"tx2_attenuation=", "Sets the TX2 attenuation [mdB].", "", set_tx2_attenuation},
	{"tx_fir_en?", "Gets current TX FIR state.", "", get_tx_fir_en},
	{"tx_fir_en=", "Sets the TX FIR state.", "", set_tx_fir_en},
	{"rx_lo_freq?", "Gets current RX LO frequency [MHz].", "", get_rx_lo_freq},
	{"rx_lo_freq=", "Sets the RX LO frequency [MHz].", "", set_rx_lo_freq},
	{"rx_samp_freq?", "Gets current RX sampling frequency [Hz].", "", get_rx_samp_freq},
	{"rx_samp_freq=", "Sets the RX sampling frequency [Hz].", "", set_rx_samp_freq},
	{"rx_rf_bandwidth?", "Gets current RX RF bandwidth [Hz].", "", get_rx_rf_bandwidth},
	{"rx_rf_bandwidth=", "Sets the RX RF bandwidth [Hz].", "", set_rx_rf_bandwidth},
	{"rx1_gc_mode?", "Gets current RX1 GC mode.", "", get_rx1_gc_mode},
	{"rx1_gc_mode=", "Sets the RX1 GC mode.", "", set_rx1_gc_mode},
	{"rx2_gc_mode?", "Gets current RX2 GC mode.", "", get_rx2_gc_mode},
	{"rx2_gc_mode=", "Sets the RX2 GC mode.", "", set_rx2_gc_mode},
	{"rx1_rf_gain?", "Gets current RX1 RF gain.", "", get_rx1_rf_gain},
	{"rx1_rf_gain=", "Sets the RX1 RF gain.", "", set_rx1_rf_gain},
	{"rx2_rf_gain?", "Gets current RX2 RF gain.", "", get_rx2_rf_gain},
	{"rx2_rf_gain=", "Sets the RX2 RF gain.", "", set_rx2_rf_gain},
	{"rx_fir_en?", "Gets current RX FIR state.", "", get_rx_fir_en},
	{"rx_fir_en=", "Sets the RX FIR state.", "", set_rx_fir_en},
	{"tx_loopback_test=","Runs DAC->ADC loopback test.","",tx_loopback_test},
	{"asic_loopback_test=","Enables/Disables ADC->DAC loopback test.","",set_asfe_loopback_test},
	{"bist_loopback_en=","Enables/Disables DAC->ADC data ports loopback.","",bist_loopback},

#if 0
	{"dds_tx1_tone1_freq?", "Gets current DDS TX1 Tone 1 frequency [Hz].", "", get_dds_tx1_tone1_freq},
	{"dds_tx1_tone1_freq=", "Sets the DDS TX1 Tone 1 frequency [Hz].", "", set_dds_tx1_tone1_freq},
	{"dds_tx1_tone2_freq?", "Gets current DDS TX1 Tone 2 frequency [Hz].", "", get_dds_tx1_tone2_freq},
	{"dds_tx1_tone2_freq=", "Sets the DDS TX1 Tone 2 frequency [Hz].", "", set_dds_tx1_tone2_freq},
	{"dds_tx1_tone1_phase?", "Gets current DDS TX1 Tone 1 phase [degrees].", "", get_dds_tx1_tone1_phase},
	{"dds_tx1_tone1_phase=", "Sets the DDS TX1 Tone 1 phase [degrees].", "", set_dds_tx1_tone1_phase},
	{"dds_tx1_tone2_phase?", "Gets current DDS TX1 Tone 2 phase [degrees].", "", get_dds_tx1_tone2_phase},
	{"dds_tx1_tone2_phase=", "Sets the DDS TX1 Tone 2 phase [degrees].", "", set_dds_tx1_tone2_phase},
	{"dds_tx1_tone1_scale?", "Gets current DDS TX1 Tone 1 scale.", "", get_dds_tx1_tone1_scale},
	{"dds_tx1_tone1_scale=", "Sets the DDS TX1 Tone 1 scale.", "", set_dds_tx1_tone1_scale},
	{"dds_tx1_tone2_scale?", "Gets current DDS TX1 Tone 2 scale.", "", get_dds_tx1_tone2_scale},
	{"dds_tx1_tone2_scale=", "Sets the DDS TX1 Tone 2 scale.", "", set_dds_tx1_tone2_scale},
	{"dds_tx2_tone1_freq?", "Gets current DDS TX2 Tone 1 frequency [Hz].", "", get_dds_tx2_tone1_freq},
	{"dds_tx2_tone1_freq=", "Sets the DDS TX2 Tone 1 frequency [Hz].", "", set_dds_tx2_tone1_freq},
	{"dds_tx2_tone2_freq?", "Gets current DDS TX2 Tone 2 frequency [Hz].", "", get_dds_tx2_tone2_freq},
	{"dds_tx2_tone2_freq=", "Sets the DDS TX2 Tone 2 frequency [Hz].", "", set_dds_tx2_tone2_freq},
	{"dds_tx2_tone1_phase?", "Gets current DDS TX2 Tone 1 phase [degrees].", "", get_dds_tx2_tone1_phase},
	{"dds_tx2_tone1_phase=", "Sets the DDS TX2 Tone 1 phase [degrees].", "", set_dds_tx2_tone1_phase},
	{"dds_tx2_tone2_phase?", "Gets current DDS TX2 Tone 2 phase [degrees].", "", get_dds_tx2_tone2_phase},
	{"dds_tx2_tone2_phase=", "Sets the DDS TX2 Tone 2 phase [degrees].", "", set_dds_tx2_tone2_phase},
	{"dds_tx2_tone1_scale?", "Gets current DDS TX2 Tone 1 scale.", "", get_dds_tx2_tone1_scale},
	{"dds_tx2_tone1_scale=", "Sets the DDS TX2 Tone 1 scale.", "", set_dds_tx2_tone1_scale},
	{"dds_tx2_tone2_scale?", "Gets current DDS TX2 Tone 2 scale.", "", dds_tx2_tone2_scale},
	{"dds_tx2_tone2_scale=", "Sets the DDS TX2 Tone 2 scale.", "", set_dds_tx2_tone2_scale},
#endif
};
const char cmd_no = (sizeof(cmd_list) / sizeof(command));

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/

/**************************************************************************//***
 * @brief Show the invalid parameter message.
 *
 * @return None.
*******************************************************************************/
void show_invalid_param_message(unsigned char cmd_no)
{
	console_print("Invalid parameter!\n");
	console_print("%s  - %s\n", (char*)cmd_list[cmd_no].name, (char*)cmd_list[cmd_no].description);
	console_print("Example: %s\n", (char*)cmd_list[cmd_no].example);
}

/**************************************************************************//***
 * @brief Displays all available commands.
 *
 * @return None.
*******************************************************************************/
void get_help(double* param, char param_no) // "help?" command
{
	unsigned char display_cmd;

	console_print("Available commands:\n");
	for(display_cmd = 0; display_cmd < cmd_no; display_cmd++)
	{
		console_print("%s  - %s\n", (char*)cmd_list[display_cmd].name,
								  (char*)cmd_list[display_cmd].description);
	}
}

/**************************************************************************//***
 * @brief Gets the specified register value.
 *
 * @return None.
*******************************************************************************/
void get_register(double* param, char param_no) // "register?" command
{
	uint16_t reg_addr;
	uint8_t reg_val;

	if(param_no >= 1)
	{
		reg_addr = (uint16_t)param[0];
		reg_val = ad9361_spi_read(ad9361_phy->spi, reg_addr);
		console_print("register[0x%x]=0x%x\n", reg_addr, reg_val);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Sets the specified register value.
 *
 * @return None.
*******************************************************************************/
void set_register(double* param, char param_no)
{
	uint16_t reg_addr;
	uint8_t reg_val;

	if(param_no >= 2)
	{
		reg_addr = (uint16_t)param[0];
		reg_val = (uint8_t)param[1];
		ad9361_spi_write(ad9361_phy->spi, reg_addr, reg_val);
		console_print("register[0x%x]=0x%x\n", reg_addr, reg_val);
	}
	else
		show_invalid_param_message(1);

}
/**************************************************************************//***
 * @brief Gets current TX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void get_tx_lo_freq(double* param, char param_no) // "tx_lo_freq?" command
{
	uint64_t lo_freq_hz;

	ad9361_get_tx_lo_freq(ad9361_phy, &lo_freq_hz);
	lo_freq_hz /= 1000000;
	console_print("tx_lo_freq=%d MHz\n", (uint32_t)lo_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the TX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void set_tx_lo_freq(double* param, char param_no) // "tx_lo_freq=" command
{
	uint64_t lo_freq_hz;

	if(param_no >= 1)
	{
		lo_freq_hz = param[0];
		lo_freq_hz *= 1000000;
		ad9361_set_tx_lo_freq(ad9361_phy, lo_freq_hz);
		lo_freq_hz /= 1000000;
		console_print("tx_lo_freq=%d MHz\n", (uint32_t)lo_freq_hz);
	}
}

/**************************************************************************//***
 * @brief Gets current sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_tx_samp_freq(double* param, char param_no) // "tx_samp_freq?" command
{
	uint32_t sampling_freq_hz;

	ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	console_print("tx_samp_freq=%d\n", sampling_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_tx_samp_freq(double* param, char param_no) // "tx_samp_freq=" command
{
	uint32_t sampling_freq_hz;

	if(param_no >= 1)
	{
		sampling_freq_hz = (uint32_t)param[0];
		ad9361_set_tx_sampling_freq(ad9361_phy, sampling_freq_hz);
		ad9361_get_tx_sampling_freq(ad9361_phy, &sampling_freq_hz);
#if 0
		dds_update();
#endif
		console_print("tx_samp_freq=%d Hz\n", sampling_freq_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void get_tx_rf_bandwidth(double* param, char param_no) // "tx_rf_bandwidth?" command
{
	uint32_t bandwidth_hz;

	ad9361_get_tx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	console_print("tx_rf_bandwidth=%d Hz\n", bandwidth_hz);
}

/**************************************************************************//***
 * @brief Sets the TX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void set_tx_rf_bandwidth(double* param, char param_no) // "tx_rf_bandwidth=" command
{
	uint32_t bandwidth_hz;

	if(param_no >= 1)
	{
		bandwidth_hz = param[0];
		ad9361_set_tx_rf_bandwidth(ad9361_phy, bandwidth_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX1 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void get_tx1_attenuation(double* param, char param_no) // "tx1_attenuation?" command
{
	uint32_t attenuation_mdb;

	ad9361_get_tx_attenuation(ad9361_phy, 0, &attenuation_mdb);
	console_print("tx1_attenuation=%d\n", attenuation_mdb);
}

/**************************************************************************//***
 * @brief Sets the TX1 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void set_tx1_attenuation(double* param, char param_no) // "tx1_attenuation=" command
{
	uint32_t attenuation_mdb;

	if(param_no >= 1)
	{
		attenuation_mdb = param[0];
		ad9361_set_tx_attenuation(ad9361_phy, 0, attenuation_mdb);
		console_print("tx1_attenuation=%d\n", attenuation_mdb);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX2 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void get_tx2_attenuation(double* param, char param_no) // "tx1_attenuation?" command
{
	uint32_t attenuation_mdb;

	ad9361_get_tx_attenuation(ad9361_phy, 1, &attenuation_mdb);
	console_print("tx2_attenuation=%d\n", attenuation_mdb);
}

/**************************************************************************//***
 * @brief Sets the TX2 attenuation [mdB].
 *
 * @return None.
*******************************************************************************/
void set_tx2_attenuation(double* param, char param_no) // "tx1_attenuation=" command
{
	uint32_t attenuation_mdb;

	if(param_no >= 1)
	{
		attenuation_mdb = param[0];
		ad9361_set_tx_attenuation(ad9361_phy, 1, attenuation_mdb);
		console_print("tx2_attenuation=%d\n", attenuation_mdb);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current TX FIR state.
 *
 * @return None.
*******************************************************************************/
void get_tx_fir_en(double* param, char param_no) // "tx_fir_en?" command
{
	uint8_t en_dis;

	ad9361_get_tx_fir_en_dis(ad9361_phy, &en_dis);
	console_print("tx_fir_en=%d\n", en_dis);
}

/**************************************************************************//***
 * @brief Sets the TX FIR state.
 *
 * @return None.
*******************************************************************************/
void set_tx_fir_en(double* param, char param_no) // "tx_fir_en=" command
{
	uint8_t en_dis;

	if(param_no >= 1)
	{
		en_dis = param[0];
		ad9361_set_tx_fir_en_dis(ad9361_phy, en_dis);
		console_print("tx_fir_en=%d\n", en_dis);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void get_rx_lo_freq(double* param, char param_no) // "rx_lo_freq?" command
{
	uint64_t lo_freq_hz;

	ad9361_get_rx_lo_freq(ad9361_phy, &lo_freq_hz);
	lo_freq_hz /= 1000000;
	console_print("rx_lo_freq=%d MHz\n", (uint32_t)lo_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the RX LO frequency [MHz].
 *
 * @return None.
*******************************************************************************/
void set_rx_lo_freq(double* param, char param_no) // "rx_lo_freq=" command
{
	uint64_t lo_freq_hz;

	if(param_no >= 1)
	{
		lo_freq_hz = param[0];
		lo_freq_hz *= 1000000;
		ad9361_set_rx_lo_freq(ad9361_phy, lo_freq_hz);
		lo_freq_hz /= 1000000;
		console_print("rx_lo_freq=%d MHz\n", (uint32_t)lo_freq_hz);
	}
}

/**************************************************************************//***
 * @brief Gets current RX sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_rx_samp_freq(double* param, char param_no) // "rx_samp_freq?" command
{
	uint32_t sampling_freq_hz;

	ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);
	console_print("rx_samp_freq=%d Hz\n", sampling_freq_hz);
}

/**************************************************************************//***
 * @brief Sets the RX sampling frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_rx_samp_freq(double* param, char param_no) // "rx_samp_freq=" command
{
	uint32_t sampling_freq_hz;

	if(param_no >= 1)
	{
		sampling_freq_hz = (uint32_t)param[0];
		ad9361_set_rx_sampling_freq(ad9361_phy, sampling_freq_hz);
		ad9361_get_rx_sampling_freq(ad9361_phy, &sampling_freq_hz);
#if 0
		dds_update();
#endif
		console_print("rx_samp_freq=%d Hz\n", sampling_freq_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void get_rx_rf_bandwidth(double* param, char param_no) // "rx_rf_bandwidth?" command
{
	uint32_t bandwidth_hz;

	ad9361_get_rx_rf_bandwidth(ad9361_phy, &bandwidth_hz);
	console_print("rx_rf_bandwidth=%d Hz\n", bandwidth_hz);
}

/**************************************************************************//***
 * @brief Sets the RX RF bandwidth [Hz].
 *
 * @return None.
*******************************************************************************/
void set_rx_rf_bandwidth(double* param, char param_no) // "rx_rf_bandwidth=" command
{
	uint32_t bandwidth_hz;

	if(param_no >= 1)
	{
		bandwidth_hz = param[0];
		ad9361_set_rx_rf_bandwidth(ad9361_phy, bandwidth_hz);
		console_print("rx_rf_bandwidth=%d Hz\n", bandwidth_hz);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX1 GC mode.
 *
 * @return None.
*******************************************************************************/
void get_rx1_gc_mode(double* param, char param_no) // "rx1_gc_mode?" command
{
	uint8_t gc_mode;

	ad9361_get_rx_gain_control_mode(ad9361_phy, 0, &gc_mode);
	console_print("rx1_gc_mode=%d\n", gc_mode);
}

/**************************************************************************//***
 * @brief Sets the RX1 GC mode.
 *
 * @return None.
*******************************************************************************/
void set_rx1_gc_mode(double* param, char param_no) // "rx1_gc_mode=" command
{
	uint8_t gc_mode;

	if(param_no >= 1)
	{
		gc_mode = param[0];
		ad9361_set_rx_gain_control_mode(ad9361_phy, 0, gc_mode);
		console_print("rx1_gc_mode=%d\n", gc_mode);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX2 GC mode.
 *
 * @return None.
*******************************************************************************/
void get_rx2_gc_mode(double* param, char param_no) // "rx2_gc_mode?" command
{
	uint8_t gc_mode;

	ad9361_get_rx_gain_control_mode(ad9361_phy, 1, &gc_mode);
	console_print("rx2_gc_mode=%d\n", gc_mode);
}

/**************************************************************************//***
 * @brief Sets the RX2 GC mode.
 *
 * @return None.
*******************************************************************************/
void set_rx2_gc_mode(double* param, char param_no) // "rx2_gc_mode=" command
{
	uint8_t gc_mode;

	if(param_no >= 1)
	{
		gc_mode = param[0];
		ad9361_set_rx_gain_control_mode(ad9361_phy, 1, gc_mode);
		console_print("rx2_gc_mode=%d\n", gc_mode);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX1 RF gain.
 *
 * @return None.
*******************************************************************************/
void get_rx1_rf_gain(double* param, char param_no) // "rx1_rf_gain?" command
{
	int32_t gain_db;

	ad9361_get_rx_rf_gain (ad9361_phy, 0, &gain_db);
	console_print("rx1_rf_gain=%d\n", gain_db);
}

/**************************************************************************//***
 * @brief Sets the RX1 RF gain.
 *
 * @return None.
*******************************************************************************/
void set_rx1_rf_gain(double* param, char param_no) // "rx1_rf_gain=" command
{
	int32_t gain_db;

	if(param_no >= 1)
	{
		gain_db = param[0];
		ad9361_set_rx_rf_gain (ad9361_phy, 0, gain_db);
		console_print("rx1_rf_gain=%d\n", gain_db);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX2 RF gain.
 *
 * @return None.
*******************************************************************************/
void get_rx2_rf_gain(double* param, char param_no) // "rx2_rf_gain?" command
{
	int32_t gain_db;

	ad9361_get_rx_rf_gain (ad9361_phy, 1, &gain_db);
	console_print("rx2_rf_gain=%d\n", gain_db);
}

/**************************************************************************//***
 * @brief Sets the RX2 RF gain.
 *
 * @return None.
*******************************************************************************/
void set_rx2_rf_gain(double* param, char param_no) // "rx2_rf_gain=" command
{
	int32_t gain_db;

	if(param_no >= 1)
	{
		gain_db = param[0];
		ad9361_set_rx_rf_gain (ad9361_phy, 1, gain_db);
		console_print("rx2_rf_gain=%d\n", gain_db);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current RX FIR state.
 *
 * @return None.
*******************************************************************************/
void get_rx_fir_en(double* param, char param_no) // "rx_fir_en?" command
{
	uint8_t en_dis;

	ad9361_get_rx_fir_en_dis(ad9361_phy, &en_dis);
	console_print("rx_fir_en=%d\n", en_dis);
}

/**************************************************************************//***
 * @brief Sets the RX FIR state.
 *
 * @return None.
*******************************************************************************/
void set_rx_fir_en(double* param, char param_no) // "rx_fir_en=" command
{
	uint8_t en_dis;

	if(param_no >= 1)
	{
		en_dis = param[0];
		ad9361_set_rx_fir_en_dis(ad9361_phy, en_dis);
		console_print("rx_fir_en=%d\n", en_dis);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Enables/disables DAC->ADC loopback test
 *
 * @return None.
*******************************************************************************/
void bist_loopback(double* param, char param_no)
{
	int32_t		status = 0;
	uint32_t    en_dis = 0;

	if(NULL == ad9361_phy)
	{
		console_print("%s: ad9361_phy structure is invalid\n",__func__);
		return;
	}

	if(param_no >= 1)
	{
		en_dis = param[0];
		/* Turn AD9361 loopback mode on */
		status = ad9361_bist_loopback(ad9361_phy, en_dis);
	}
	else
	{
		show_invalid_param_message(1);
		status = -1;
	}


	if(status)
	{
		console_print("%s: Failed to setup tx->rx loopback mode\n", __func__);
		return;
	}

	if(en_dis)
	{
		console_print("BIST loopback mode is on\n");

	}
	else
	{
		console_print("BIST loopback mode is off\n");

	}


}
/**************************************************************************//***
 * @brief Runs DAC->ADC loopback test
 *
 * @return None.
*******************************************************************************/
void tx_loopback_test(double* param, char param_no)
{
	uint32_t 	bus = 0;
	uint8_t	    *test_buf = (uint8_t*)CONFIG_AD9361_RAM_BUFFER_ADDR;
	uint32_t	num_samples = 32;
	uint32_t	i,j;
	uint32_t	adc_rotate = 0;
	uint32_t    tx_shift = 0;
	uint32_t	pattern[] = {0x00000000, 0xfff0fff0, 0xaaa0aaa0, 0x55505550,
							 0x82008200, 0x41004100, 0x20802080, 0x10401040,
							 0x08200820,0x04100410,0xaaa05550,0x12304560,0xded0bef0};
	int32_t		status = 0;
	uint32_t    val;

	if(NULL != ad9361_phy)
	{
		bus = ad9361_phy->spi->dev.bus;
	}
	else
	{
		console_print("%s: ad9361_phy structure is invalid\n",__func__);
		return;
	}

	adc_rotate = platform_axiadc_read(NULL, (AD_FORMAT));
	debug("original adc_rotate register value = %x\n", adc_rotate);
	adc_rotate = (adc_rotate >> (8 * bus)) & (ROTATE0_BITMASK);
	debug("adc_rotate value after shift= %x\n", adc_rotate);

	tx_shift = platform_axiadc_read(NULL, (TX_TDM_FORMAT));
	debug("original tx_shift register value = %x\n", tx_shift);
	tx_shift  = (tx_shift >> (8 * bus)) & (SLOT0_SHIFT_BITMASK);
	debug("tx_shift value = %x\n", tx_shift);

	/* Turn AD9361 loopback mode on */
	status = ad9361_bist_loopback(ad9361_phy, 1);
	if(status)
	{
		console_print("%s: Failed to setup tx->rx loopback mode\n", __func__);
		return;
	}
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

    		memset(&test_buf[0], 0xbb, CONFIG_S3MA_OCM_RAM_SIZE);

    		for(j = 0; j < num_samples; j++)
    		{
    			((uint32_t*)tx_ptr)[j] = pattern[i];
    		}
    		console_print("Trying pattern %x\n", pattern[i]);

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

    		platform_axiadc_write(NULL, RF_READ_COUNT, 8*num_samples);
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

    		platform_axiadc_write(NULL, RF_WRITE_COUNT, 4*num_samples);
    		debug("RF_WRITE_COUNT is %x\n",platform_axiadc_read(NULL,RF_WRITE_COUNT));

    		/* Select both channels for TX and RX*/
    		platform_axiadc_write(NULL,
    							 RF_CHANNEL_EN,
    				             ((0x3 << RX_CH_ENABLE_SHIFT)|(0x3 << TX_CH_ENABLE_SHIFT)) << (2*bus));

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

    		for(j = 0; j < num_samples*2; j++)
    		{
    			if((tx_ptr[j] >> tx_shift) != (rx_ptr[j] >> adc_rotate))
    			{
    				console_print("Error in sample %d: tx_sample= 0x%x rx_sample= 0x%x\n", j/2, tx_ptr[j] >> adc_rotate, rx_ptr[j] >> adc_rotate);
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
    	}

//    }while(!ctrlc());
    }while(0);

}
/**************************************************************************//***
 * @brief Enables/Disables ADC->DAC loopback test
 *
 * @return None.
*/
void set_asfe_loopback_test(double* param, char param_no)
{
	uint32_t *test_buf = (uint32_t*) CONFIG_AD9361_RAM_BUFFER_ADDR;
	uint32_t num_samples = 8192;
	uint32_t en_dis = (uint32_t) param[0];
	uint32_t bus = 0;
	uint32_t val = 0;

	if (param_no >= 1)
	{
		/* Disable any ongoing transfers first */
		platform_axiadc_write(NULL, RF_CONFIG, 0);
		platform_axiadc_write(NULL, (RF_CHANNEL_EN), 0);
#ifdef CONFIG_SP3DTC
		/* Turn all PAs off */
		platform_pa_bias_dis(0|ASFE_AD1_TX1_PA_BIAS|ASFE_AD1_TX2_PA_BIAS|ASFE_AD2_TX1_PA_BIAS|ASFE_AD2_TX2_PA_BIAS);
		/* Turn all LNAs off  */
		platform_lna_dis(ASFE_AD1_RX1_LNA | ASFE_AD1_RX2_LNA | ASFE_AD2_RX1_LNA | ASFE_AD2_RX2_LNA);
#endif
		if (NULL != ad9361_phy)
		{
			bus = ad9361_phy->spi->dev.bus;
		}
		else
		{
			console_print("%s: ad9361_phy structure is invalid\n", __func__);
			return;
		}

		debug("Bus = %d \n", bus);

		if (1 == en_dis)
		{
			double param[4];
			char param_no;
			/* Setup AD9361 */
			param[0] = 38400000;
			param_no = 1;
			set_tx_samp_freq(param, param_no);
			set_rx_samp_freq(param, param_no);

			param[0] = 10000000;
			param_no = 1;
			set_tx_rf_bandwidth(param, param_no);
			set_rx_rf_bandwidth(param, param_no);

			param[0] = 10000;
			set_tx1_attenuation(param, param_no);
			set_tx2_attenuation(param, param_no);

			param[0] = 0;
			set_rx1_rf_gain(param, param_no);
			set_rx2_rf_gain(param, param_no);

			if (0 == bus)
			{

				param[0] = 300.000000;
				set_tx_lo_freq(param, param_no);
				param[0] = 500.000000;
				set_rx_lo_freq(param, param_no);

#ifdef CONFIG_SP3DTC
				/* Setup ASFE for loopback */
				platform_lna_en(ASFE_AD1_RX1_LNA | ASFE_AD1_RX2_LNA);
				platform_pa_bias_en(ASFE_AD1_TX1_PA_BIAS | ASFE_AD1_TX2_PA_BIAS);
#endif
			}
			else
			{
				/* Setup AD9361 */
				param[0] = 2500.0000000;
				set_tx_lo_freq(param, param_no);
				param[0] = 2400.0000000;
				set_rx_lo_freq(param, param_no);
#ifdef CONFIG_SP3DTC
				/* Setup ASFE for loopback */
				platform_tr_rx_en(ASFE_AD2_TR_SWITCH);
				platform_lna_en(ASFE_AD2_RX1_LNA | ASFE_AD2_RX2_LNA);
				platform_pa_bias_en(ASFE_AD2_TX1_PA_BIAS | ASFE_AD2_TX2_PA_BIAS);
#endif
			}

			/* Setup digital ADC->DAC loopback */

			/* Setup TX DMA registers */
			val = (uint32_t) test_buf;
			val -= CONFIG_AD9361_RAM_BUFFER_ADDR;
			debug("RF_READ_BASE setting to %x\n", val);
			platform_axiadc_write(NULL, RF_READ_BASE, val);
			debug("RF_READ_BASE is %x\n",
					platform_axiadc_read(NULL, RF_READ_BASE));

			val += num_samples * sizeof(uint32_t);
			debug("RF_READ_TOP setting to %x\n", val);
			platform_axiadc_write(NULL, RF_READ_TOP, val);
			debug("RF_READ_TOP is %x\n",
					platform_axiadc_read(NULL, RF_READ_TOP));

			platform_axiadc_write(NULL, RF_READ_COUNT, 0xffffffff);
			debug("RF_READ_COUNT is %x\n",
					platform_axiadc_read(NULL, RF_READ_COUNT));

			val = (uint32_t) test_buf;
			val -= CONFIG_AD9361_RAM_BUFFER_ADDR;

			debug("RF_WRITE_BASE setting to %x\n", val);
			platform_axiadc_write(NULL, RF_WRITE_BASE, (uint32_t) val);
			debug("RF_WRITE_BASE is %x\n",
					platform_axiadc_read(NULL,RF_WRITE_BASE));

			val += num_samples * sizeof(uint32_t);
			debug("RF_WRITE_TOP setting to %x\n", val);
			platform_axiadc_write(NULL, RF_WRITE_TOP, val);
			debug("RF_WRITE_TOP is %x\n",
					platform_axiadc_read(NULL,RF_WRITE_TOP));

			platform_axiadc_write(NULL, RF_WRITE_COUNT, 0xffffffff);
			debug("RF_WRITE_COUNT is %x\n",
					platform_axiadc_read(NULL,RF_WRITE_COUNT));

			/* Select both channels for TX and RX*/
			platform_axiadc_write(NULL,
			RF_CHANNEL_EN,
					((0x3 << RX_CH_ENABLE_SHIFT) | (0x3 << TX_CH_ENABLE_SHIFT))
							<< (2 * bus));

			debug("RF_CHANNEL_EN = 0x%x\n",
					platform_axiadc_read(NULL,RF_CHANNEL_EN));

			/* Select TX source */
			platform_axiadc_write(NULL, TX_SOURCE, 0x55555555);
			debug("TX_SOURCE = 0x%x\n", platform_axiadc_read(NULL,TX_SOURCE));

			/* Transition RFIC into FDD mode */
			ad9361_get_en_state_machine_mode(ad9361_phy, &val);
			if(ENSM_STATE_ALERT != val)
			{
				ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_STATE_ALERT);
			}

			ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_STATE_FDD);

			/* Enable transfer */
			platform_axiadc_write(NULL, RF_CONFIG,
					RF_CONFIG_RX_ENABLE_BITMASK | RF_CONFIG_TX_ENABLE_BITMASK);

			/* Wait for transfer to complete or CTRL^C */
			while (!ctrlc())
			{
				if(platform_axiadc_read(NULL, RF_WRITE_COUNT_AXI) == 0x0)
				{
					/* Disable transfer */
					platform_axiadc_write(NULL, RF_CONFIG, 0);
				    /* Re-Enable transfer */
					platform_axiadc_write(NULL, RF_CONFIG,
							RF_CONFIG_RX_ENABLE_BITMASK | RF_CONFIG_TX_ENABLE_BITMASK);
				}

			}

		}

		/* Disable transfer */
		platform_axiadc_write(NULL, RF_CONFIG, 0);
		platform_axiadc_write(NULL, (RF_CHANNEL_EN), 0);
		ad9361_set_en_state_machine_mode(ad9361_phy, ENSM_STATE_ALERT);

#ifdef CONFIG_SP3DTC
		/* Disable PA and LNA */
		if(0 == bus)
		{
			platform_lna_dis(ASFE_AD1_RX1_LNA | ASFE_AD1_RX2_LNA);
			platform_pa_bias_dis(ASFE_AD1_TX1_PA_BIAS | ASFE_AD1_TX2_PA_BIAS);
		}
		else
		{
			platform_lna_dis(ASFE_AD2_RX1_LNA | ASFE_AD2_RX2_LNA);
			platform_pa_bias_dis(ASFE_AD2_TX1_PA_BIAS | ASFE_AD2_TX2_PA_BIAS);
		}
#endif
	}
	else
	{
		show_invalid_param_message(1);

	}

}



/**************************************************************************//***
 * @brief Gets current AUX ADC value.
 *
 * @return None.
*******************************************************************************/
void get_aux_adc(double* param, char param_no)
{
	int32_t	val;

	if(param_no >= 1)
	{
		val = ad9361_get_auxadc(ad9361_phy);
		console_print("AUX ADC value = 0x%x\n", val);
	}
	else
	{
		show_invalid_param_message(1);
	}
}

/**************************************************************************//***
 * @brief Gets current AUX DAC value.
 *
 * @return None.
*******************************************************************************/
void get_aux_dac(double* param, char param_no)
{
	if(param_no >= 1)
	{
		int32_t	val;
		val = ad9361_auxdac_get(ad9361_phy, param[0]);
		console_print("AUX DAC value = 0x%x\n", val);
	}
	else
	{
		show_invalid_param_message(1);
	}
}

/**************************************************************************//***
 * @brief Sets current AUX DAC value.
 *
 * @return None.
*******************************************************************************/
void set_aux_dac(double* param, char param_no)
{
	if(param_no >= 2)
	{
		ad9361_auxdac_set(ad9361_phy, param[0], param[1]);
		console_print("AUX DAC value = %d\n", (int32_t)param[1]);
	}
	else
	{
		show_invalid_param_message(1);
	}

}

#if 0
/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone1_freq(double* param, char param_no)	// dds_tx1_tone1_freq?
{
	uint32_t freq = dds_st.cached_freq[DDS_CHAN_TX1_I_F1];

	console_print("dds_tx1_tone1_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone1_freq(double* param, char param_no)	// dds_tx1_tone1_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_frequency(DDS_CHAN_TX1_I_F1, freq);
		dds_set_frequency(DDS_CHAN_TX1_Q_F1, freq);
		console_print("dds_tx1_tone1_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone2_freq(double* param, char param_no)	// dds_tx1_tone2_freq?
{
	uint32_t freq = dds_st.cached_freq[DDS_CHAN_TX1_I_F2];

	console_print("dds_tx1_tone2_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone2_freq(double* param, char param_no)	// dds_tx1_tone2_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_frequency(DDS_CHAN_TX1_I_F2, freq);
		dds_set_frequency(DDS_CHAN_TX1_Q_F2, freq);
		console_print("dds_tx1_tone2_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone1_phase(double* param, char param_no)	// dds_tx1_tone1_phase?
{
	uint32_t phase = dds_st.cached_phase[DDS_CHAN_TX1_I_F1];

	phase /= 1000;
	console_print("dds_tx1_tone1_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone1_phase(double* param, char param_no)	// dds_tx1_tone1_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_phase(DDS_CHAN_TX1_I_F1, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		dds_set_phase(DDS_CHAN_TX1_Q_F1, (uint32_t)((phase - 90) * 1000));
		phase = dds_st.cached_phase[DDS_CHAN_TX1_I_F1] / 1000;
		console_print("dds_tx1_tone1_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone2_phase(double* param, char param_no)	// dds_tx1_tone2_phase?
{
	uint32_t phase = dds_st.cached_phase[DDS_CHAN_TX1_I_F2];

	phase /= 1000;
	console_print("dds_tx1_tone2_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone2_phase(double* param, char param_no)	// dds_tx1_tone2_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_phase(DDS_CHAN_TX1_I_F2, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		dds_set_phase(DDS_CHAN_TX1_Q_F2, (uint32_t)((phase - 90) * 1000));
		phase = dds_st.cached_phase[DDS_CHAN_TX1_I_F2] / 1000;
		console_print("dds_tx1_tone2_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone1_scale(double* param, char param_no)	// dds_tx1_tone1_scale?
{
	double scale = dds_st.cached_scale[DDS_CHAN_TX1_I_F1];

	console_print("dds_tx1_tone1_scale=%f\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone1_scale(double* param, char param_no)	// dds_tx1_tone1_scale=
{
	double scale = param[0];

	if(param_no >= 1)
	{
		dds_set_scale(DDS_CHAN_TX1_I_F1, scale);
		dds_set_scale(DDS_CHAN_TX1_Q_F1, scale);
		scale = dds_st.cached_scale[DDS_CHAN_TX1_I_F1];
		console_print("dds_tx1_tone1_scale=%f\n", scale);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX1 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void get_dds_tx1_tone2_scale(double* param, char param_no)	// dds_tx1_tone2_scale?
{
	double scale = dds_st.cached_scale[DDS_CHAN_TX1_I_F2];

	console_print("dds_tx1_tone2_scale=%f\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX1 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx1_tone2_scale(double* param, char param_no)	// dds_tx1_tone2_scale=
{
	double scale = param[0];

	if(param_no >= 1)
	{
		dds_set_scale(DDS_CHAN_TX1_I_F2, scale);
		dds_set_scale(DDS_CHAN_TX1_Q_F2, scale);
		scale = dds_st.cached_scale[DDS_CHAN_TX1_I_F2];
		console_print("dds_tx1_tone2_scale=%f\n", scale);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone1_freq(double* param, char param_no)	// dds_tx2_tone1_freq?
{
	uint32_t freq = dds_st.cached_freq[DDS_CHAN_TX2_I_F1];

	console_print("dds_tx2_tone1_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 1 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone1_freq(double* param, char param_no)	// dds_tx2_tone1_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_frequency(DDS_CHAN_TX2_I_F1, freq);
		dds_set_frequency(DDS_CHAN_TX2_Q_F1, freq);
		console_print("dds_tx2_tone1_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone2_freq(double* param, char param_no)	// dds_tx2_tone2_freq?
{
	uint32_t freq = dds_st.cached_freq[DDS_CHAN_TX2_I_F2];

	console_print("dds_tx2_tone2_freq=%d\n", freq);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 2 frequency [Hz].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone2_freq(double* param, char param_no)	// dds_tx2_tone2_freq=
{
	uint32_t freq = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_frequency(DDS_CHAN_TX2_I_F2, freq);
		dds_set_frequency(DDS_CHAN_TX2_Q_F2, freq);
		console_print("dds_tx2_tone2_freq=%d\n", freq);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone1_phase(double* param, char param_no)	// dds_tx2_tone1_phase?
{
	uint32_t phase = dds_st.cached_phase[DDS_CHAN_TX2_I_F1];

	phase /= 1000;
	console_print("dds_tx2_tone1_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 1 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone1_phase(double* param, char param_no)	// dds_tx2_tone1_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_phase(DDS_CHAN_TX2_I_F1, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		dds_set_phase(DDS_CHAN_TX2_Q_F1, (uint32_t)((phase - 90) * 1000));
		phase = dds_st.cached_phase[DDS_CHAN_TX2_I_F1] / 1000;
		console_print("dds_tx2_tone1_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone2_phase(double* param, char param_no)	// dds_tx2_tone2_phase?
{
	uint32_t phase = dds_st.cached_phase[DDS_CHAN_TX2_I_F2];

	phase /= 1000;
	console_print("dds_tx2_f2_phase=%d\n", phase);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 2 phase [degrees].
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone2_phase(double* param, char param_no)	// dds_tx2_tone2_phase=
{
	int32_t phase = (uint32_t)param[0];

	if(param_no >= 1)
	{
		dds_set_phase(DDS_CHAN_TX2_I_F2, (uint32_t)(phase * 1000));
		if ((phase - 90) < 0)
			phase += 360;
		dds_set_phase(DDS_CHAN_TX2_Q_F2, (uint32_t)((phase - 90) * 1000));
		phase = dds_st.cached_phase[DDS_CHAN_TX2_I_F2] / 1000;
		console_print("dds_tx2_tone2_phase=%d\n", phase);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void get_dds_tx2_tone1_scale(double* param, char param_no)	// dds_tx2_tone1_scale?
{
	double scale = dds_st.cached_scale[DDS_CHAN_TX2_I_F1];

	console_print("dds_tx2_tone1_scale=%f\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 1 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone1_scale(double* param, char param_no)	// dds_tx2_tone1_scale=
{
	double scale = param[0];

	if(param_no >= 1)
	{
		dds_set_scale(DDS_CHAN_TX2_I_F1, scale);
		dds_set_scale(DDS_CHAN_TX2_Q_F1, scale);
		scale = dds_st.cached_scale[DDS_CHAN_TX2_I_F1];
		console_print("dds_tx2_tone1_scale=%f\n", scale);
	}
	else
		show_invalid_param_message(1);
}

/**************************************************************************//***
 * @brief Gets current DDS TX2 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void dds_tx2_tone2_scale(double* param, char param_no)	// dds_tx2_tone2_scale?
{
	double scale = dds_st.cached_scale[DDS_CHAN_TX2_I_F2];

	console_print("dds_tx2_tone2_scale=%f\n", scale);
}

/**************************************************************************//***
 * @brief Sets the DDS TX2 Tone 2 scale.
 *
 * @return None.
*******************************************************************************/
void set_dds_tx2_tone2_scale(double* param, char param_no)	// dds_tx2_tone2_scale=
{
	double scale = param[0];

	if(param_no >= 1)
	{
		dds_set_scale(DDS_CHAN_TX2_I_F2, scale);
		dds_set_scale(DDS_CHAN_TX2_Q_F2, scale);
		scale = dds_st.cached_scale[DDS_CHAN_TX2_I_F2];
		console_print("dds_tx2_tone2_scale=%f\n", scale);
	}
	else
		show_invalid_param_message(1);
}
#endif
