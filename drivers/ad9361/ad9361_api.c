/***************************************************************************//**
 *   @file   ad9361_api.c
 *   @brief  Implementation of AD9361 API Driver.
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
#include <malloc.h>
#include <spi.h>
#include <ad9361/ad9361.h>
#include <ad9361/ad9361_api.h>
#include <ad9361/platform.h>
#include <ad9361/util.h>

/******************************************************************************/
/************************ Constants Definitions *******************************/
/******************************************************************************/
const AD9361_InitParam default_init_param = {
	/* Reference Clock */
	38400000UL,	//reference_clk_rate
	/* Base Configuration */
	1,		//two_rx_two_tx_mode_enable *** adi,2rx-2tx-mode-enable
	1,		//frequency_division_duplex_mode_enable *** adi,frequency-division-duplex-mode-enable
	0,		//tdd_use_dual_synth_mode_enable *** adi,tdd-use-dual-synth-mode-enable
	0,		//tdd_skip_vco_cal_enable *** adi,tdd-skip-vco-cal-enable
	0,		//tx_fastlock_delay_ns *** adi,tx-fastlock-delay-ns
	0,		//rx_fastlock_delay_ns *** adi,rx-fastlock-delay-ns
	0,		//rx_fastlock_pincontrol_enable *** adi,rx-fastlock-pincontrol-enable
	0,		//tx_fastlock_pincontrol_enable *** adi,tx-fastlock-pincontrol-enable
	0,		//external_rx_lo_enable *** adi,external-rx-lo-enable
	0,		//external_tx_lo_enable *** adi,external-tx-lo-enable
	5,		//dc_offset_tracking_update_event_mask *** adi,dc-offset-tracking-update-event-mask
	6,		//dc_offset_attenuation_high_range *** adi,dc-offset-tracking-update-event-mask
	5,		//dc_offset_attenuation_low_range *** adi,dc-offset-tracking-update-event-mask
	0x28,	//dc_offset_count_high_range *** adi,dc-offset-tracking-update-event-mask
	0x32,	//dc_offset_count_low_range *** adi,dc-offset-tracking-update-event-mask
	0,		//tdd_use_fdd_vco_tables_enable *** adi,tdd-use-fdd-vco-tables-enable
	0,		//split_gain_table_mode_enable *** adi,split-gain-table-mode-enable
	/* ENSM Control */
	0,		//ensm_enable_pin_pulse_mode_enable *** adi,ensm-enable-pin-pulse-mode-enable
	0,		//ensm_enable_txnrx_control_enable *** adi,ensm-enable-txnrx-control-enable
	/* LO Control */
	2400000000UL,	//rx_synthesizer_frequency_hz *** adi,rx-synthesizer-frequency-hz
	2400000000UL,	//tx_synthesizer_frequency_hz *** adi,tx-synthesizer-frequency-hz
	/* Rate & BW Control */
//	{983040000, 245760000, 122880000, 61440000, 30720000, 30720000},//uint32_t	rx_path_clock_frequencies[6] *** adi,rx-path-clock-frequencies
//	{983040000, 122880000, 122880000, 61440000, 30720000, 30720000},//uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	{1228800000, 307200000, 153600000, 76800000, 38400000, 38400000},
	{1228800000, 307200000, 153600000, 76800000, 38400000, 38400000},
	18000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
	18000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
	/* RF Port Control */
	0,		//rx_rf_port_input_select *** adi,rx-rf-port-input-select
	0,		//tx_rf_port_input_select *** adi,tx-rf-port-input-select
	/* TX Attenuation Control */
	10000,	//tx_attenuation_mdB *** adi,tx-attenuation-mdB
	0,		//update_tx_gain_in_alert_enable *** adi,update-tx-gain-in-alert-enable
	/* Reference Clock Control */
	0,		//xo_disable_use_ext_refclk_enable *** adi,xo-disable-use-ext-refclk-enable
	{8, 5920},	//dcxo_coarse_and_fine_tune[2] *** adi,dcxo-coarse-and-fine-tune
	0,		//clk_output_mode_select *** adi,clk-output-mode-select
	/* Gain Control */
	2,		//gc_rx1_mode *** adi,gc-rx1-mode
	2,		//gc_rx2_mode *** adi,gc-rx2-mode
	58,		//gc_adc_large_overload_thresh *** adi,gc-adc-large-overload-thresh
	4,		//gc_adc_ovr_sample_size *** adi,gc-adc-ovr-sample-size
	47,		//gc_adc_small_overload_thresh *** adi,gc-adc-small-overload-thresh
	8192,	//gc_dec_pow_measurement_duration *** adi,gc-dec-pow-measurement-duration
	0,		//gc_dig_gain_enable *** adi,gc-dig-gain-enable
	800,	//gc_lmt_overload_high_thresh *** adi,gc-lmt-overload-high-thresh
	704,	//gc_lmt_overload_low_thresh *** adi,gc-lmt-overload-low-thresh
	24,		//gc_low_power_thresh *** adi,gc-low-power-thresh
	15,		//gc_max_dig_gain *** adi,gc-max-dig-gain
	/* Gain MGC Control */
	2,		//mgc_dec_gain_step *** adi,mgc-dec-gain-step
	2,		//mgc_inc_gain_step *** adi,mgc-inc-gain-step
	0,		//mgc_rx1_ctrl_inp_enable *** adi,mgc-rx1-ctrl-inp-enable
	0,		//mgc_rx2_ctrl_inp_enable *** adi,mgc-rx2-ctrl-inp-enable
	0,		//mgc_split_table_ctrl_inp_gain_mode *** adi,mgc-split-table-ctrl-inp-gain-mode
	/* Gain AGC Control */
	10,		//agc_adc_large_overload_exceed_counter *** adi,agc-adc-large-overload-exceed-counter
	2,		//agc_adc_large_overload_inc_steps *** adi,agc-adc-large-overload-inc-steps
	0,		//agc_adc_lmt_small_overload_prevent_gain_inc_enable *** adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable
	10,		//agc_adc_small_overload_exceed_counter *** adi,agc-adc-small-overload-exceed-counter
	4,		//agc_dig_gain_step_size *** adi,agc-dig-gain-step-size
	3,		//agc_dig_saturation_exceed_counter *** adi,agc-dig-saturation-exceed-counter
	1000,	// agc_gain_update_interval_us *** adi,agc-gain-update-interval-us
	0,		//agc_immed_gain_change_if_large_adc_overload_enable *** adi,agc-immed-gain-change-if-large-adc-overload-enable
	0,		//agc_immed_gain_change_if_large_lmt_overload_enable *** adi,agc-immed-gain-change-if-large-lmt-overload-enable
	10,		//agc_inner_thresh_high *** adi,agc-inner-thresh-high
	1,		//agc_inner_thresh_high_dec_steps *** adi,agc-inner-thresh-high-dec-steps
	12,		//agc_inner_thresh_low *** adi,agc-inner-thresh-low
	1,		//agc_inner_thresh_low_inc_steps *** adi,agc-inner-thresh-low-inc-steps
	10,		//agc_lmt_overload_large_exceed_counter *** adi,agc-lmt-overload-large-exceed-counter
	2,		//agc_lmt_overload_large_inc_steps *** adi,agc-lmt-overload-large-inc-steps
	10,		//agc_lmt_overload_small_exceed_counter *** adi,agc-lmt-overload-small-exceed-counter
	5,		//agc_outer_thresh_high *** adi,agc-outer-thresh-high
	2,		//agc_outer_thresh_high_dec_steps *** adi,agc-outer-thresh-high-dec-steps
	18,		//agc_outer_thresh_low *** adi,agc-outer-thresh-low
	2,		//agc_outer_thresh_low_inc_steps *** adi,agc-outer-thresh-low-inc-steps
	1,		//agc_attack_delay_extra_margin_us; *** adi,agc-attack-delay-extra-margin-us
	0,		//agc_sync_for_gain_counter_enable *** adi,agc-sync-for-gain-counter-enable
	/* Fast AGC */
	64,		//fagc_dec_pow_measuremnt_duration ***  adi,fagc-dec-pow-measurement-duration
	260,	//fagc_state_wait_time_ns ***  adi,fagc-state-wait-time-ns
		/* Fast AGC - Low Power */
	0,		//fagc_allow_agc_gain_increase ***  adi,fagc-allow-agc-gain-increase-enable
	5,		//fagc_lp_thresh_increment_time ***  adi,fagc-lp-thresh-increment-time
	1,		//fagc_lp_thresh_increment_steps ***  adi,fagc-lp-thresh-increment-steps
		/* Fast AGC - Lock Level */
	10,		//fagc_lock_level ***  adi,fagc-lock-level */
	1,		//fagc_lock_level_lmt_gain_increase_en ***  adi,fagc-lock-level-lmt-gain-increase-enable
	5,		//fagc_lock_level_gain_increase_upper_limit ***  adi,fagc-lock-level-gain-increase-upper-limit
		/* Fast AGC - Peak Detectors and Final Settling */
	1,		//fagc_lpf_final_settling_steps ***  adi,fagc-lpf-final-settling-steps
	1,		//fagc_lmt_final_settling_steps ***  adi,fagc-lmt-final-settling-steps
	3,		//fagc_final_overrange_count ***  adi,fagc-final-overrange-count
		/* Fast AGC - Final Power Test */
	0,		//fagc_gain_increase_after_gain_lock_en ***  adi,fagc-gain-increase-after-gain-lock-enable
		/* Fast AGC - Unlocking the Gain */
		/* 0 = MAX Gain, 1 = Optimized Gain, 2 = Set Gain */
	0,		//fagc_gain_index_type_after_exit_rx_mode ***  adi,fagc-gain-index-type-after-exit-rx-mode
	1,		//fagc_use_last_lock_level_for_set_gain_en ***  adi,fagc-use-last-lock-level-for-set-gain-enable
	1,		//fagc_rst_gla_stronger_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-stronger-sig-thresh-exceeded-enable
	5,		//fagc_optimized_gain_offset ***  adi,fagc-optimized-gain-offset
	10,		//fagc_rst_gla_stronger_sig_thresh_above_ll ***  adi,fagc-rst-gla-stronger-sig-thresh-above-ll
	1,		//fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-exceeded-enable
	1,		//fagc_rst_gla_engergy_lost_goto_optim_gain_en ***  adi,fagc-rst-gla-engergy-lost-goto-optim-gain-enable
	10,		//fagc_rst_gla_engergy_lost_sig_thresh_below_ll ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-below-ll
	8,		//fagc_energy_lost_stronger_sig_gain_lock_exit_cnt ***  adi,fagc-energy-lost-stronger-sig-gain-lock-exit-cnt
	1,		//fagc_rst_gla_large_adc_overload_en ***  adi,fagc-rst-gla-large-adc-overload-enable
	1,		//fagc_rst_gla_large_lmt_overload_en ***  adi,fagc-rst-gla-large-lmt-overload-enable
	0,		//fagc_rst_gla_en_agc_pulled_high_en ***  adi,fagc-rst-gla-en-agc-pulled-high-enable
	0,		//fagc_rst_gla_if_en_agc_pulled_high_mode ***  adi,fagc-rst-gla-if-en-agc-pulled-high-mode
	64,		//fagc_power_measurement_duration_in_state5 ***  adi,fagc-power-measurement-duration-in-state5
	/* RSSI Control */
	1,		//rssi_delay *** adi,rssi-delay
	1000,	//rssi_duration *** adi,rssi-duration
	3,		//rssi_restart_mode *** adi,rssi-restart-mode
	0,		//rssi_unit_is_rx_samples_enable *** adi,rssi-unit-is-rx-samples-enable
	1,		//rssi_wait *** adi,rssi-wait
	/* Aux ADC Control */
	256,	//aux_adc_decimation *** adi,aux-adc-decimation
	40000000UL,	//aux_adc_rate *** adi,aux-adc-rate
	/* AuxDAC Control */
	1,		//aux_dac_manual_mode_enable ***  adi,aux-dac-manual-mode-enable
	0,		//aux_dac1_default_value_mV ***  adi,aux-dac1-default-value-mV
	0,		//aux_dac1_active_in_rx_enable ***  adi,aux-dac1-active-in-rx-enable
	0,		//aux_dac1_active_in_tx_enable ***  adi,aux-dac1-active-in-tx-enable
	0,		//aux_dac1_active_in_alert_enable ***  adi,aux-dac1-active-in-alert-enable
	0,		//aux_dac1_rx_delay_us ***  adi,aux-dac1-rx-delay-us
	0,		//aux_dac1_tx_delay_us ***  adi,aux-dac1-tx-delay-us
	0,		//aux_dac2_default_value_mV ***  adi,aux-dac2-default-value-mV
	0,		//aux_dac2_active_in_rx_enable ***  adi,aux-dac2-active-in-rx-enable
	0,		//aux_dac2_active_in_tx_enable ***  adi,aux-dac2-active-in-tx-enable
	0,		//aux_dac2_active_in_alert_enable ***  adi,aux-dac2-active-in-alert-enable
	0,		//aux_dac2_rx_delay_us ***  adi,aux-dac2-rx-delay-us
	0,		//aux_dac2_tx_delay_us ***  adi,aux-dac2-tx-delay-us
	/* Temperature Sensor Control */
	256,	//temp_sense_decimation *** adi,temp-sense-decimation
	1000,	//temp_sense_measurement_interval_ms *** adi,temp-sense-measurement-interval-ms
	0xCE,	//temp_sense_offset_signed *** adi,temp-sense-offset-signed
	1,		//temp_sense_periodic_measurement_enable *** adi,temp-sense-periodic-measurement-enable
	/* Control Out Setup */
	0xFF,	//ctrl_outs_enable_mask *** adi,ctrl-outs-enable-mask
	0,		//ctrl_outs_index *** adi,ctrl-outs-index
	/* External LNA Control */
	0,		//elna_settling_delay_ns *** adi,elna-settling-delay-ns
	0,		//elna_gain_mdB *** adi,elna-gain-mdB
	0,		//elna_bypass_loss_mdB *** adi,elna-bypass-loss-mdB
	0,		//elna_rx1_gpo0_control_enable *** adi,elna-rx1-gpo0-control-enable
	0,		//elna_rx2_gpo1_control_enable *** adi,elna-rx2-gpo1-control-enable
	/* Digital Interface Control */
	1,		//pp_tx_swap_enable *** adi,pp-tx-swap-enable
	1,		//pp_rx_swap_enable *** adi,pp-rx-swap-enable
	0,		//tx_channel_swap_enable *** adi,tx-channel-swap-enable
	0,		//rx_channel_swap_enable *** adi,rx-channel-swap-enable
	1,		//rx_frame_pulse_mode_enable *** adi,rx-frame-pulse-mode-enable
	1,		//two_t_two_r_timing_enable *** adi,2t2r-timing-enable
	0,		//invert_data_bus_enable *** adi,invert-data-bus-enable
	0,		//invert_data_clk_enable *** adi,invert-data-clk-enable
	0,		//fdd_alt_word_order_enable *** adi,fdd-alt-word-order-enable
	0,		//invert_rx_frame_enable *** adi,invert-rx-frame-enable
	0,		//fdd_rx_rate_2tx_enable *** adi,fdd-rx-rate-2tx-enable
	0,		//swap_ports_enable *** adi,swap-ports-enable
	0,		//single_data_rate_enable *** adi,single-data-rate-enable
	1,		//lvds_mode_enable *** adi,lvds-mode-enable
	0,		//half_duplex_mode_enable *** adi,half-duplex-mode-enable
	0,		//single_port_mode_enable *** adi,single-port-mode-enable
	0,		//full_port_enable *** adi,full-port-enable
	0,		//full_duplex_swap_bits_enable *** adi,full-duplex-swap-bits-enable
	0,		//delay_rx_data *** adi,delay-rx-data
	0,		//rx_data_clock_delay *** adi,rx-data-clock-delay
	4,		//rx_data_delay *** adi,rx-data-delay
	7,		//tx_fb_clock_delay *** adi,tx-fb-clock-delay
	0,		//tx_data_delay *** adi,tx-data-delay
	150,	//lvds_bias_mV *** adi,lvds-bias-mV
	1,		//lvds_rx_onchip_termination_enable *** adi,lvds-rx-onchip-termination-enable
	0,		//rx1rx2_phase_inversion_en *** adi,rx1-rx2-phase-inversion-enable
	/* Tx Monitor Control */
	37000,	//low_high_gain_threshold_mdB *** adi,txmon-low-high-thresh
	0,		//low_gain_dB *** adi,txmon-low-gain
	24,		//high_gain_dB *** adi,txmon-high-gain
	0,		//tx_mon_track_en *** adi,txmon-dc-tracking-enable
	0,		//one_shot_mode_en *** adi,txmon-one-shot-mode-enable
	511,	//tx_mon_delay *** adi,txmon-delay
	8192,	//tx_mon_duration *** adi,txmon-duration
	2,		//tx1_mon_front_end_gain *** adi,txmon-1-front-end-gain
	2,		//tx2_mon_front_end_gain *** adi,txmon-2-front-end-gain
	48,		//tx1_mon_lo_cm *** adi,txmon-1-lo-cm
	48,		//tx2_mon_lo_cm *** adi,txmon-2-lo-cm
	/* GPIO definitions */
	-1,		//gpio_resetb;	/* reset-gpios */
	/* MCS Sync */
	-1,		//gpio_sync;		/* sync-gpios */
	-1,		//gpio_cal_sw1;	/* cal-sw1-gpios */
	-1		//gpio_cal_sw2;	/* cal-sw2-gpios */
};

const AD9361_RXFIRConfig rx_fir_config = {
	3, // rx;
	0, // rx_gain;
	1, // rx_dec;
	{0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 -4, -6, -37, 35, 186, 86, -284, 315,
	 107, 219, -4, 271, 558, -307, -1182, -356,
	 658, 157, 207, 1648, 790, -2525, -2553, 748,
	 865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
	 14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
	 748, -2553, -2525, 790, 1648, 207, 157, 658,
	 -356, -1182, -307, 558, 271, -4, 219, 107,
	 -315, -284, 86, 186, 35, -37, -6, -4,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0} // rx_coef[128];
};

const AD9361_TXFIRConfig tx_fir_config = {
	3, // tx;
	-6, // tx_gain;
	1, // tx_inc;
	{0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 -4, -6, -37, 35, 186, 86, -284, 315,
	 107, 219, -4, 271, 558, -307, -1182, -356,
	 658, 157, 207, 1648, 790, -2525, -2553, 748,
	 865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
	 14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
	 748, -2553, -2525, 790, 1648, 207, 157, 658,
	 -356, -1182, -307, 558, 271, -4, 219, 107,
	 -315, -284, 86, 186, 35, -37, -6, -4,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0,
	 0, 0, 0, 0, 0, 0, 0, 0} // tx_coef[128];
};

static struct axiadc_chip_info axiadc_chip_info_tbl[] =
{
	{
		"AD9361",
		4,
		61440000UL,
	},
	{
		"AD9364",
		2,
		122880000UL,
	},
};

/**
 * Initialize the AD9361 part.
 * @param init_param The structure that contains the AD9361 initial parameters.
 * @return A structure that contains the AD9361 current state in case of
 *         success, negative error code otherwise.
 */
struct ad9361_rf_phy *ad9361_init (AD9361_InitParam *init_param, uint32_t bus)
{
	struct ad9361_rf_phy *phy;
	int32_t ret = 0;
	int32_t rev = 0;
	int32_t i   = 0;

	phy = (struct ad9361_rf_phy *)zmalloc(sizeof(*phy));
	if (!phy) {
		return (struct ad9361_rf_phy *)ERR_PTR(-ENOMEM);
	}

	phy->spi = (struct spi_device *)zmalloc(sizeof(*phy->spi));
	if (!phy->spi) {
		return (struct ad9361_rf_phy *)ERR_PTR(-ENOMEM);
	}

	phy->clk_refin = (struct clk *)zmalloc(sizeof(*phy->clk_refin));
	if (!phy->clk_refin) {
		return (struct ad9361_rf_phy *)ERR_PTR(-ENOMEM);
	}

	phy->pdata = (struct ad9361_phy_platform_data *)zmalloc(sizeof(*phy->pdata));
	if (!phy->pdata) {
		return (struct ad9361_rf_phy *)ERR_PTR(-ENOMEM);
	}

	phy->adc_conv = (struct axiadc_converter *)zmalloc(sizeof(*phy->adc_conv));
	if (!phy->adc_conv) {
		return (struct ad9361_rf_phy *)ERR_PTR(-ENOMEM);
	}

	phy->adc_state = (struct axiadc_state *)zmalloc(sizeof(*phy->adc_state));
	if (!phy->adc_state) {
		return (struct ad9361_rf_phy *)ERR_PTR(-ENOMEM);
	}

	/* Reference Clock */
	phy->clk_refin->rate = init_param->reference_clk_rate;

	/* SPI interface */
	phy->spi->dev.bus = bus;

	if(NULL == init_param)
		init_param = &default_init_param;

	/* Base Configuration */
	phy->pdata->fdd = init_param->frequency_division_duplex_mode_enable;
	phy->pdata->rx2tx2 = init_param->two_rx_two_tx_mode_enable;
	phy->pdata->tdd_use_dual_synth = init_param->tdd_use_dual_synth_mode_enable;
	phy->pdata->tdd_skip_vco_cal = init_param->tdd_skip_vco_cal_enable;
	phy->pdata->rx_fastlock_delay_ns = init_param->rx_fastlock_delay_ns;
	phy->pdata->tx_fastlock_delay_ns = init_param->tx_fastlock_delay_ns;
	phy->pdata->trx_fastlock_pinctrl_en[0] = init_param->rx_fastlock_pincontrol_enable;
	phy->pdata->trx_fastlock_pinctrl_en[1] = init_param->tx_fastlock_pincontrol_enable;
	phy->pdata->use_ext_rx_lo = init_param->external_rx_lo_enable;
	phy->pdata->use_ext_tx_lo = init_param->external_tx_lo_enable;
	phy->pdata->dc_offset_update_events = init_param->dc_offset_tracking_update_event_mask;
	phy->pdata->dc_offset_attenuation_high = init_param->dc_offset_attenuation_high_range;
	phy->pdata->dc_offset_attenuation_low = init_param->dc_offset_attenuation_low_range;
	phy->pdata->rf_dc_offset_count_high = init_param->dc_offset_count_high_range;
	phy->pdata->rf_dc_offset_count_low = init_param->dc_offset_count_low_range;
	phy->pdata->tdd_use_fdd_tables = init_param->tdd_use_fdd_vco_tables_enable;
	phy->pdata->split_gt = init_param->split_gain_table_mode_enable;

	/* ENSM Control */
	phy->pdata->ensm_pin_pulse_mode = init_param->ensm_enable_pin_pulse_mode_enable;
	phy->pdata->ensm_pin_ctrl = init_param->ensm_enable_txnrx_control_enable;

	/* LO Control */
	phy->pdata->rx_synth_freq = init_param->rx_synthesizer_frequency_hz;
	phy->pdata->tx_synth_freq = init_param->tx_synthesizer_frequency_hz;

	/* Rate & BW Control */
	for(i = 0; i < 6; i++) {
		phy->pdata->rx_path_clks[i] = init_param->rx_path_clock_frequencies[i];
	}
	for(i = 0; i < 6; i++) {
		phy->pdata->tx_path_clks[i] = init_param->tx_path_clock_frequencies[i];
	}
	phy->pdata->rf_rx_bandwidth_Hz = init_param->rf_rx_bandwidth_hz;
	phy->pdata->rf_tx_bandwidth_Hz = init_param->rf_tx_bandwidth_hz;

	/* RF Port Control */
	phy->pdata->rf_rx_input_sel = init_param->rx_rf_port_input_select;
	phy->pdata->rf_tx_output_sel = init_param->tx_rf_port_input_select;

	/* TX Attenuation Control */
	phy->pdata->tx_atten = init_param->tx_attenuation_mdB;
	phy->pdata->update_tx_gain_via_alert = init_param->update_tx_gain_in_alert_enable;

	/* Reference Clock Control */
	phy->pdata->use_extclk = init_param->xo_disable_use_ext_refclk_enable;
	phy->pdata->dcxo_coarse = init_param->dcxo_coarse_and_fine_tune[0];
	phy->pdata->dcxo_fine = init_param->dcxo_coarse_and_fine_tune[1];
	phy->pdata->ad9361_clkout_mode = (enum ad9361_clkout)init_param->clk_output_mode_select;

	/* Gain Control */
	phy->pdata->gain_ctrl.rx1_mode = (enum rf_gain_ctrl_mode)init_param->gc_rx1_mode;
	phy->pdata->gain_ctrl.rx2_mode = (enum rf_gain_ctrl_mode)init_param->gc_rx2_mode;
	phy->pdata->gain_ctrl.adc_large_overload_thresh = init_param->gc_adc_large_overload_thresh;
	phy->pdata->gain_ctrl.adc_ovr_sample_size = init_param->gc_adc_ovr_sample_size;
	phy->pdata->gain_ctrl.adc_small_overload_thresh = init_param->gc_adc_small_overload_thresh;
	phy->pdata->gain_ctrl.dec_pow_measuremnt_duration = init_param->gc_dec_pow_measurement_duration;
	phy->pdata->gain_ctrl.dig_gain_en = init_param->gc_dig_gain_enable;
	phy->pdata->gain_ctrl.lmt_overload_high_thresh = init_param->gc_lmt_overload_high_thresh;
	phy->pdata->gain_ctrl.lmt_overload_low_thresh = init_param->gc_lmt_overload_low_thresh;
	phy->pdata->gain_ctrl.low_power_thresh = init_param->gc_low_power_thresh;
	phy->pdata->gain_ctrl.max_dig_gain = init_param->gc_max_dig_gain;

	/* Gain MGC Control */
	phy->pdata->gain_ctrl.mgc_dec_gain_step = init_param->mgc_dec_gain_step;
	phy->pdata->gain_ctrl.mgc_inc_gain_step = init_param->mgc_inc_gain_step;
	phy->pdata->gain_ctrl.mgc_rx1_ctrl_inp_en = init_param->mgc_rx1_ctrl_inp_enable;
	phy->pdata->gain_ctrl.mgc_rx2_ctrl_inp_en = init_param->mgc_rx2_ctrl_inp_enable;
	phy->pdata->gain_ctrl.mgc_split_table_ctrl_inp_gain_mode = init_param->mgc_split_table_ctrl_inp_gain_mode;

	/* Gain AGC Control */
	phy->pdata->gain_ctrl.adc_large_overload_exceed_counter = init_param->agc_adc_large_overload_exceed_counter;
	phy->pdata->gain_ctrl.adc_large_overload_inc_steps = init_param->agc_adc_large_overload_inc_steps;
	phy->pdata->gain_ctrl.adc_lmt_small_overload_prevent_gain_inc = init_param->agc_adc_lmt_small_overload_prevent_gain_inc_enable;
	phy->pdata->gain_ctrl.adc_small_overload_exceed_counter = init_param->agc_adc_small_overload_exceed_counter;
	phy->pdata->gain_ctrl.dig_gain_step_size = init_param->agc_dig_gain_step_size;
	phy->pdata->gain_ctrl.dig_saturation_exceed_counter = init_param->agc_dig_saturation_exceed_counter;
	phy->pdata->gain_ctrl.gain_update_interval_us = init_param->agc_gain_update_interval_us;
	phy->pdata->gain_ctrl.immed_gain_change_if_large_adc_overload = init_param->agc_immed_gain_change_if_large_adc_overload_enable;
	phy->pdata->gain_ctrl.immed_gain_change_if_large_lmt_overload = init_param->agc_immed_gain_change_if_large_lmt_overload_enable;
	phy->pdata->gain_ctrl.agc_inner_thresh_high = init_param->agc_inner_thresh_high;
	phy->pdata->gain_ctrl.agc_inner_thresh_high_dec_steps = init_param->agc_inner_thresh_high_dec_steps;
	phy->pdata->gain_ctrl.agc_inner_thresh_low = init_param->agc_inner_thresh_low;
	phy->pdata->gain_ctrl.agc_inner_thresh_low_inc_steps = init_param->agc_inner_thresh_low_inc_steps;
	phy->pdata->gain_ctrl.lmt_overload_large_exceed_counter = init_param->agc_lmt_overload_large_exceed_counter;
	phy->pdata->gain_ctrl.lmt_overload_large_inc_steps = init_param->agc_lmt_overload_large_inc_steps;
	phy->pdata->gain_ctrl.lmt_overload_small_exceed_counter = init_param->agc_lmt_overload_small_exceed_counter;
	phy->pdata->gain_ctrl.agc_outer_thresh_high = init_param->agc_outer_thresh_high;
	phy->pdata->gain_ctrl.agc_outer_thresh_high_dec_steps = init_param->agc_outer_thresh_high_dec_steps;
	phy->pdata->gain_ctrl.agc_outer_thresh_low = init_param->agc_outer_thresh_low;
	phy->pdata->gain_ctrl.agc_outer_thresh_low_inc_steps = init_param->agc_outer_thresh_low_inc_steps;
	phy->pdata->gain_ctrl.agc_attack_delay_extra_margin_us = init_param->agc_attack_delay_extra_margin_us;
	phy->pdata->gain_ctrl.sync_for_gain_counter_en = init_param->agc_sync_for_gain_counter_enable;

	/* Fast AGC */

	phy->pdata->gain_ctrl.f_agc_dec_pow_measuremnt_duration = init_param->fagc_dec_pow_measuremnt_duration;
	phy->pdata->gain_ctrl.f_agc_dec_pow_measuremnt_duration = init_param->fagc_state_wait_time_ns;
		/* Fast AGC - Low Power */
	phy->pdata->gain_ctrl.f_agc_allow_agc_gain_increase = init_param->fagc_allow_agc_gain_increase;
	phy->pdata->gain_ctrl.f_agc_lp_thresh_increment_time = init_param->fagc_lp_thresh_increment_time;
	phy->pdata->gain_ctrl.f_agc_lp_thresh_increment_steps = init_param->fagc_lp_thresh_increment_steps;
		/* Fast AGC - Lock Level */
	phy->pdata->gain_ctrl.f_agc_lock_level = init_param->fagc_lock_level;
	phy->pdata->gain_ctrl.f_agc_lock_level_lmt_gain_increase_en = init_param->fagc_lock_level_lmt_gain_increase_en;
	phy->pdata->gain_ctrl.f_agc_lock_level_gain_increase_upper_limit = init_param->fagc_lock_level_gain_increase_upper_limit;
		/* Fast AGC - Peak Detectors and Final Settling */
	phy->pdata->gain_ctrl.f_agc_lpf_final_settling_steps = init_param->fagc_lpf_final_settling_steps;
	phy->pdata->gain_ctrl.f_agc_lmt_final_settling_steps = init_param->fagc_lmt_final_settling_steps;
	phy->pdata->gain_ctrl.f_agc_final_overrange_count = init_param->fagc_final_overrange_count;
		/* Fast AGC - Final Power Test */
	phy->pdata->gain_ctrl.f_agc_gain_increase_after_gain_lock_en = init_param->fagc_gain_increase_after_gain_lock_en;
		/* Fast AGC - Unlocking the Gain */
	phy->pdata->gain_ctrl.f_agc_gain_index_type_after_exit_rx_mode = (enum f_agc_target_gain_index_type)init_param->fagc_gain_index_type_after_exit_rx_mode;
	phy->pdata->gain_ctrl.f_agc_use_last_lock_level_for_set_gain_en = init_param->fagc_use_last_lock_level_for_set_gain_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_stronger_sig_thresh_exceeded_en = init_param->fagc_rst_gla_stronger_sig_thresh_exceeded_en;
	phy->pdata->gain_ctrl.f_agc_optimized_gain_offset = init_param->fagc_optimized_gain_offset;
	phy->pdata->gain_ctrl.f_agc_rst_gla_stronger_sig_thresh_above_ll = init_param->fagc_rst_gla_stronger_sig_thresh_above_ll;
	phy->pdata->gain_ctrl.f_agc_rst_gla_engergy_lost_sig_thresh_exceeded_en = init_param->fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_engergy_lost_goto_optim_gain_en = init_param->fagc_rst_gla_engergy_lost_goto_optim_gain_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_engergy_lost_sig_thresh_below_ll = init_param->fagc_rst_gla_engergy_lost_sig_thresh_below_ll;
	phy->pdata->gain_ctrl.f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt = init_param->fagc_energy_lost_stronger_sig_gain_lock_exit_cnt;
	phy->pdata->gain_ctrl.f_agc_rst_gla_large_adc_overload_en = init_param->fagc_rst_gla_large_adc_overload_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_large_lmt_overload_en = init_param->fagc_rst_gla_large_lmt_overload_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_en_agc_pulled_high_en = init_param->fagc_rst_gla_en_agc_pulled_high_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_if_en_agc_pulled_high_mode = (enum f_agc_target_gain_index_type)init_param->fagc_rst_gla_if_en_agc_pulled_high_mode;
	phy->pdata->gain_ctrl.f_agc_power_measurement_duration_in_state5 = init_param->fagc_power_measurement_duration_in_state5;

	/* RSSI Control */
	phy->pdata->rssi_ctrl.rssi_delay = init_param->rssi_delay;
	phy->pdata->rssi_ctrl.rssi_duration = init_param->rssi_duration;
	phy->pdata->rssi_ctrl.restart_mode = (enum rssi_restart_mode)init_param->rssi_restart_mode;
	phy->pdata->rssi_ctrl.rssi_unit_is_rx_samples = init_param->rssi_unit_is_rx_samples_enable;
	phy->pdata->rssi_ctrl.rssi_wait = init_param->rssi_wait;

	/* Aux ADC Control */
	phy->pdata->auxadc_ctrl.auxadc_decimation = init_param->aux_adc_decimation;
	phy->pdata->auxadc_ctrl.auxadc_clock_rate = init_param->aux_adc_rate;

	/* AuxDAC Control */
	phy->pdata->auxdac_ctrl.auxdac_manual_mode_en = init_param->aux_dac_manual_mode_enable;
	phy->pdata->auxdac_ctrl.dac1_default_value = init_param->aux_dac1_default_value_mV;
	phy->pdata->auxdac_ctrl.dac1_in_rx_en = init_param->aux_dac1_active_in_rx_enable;
	phy->pdata->auxdac_ctrl.dac1_in_tx_en = init_param->aux_dac1_active_in_tx_enable;
	phy->pdata->auxdac_ctrl.dac1_in_alert_en = init_param->aux_dac1_active_in_alert_enable;
	phy->pdata->auxdac_ctrl.dac1_rx_delay_us = init_param->aux_dac1_rx_delay_us;
	phy->pdata->auxdac_ctrl.dac1_tx_delay_us = init_param->aux_dac1_tx_delay_us;
	phy->pdata->auxdac_ctrl.dac2_default_value = init_param->aux_dac2_default_value_mV;
	phy->pdata->auxdac_ctrl.dac2_in_rx_en = init_param->aux_dac2_active_in_rx_enable;
	phy->pdata->auxdac_ctrl.dac2_in_tx_en = init_param->aux_dac2_active_in_tx_enable;
	phy->pdata->auxdac_ctrl.dac2_in_alert_en = init_param->aux_dac2_active_in_alert_enable;
	phy->pdata->auxdac_ctrl.dac2_rx_delay_us = init_param->aux_dac2_rx_delay_us;
	phy->pdata->auxdac_ctrl.dac2_tx_delay_us = init_param->aux_dac2_tx_delay_us;

	/* Temperature Sensor Control */
	phy->pdata->auxadc_ctrl.temp_sensor_decimation = init_param->temp_sense_decimation;
	phy->pdata->auxadc_ctrl.temp_time_inteval_ms = init_param->temp_sense_measurement_interval_ms;
	phy->pdata->auxadc_ctrl.offset = init_param->temp_sense_offset_signed;
	phy->pdata->auxadc_ctrl.periodic_temp_measuremnt = init_param->temp_sense_periodic_measurement_enable;

	/* Control Out Setup */
	phy->pdata->ctrl_outs_ctrl.en_mask = init_param->ctrl_outs_enable_mask;
	phy->pdata->ctrl_outs_ctrl.index = init_param->ctrl_outs_index;

	/* External LNA Control */
	phy->pdata->elna_ctrl.settling_delay_ns = init_param->elna_settling_delay_ns;
	phy->pdata->elna_ctrl.gain_mdB = init_param->elna_gain_mdB;
	phy->pdata->elna_ctrl.bypass_loss_mdB = init_param->elna_bypass_loss_mdB;
	phy->pdata->elna_ctrl.elna_1_control_en = init_param->elna_rx1_gpo0_control_enable;
	phy->pdata->elna_ctrl.elna_2_control_en = init_param->elna_rx2_gpo1_control_enable;

	/* Digital Interface Control */
	phy->pdata->port_ctrl.pp_conf[0] = (init_param->pp_tx_swap_enable << 7);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->pp_rx_swap_enable << 6);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->tx_channel_swap_enable << 5);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->rx_channel_swap_enable << 4);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->rx_frame_pulse_mode_enable << 3);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->two_t_two_r_timing_enable << 2);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->invert_data_bus_enable << 1);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->invert_data_clk_enable << 0);
	phy->pdata->port_ctrl.pp_conf[1] = (init_param->fdd_alt_word_order_enable << 7);
	phy->pdata->port_ctrl.pp_conf[1] |= (init_param->invert_rx_frame_enable << 2);
	phy->pdata->port_ctrl.pp_conf[2] = (init_param->fdd_rx_rate_2tx_enable << 7);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->swap_ports_enable << 6);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->single_data_rate_enable << 5);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->lvds_mode_enable << 4);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->half_duplex_mode_enable << 3);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->single_port_mode_enable << 2);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->full_port_enable << 1);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->full_duplex_swap_bits_enable << 0);
	phy->pdata->port_ctrl.pp_conf[1] |= (init_param->delay_rx_data & 0x3);
	phy->pdata->port_ctrl.rx_clk_data_delay = DATA_CLK_DELAY(init_param->rx_data_clock_delay);
	phy->pdata->port_ctrl.rx_clk_data_delay |= RX_DATA_DELAY(init_param->rx_data_delay);
	phy->pdata->port_ctrl.tx_clk_data_delay = FB_CLK_DELAY(init_param->tx_fb_clock_delay);
	phy->pdata->port_ctrl.tx_clk_data_delay |= TX_DATA_DELAY(init_param->tx_data_delay);
	phy->pdata->port_ctrl.lvds_bias_ctrl = (init_param->lvds_bias_mV / 75) & 0x7;
	phy->pdata->port_ctrl.lvds_bias_ctrl |= (init_param->lvds_rx_onchip_termination_enable << 5);
	phy->pdata->rx1rx2_phase_inversion_en = init_param->rx1rx2_phase_inversion_en;

	/* Tx Monitor Control */
	phy->pdata->txmon_ctrl.low_high_gain_threshold_mdB = init_param->low_high_gain_threshold_mdB;
	phy->pdata->txmon_ctrl.low_gain_dB = init_param->low_gain_dB;
	phy->pdata->txmon_ctrl.high_gain_dB = init_param->high_gain_dB;
	phy->pdata->txmon_ctrl.tx_mon_track_en = init_param->tx_mon_track_en;
	phy->pdata->txmon_ctrl.one_shot_mode_en = init_param->one_shot_mode_en;
	phy->pdata->txmon_ctrl.tx_mon_delay = init_param->tx_mon_delay;
	phy->pdata->txmon_ctrl.tx_mon_duration = init_param->tx_mon_duration;
	phy->pdata->txmon_ctrl.tx1_mon_front_end_gain = init_param->tx1_mon_front_end_gain;
	phy->pdata->txmon_ctrl.tx2_mon_front_end_gain = init_param->tx2_mon_front_end_gain;
	phy->pdata->txmon_ctrl.tx1_mon_lo_cm = init_param->tx1_mon_lo_cm;
	phy->pdata->txmon_ctrl.tx2_mon_lo_cm = init_param->tx2_mon_lo_cm;

	phy->pdata->debug_mode = true;
	phy->pdata->gpio_resetb = init_param->gpio_resetb;
	/* Optional: next three GPIOs are used for MCS synchronization */
	phy->pdata->gpio_sync = init_param->gpio_sync;
	phy->pdata->gpio_cal_sw1 = init_param->gpio_cal_sw1;
	phy->pdata->gpio_cal_sw2 = init_param->gpio_cal_sw2;

	phy->pdata->port_ctrl.digital_io_ctrl = 0;
	phy->pdata->port_ctrl.lvds_invert[0] = 0xFF;
	phy->pdata->port_ctrl.lvds_invert[1] = 0x0F;

	phy->adc_conv->chip_info = &axiadc_chip_info_tbl[phy->pdata->rx2tx2 ? ID_AD9361 : ID_AD9364];

	phy->rx_eq_2tx = false;

	phy->current_table = RXGAIN_TBLS_END;
	phy->bypass_tx_fir = true;
	phy->bypass_rx_fir = true;
	phy->rate_governor = 1;
	phy->rfdc_track_en = true;
	phy->bbdc_track_en = true;
	phy->quad_track_en = true;

	ad9361_reset(phy);
	ad9361_spi_write(phy->spi, REG_SPI_CONF, SOFT_RESET | _SOFT_RESET);
	platform_mdelay(1000);
	ad9361_spi_write(phy->spi, REG_SPI_CONF, 0x0);

	ret = ad9361_spi_read(phy->spi, REG_PRODUCT_ID);
	if ((ret & PRODUCT_ID_MASK) != PRODUCT_ID_9361) {
		printf("%s : Unsupported PRODUCT_ID 0x%X\n", "ad9361_init", (unsigned int)ret);
		ret = -ENODEV;
		goto out;
	}
	rev = ret & REV_MASK;

	ret = register_clocks(phy);
	if (ret < 0)
		goto out;

	platform_axiadc_init(phy);

	ad9361_init_gain_tables(phy);

	ret = ad9361_setup(phy);
	if (ret < 0)
		goto out;

	ret = ad9361_post_setup(phy);
	if (ret < 0)
		goto out;

	printf("%s : AD9361 Rev %d successfully initialized\n", "ad9361_init", (int)rev);

	return phy;

out:
	free(phy->spi);
	free(phy->adc_conv);
	free(phy->adc_state);
	free(phy->clk_refin);
	free(phy->pdata);
	free(phy);
	printf("%s : AD9361 initialization error\n", "ad9361_init");

	return (struct ad9361_rf_phy *)NULL;
}

/**
 * Set the Enable State Machine (ENSM) mode.
 * @param phy The AD9361 current state structure.
 * @param mode The ENSM mode (SLEEP, ALERT, FDD, PINCTRL).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_en_state_machine_mode (struct ad9361_rf_phy *phy,
										  uint32_t mode)
{
	int32_t ret;

	ad9361_set_ensm_mode(phy, phy->pdata->fdd, false);
	ret = ad9361_ensm_set_state(phy, mode, false);

	return ret;
}

/**
 * Get the Enable State Machine (ENSM) mode.
 * @param phy The AD9361 current state structure.
 * @param mode A variable to store the selected ENSM mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_en_state_machine_mode (struct ad9361_rf_phy *phy,
										  uint32_t *mode)
{
	*mode = phy->curr_ensm_state;

	return 0;
}

/**
 * Set the receive RF gain for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (1, 2).
 * @param gain_db The RF gain (dB).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_rf_gain (struct ad9361_rf_phy *phy,
							   uint8_t ch, int32_t gain_db)
{
	struct rf_rx_gain rx_gain = {0};
	int32_t ret = 0;

	rx_gain.gain_db = gain_db;
	ret = ad9361_set_rx_gain(phy, ch + 1, &rx_gain);

	return ret;
}

/**
 * Get current receive RF gain for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (1, 2).
 * @param gain_db A variable to store the RF gain (dB).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rf_gain (struct ad9361_rf_phy *phy,
							   uint8_t ch, int32_t *gain_db)
{
	struct rf_rx_gain rx_gain = {0};
	int32_t ret = 0;

	ret = ad9361_get_rx_gain(phy, ch + 1, &rx_gain);
	*gain_db = rx_gain.gain_db;

	return ret;
}

/**
 * Set the RX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz The desired bandwidth (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_rf_bandwidth (struct ad9361_rf_phy *phy,
									uint32_t bandwidth_hz)
{
	int32_t ret = 0;

	if (phy->current_rx_bw_Hz != bandwidth_hz)
		ret = ad9361_update_rf_bandwidth(phy, bandwidth_hz,
				phy->current_tx_bw_Hz);
	else
		ret = 0;

	return ret;
}

/**
 * Get the RX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz A variable to store the bandwidth value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rf_bandwidth (struct ad9361_rf_phy *phy,
									uint32_t *bandwidth_hz)
{
	*bandwidth_hz = phy->current_rx_bw_Hz;

	return 0;
}

/**
 * Set the RX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz The desired frequency (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_sampling_freq (struct ad9361_rf_phy *phy,
									 uint32_t sampling_freq_hz)
{
	int32_t ret;
	uint32_t rx[6], tx[6];

	ret = ad9361_calculate_rf_clock_chain(phy, sampling_freq_hz,
		phy->rate_governor, rx, tx);
	if (ret < 0)
		return ret;

	ad9361_set_trx_clock_chain(phy, rx, tx);

	ret = ad9361_update_rf_bandwidth(phy, phy->current_rx_bw_Hz,
					phy->current_tx_bw_Hz);

	return ret;
}

/**
 * Get current RX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_sampling_freq (struct ad9361_rf_phy *phy,
									 uint32_t *sampling_freq_hz)
{
	*sampling_freq_hz = (uint32_t)clk_get_rate(phy,
										phy->ref_clk_scale[RX_SAMPL_CLK]);

	return 0;
}

/**
 * Set the RX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz The desired frequency (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_lo_freq (struct ad9361_rf_phy *phy,
							   uint64_t lo_freq_hz)
{
	int32_t ret;

	ret = clk_set_rate(phy, phy->ref_clk_scale[RX_RFPLL],
			   ad9361_to_clk(lo_freq_hz));

	return ret;
}

/**
 * Get current RX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_lo_freq (struct ad9361_rf_phy *phy,
							   uint64_t *lo_freq_hz)
{
	*lo_freq_hz = ad9361_from_clk(clk_get_rate(phy,
										phy->ref_clk_scale[RX_RFPLL]));

	return 0;
}

/**
 * Get the RSSI for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (1, 2).
 * @param rssi A variable to store the RSSI.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rssi (struct ad9361_rf_phy *phy,
							uint8_t ch, struct rf_rssi *rssi)
{
	int32_t ret;

	rssi->ant = ch + 1;
	rssi->duration = 1;
	ret = ad9361_read_rssi(phy, rssi);

	return ret;
}

/**
 * Set the gain control mode for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (1, 2).
 * @param gc_mode The gain control mode (GAIN_MGC, GAIN_FASTATTACK_AGC,
 *                GAIN_SLOWATTACK_AGC, GAIN_HYBRID_AGC).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_gain_control_mode (struct ad9361_rf_phy *phy,
										 uint8_t ch, uint8_t gc_mode)
{
	struct rf_gain_ctrl gc = {0};

	gc.ant = ch + 1;
	gc.mode = phy->agc_mode[ch] = gc_mode;

	ad9361_set_gain_ctrl_mode(phy, &gc);

	return 0;
}

/**
 * Get the gain control mode for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (1, 2).
 * @param gc_mode A variable to store the gain control mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_gain_control_mode (struct ad9361_rf_phy *phy,
										 uint8_t ch, uint8_t *gc_mode)
{
	*gc_mode = phy->agc_mode[ch];

	return 0;
}

/**
 * Set the RX FIR filter configuration.
 * @param phy The AD9361 current state structure.
 * @param fir_cfg FIR filter configuration.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_fir_config (struct ad9361_rf_phy *phy,
								  AD9361_RXFIRConfig fir_cfg)
{
	int32_t ret;

	ret = ad9361_load_fir_filter_coef(phy, (enum fir_dest)(fir_cfg.rx | FIR_IS_RX),
			fir_cfg.rx_gain, 128, fir_cfg.rx_coef);
	phy->rx_fir_dec = fir_cfg.rx_dec;

	return ret;
}

/**
 * Enable/disable the RX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_fir_en_dis (struct ad9361_rf_phy *phy,
								  uint8_t en_dis)
{
	int32_t ret = 0;

	if(phy->bypass_rx_fir == !en_dis)
		return ret;

	phy->bypass_rx_fir = !en_dis;
	ret = ad9361_validate_enable_fir(phy);
	if (ret < 0) {
		phy->bypass_rx_fir = true;
	}

	return ret;
}

/**
 * Get the status of the RX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_fir_en_dis (struct ad9361_rf_phy *phy,
								  uint8_t *en_dis)
{
	*en_dis = !phy->bypass_rx_fir;

	return 0;
}

/**
 * Enable/disable the RX RFDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_rfdc_track_en_dis (struct ad9361_rf_phy *phy,
										 uint8_t en_dis)
{
	int32_t ret = 0;

	if(phy->rfdc_track_en == en_dis)
		return ret;

	phy->rfdc_track_en = en_dis;
	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
		phy->rfdc_track_en, phy->quad_track_en);

	return ret;
}

/**
 * Get the status of the RX RFDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rfdc_track_en_dis (struct ad9361_rf_phy *phy,
										 uint8_t *en_dis)
{
	*en_dis = phy->rfdc_track_en;

	return 0;
}

/**
 * Enable/disable the RX BasebandDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_bbdc_track_en_dis (struct ad9361_rf_phy *phy,
										 uint8_t en_dis)
{
	int32_t ret = 0;

	if(phy->bbdc_track_en == en_dis)
		return ret;

	phy->bbdc_track_en = en_dis;
	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
		phy->rfdc_track_en, phy->quad_track_en);

	return ret;
}

/**
 * Get the status of the RX BasebandDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_bbdc_track_en_dis (struct ad9361_rf_phy *phy,
										 uint8_t *en_dis)
{
	*en_dis = phy->bbdc_track_en;

	return 0;
}

/**
 * Enable/disable the RX Quadrature Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_quad_track_en_dis (struct ad9361_rf_phy *phy,
										 uint8_t en_dis)
{
	int32_t ret = 0;

	if(phy->quad_track_en == en_dis)
		return ret;

	phy->quad_track_en = en_dis;
	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
		phy->rfdc_track_en, phy->quad_track_en);

	return ret;
}

/**
 * Get the status of the RX Quadrature Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_quad_track_en_dis (struct ad9361_rf_phy *phy,
										 uint8_t *en_dis)
{
	*en_dis = phy->quad_track_en;

	return 0;
}

/**
 * Set the transmit attenuation for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (1, 2).
 * @param attenuation_mdb The attenuation (mdB).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_attenuation (struct ad9361_rf_phy *phy,
								   uint8_t ch, uint32_t attenuation_mdb)
{
	int32_t ret;

	ret = ad9361_set_tx_atten(phy, attenuation_mdb,
		ch == 0, ch == 1,
		!phy->pdata->update_tx_gain_via_alert);

	return ret;
}

/**
 * Get current transmit attenuation for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (1, 2).
 * @param attenuation_mdb A variable to store the attenuation value (mdB).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_attenuation (struct ad9361_rf_phy *phy,
								   uint8_t ch, uint32_t *attenuation_db)
{
	uint32_t ret;

	ret = ad9361_get_tx_atten(phy, ch + 1);
	if(ret < 0)
		return EINVAL;
	*attenuation_db = ret;

	return 0;
}

/**
 * Set the TX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz The desired bandwidth (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_rf_bandwidth (struct ad9361_rf_phy *phy,
									uint32_t  bandwidth_hz)
{
	int32_t ret = 0;

	if (phy->current_tx_bw_Hz != bandwidth_hz)
		ret = ad9361_update_rf_bandwidth(phy,
				phy->current_rx_bw_Hz, bandwidth_hz);
	else
		ret = 0;

	return ret;
}

/**
 * Get the TX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz A variable to store the bandwidth value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_rf_bandwidth (struct ad9361_rf_phy *phy,
									uint32_t *bandwidth_hz)
{
	*bandwidth_hz = phy->current_tx_bw_Hz;

	return 0;
}

/**
 * Set the TX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz The desired frequency (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_sampling_freq (struct ad9361_rf_phy *phy,
									 uint32_t sampling_freq_hz)
{
	int32_t ret;
	uint32_t rx[6], tx[6];

	ret = ad9361_calculate_rf_clock_chain(phy, sampling_freq_hz,
		phy->rate_governor, rx, tx);
	if (ret < 0)
		return ret;

	ad9361_set_trx_clock_chain(phy, rx, tx);

	ret = ad9361_update_rf_bandwidth(phy, phy->current_rx_bw_Hz,
					phy->current_tx_bw_Hz);

	return ret;
}

/**
 * Get current TX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_sampling_freq (struct ad9361_rf_phy *phy,
									 uint32_t *sampling_freq_hz)
{
	*sampling_freq_hz = (uint32_t)clk_get_rate(phy,
										phy->ref_clk_scale[TX_SAMPL_CLK]);

	return 0;
}

/**
 * Set the TX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz The desired frequency (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_lo_freq (struct ad9361_rf_phy *phy,
							   uint64_t lo_freq_hz)
{
	int32_t ret;

	ret = clk_set_rate(phy, phy->ref_clk_scale[TX_RFPLL],
			   ad9361_to_clk(lo_freq_hz));

	return ret;
}

/**
 * Get current TX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_lo_freq (struct ad9361_rf_phy *phy,
							   uint64_t *lo_freq_hz)
{
	*lo_freq_hz = ad9361_from_clk(clk_get_rate(phy,
										phy->ref_clk_scale[TX_RFPLL]));

	return 0;
}

/**
 * Set the TX FIR filter configuration.
 * @param phy The AD9361 current state structure.
 * @param fir_cfg FIR filter configuration.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_fir_config (struct ad9361_rf_phy *phy,
								  AD9361_TXFIRConfig fir_cfg)
{
	int32_t ret;

	ret = ad9361_load_fir_filter_coef(phy, (enum fir_dest)fir_cfg.tx,
			fir_cfg.tx_gain, 128, fir_cfg.tx_coef);
	phy->tx_fir_int = fir_cfg.tx_int;

	return ret;
}

/**
 * Enable/disable the TX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_fir_en_dis (struct ad9361_rf_phy *phy,
								  uint8_t en_dis)
{
	int32_t ret = 0;

	if(phy->bypass_tx_fir == !en_dis)
		return ret;

	phy->bypass_tx_fir = !en_dis;
	ret = ad9361_validate_enable_fir(phy);
	if (ret < 0) {
		phy->bypass_tx_fir = true;
	}

	return ret;
}

/**
 * Get the status of the TX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_fir_en_dis (struct ad9361_rf_phy *phy,
								  uint8_t *en_dis)
{
	*en_dis = !phy->bypass_tx_fir;

	return 0;
}

/**
 * Set the RX and TX path rates.
 * @param phy The AD9361 state structure.
 * @param rx_path_clks RX path rates buffer.
 * @param tx_path_clks TX path rates buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_trx_path_clks(struct ad9361_rf_phy *phy,
	uint32_t *rx_path_clks,
	uint32_t *tx_path_clks)
{
	int32_t ret;

	ret = ad9361_set_trx_clock_chain(phy, rx_path_clks, tx_path_clks);
	if (ret < 0)
		return ret;

	ret = ad9361_update_rf_bandwidth(phy, phy->current_rx_bw_Hz,
					phy->current_tx_bw_Hz);

	return ret;
}

/**
 * Get the RX and TX path rates.
 * @param phy The AD9361 state structure.
 * @param rx_path_clks RX path rates buffer.
 * @param tx_path_clks TX path rates buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_trx_path_clks(struct ad9361_rf_phy *phy,
	uint32_t *rx_path_clks,
	uint32_t *tx_path_clks)
{
	return ad9361_get_trx_clock_chain(phy, rx_path_clks, tx_path_clks);
}
