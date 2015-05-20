/*
 * clock.h
 *
 *  Created on: Jun 18, 2014
 *      Author: user
 */

#ifndef S3MA_CLOCK_H_
#define S3MA_CLOCK_H_


void s3ma_setup_pll(void);
void s3ma_ddr_clock_enable(void);
void s3ma_ddr_clock_disable(void);
void s3ma_sdhc_clk_enable(void);
void s3ma_sdhc_clk_disable(void);

#endif /* CLOCK_H_ */
