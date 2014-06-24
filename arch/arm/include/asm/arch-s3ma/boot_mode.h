/*
 * boot_mode.h
 *
 *  Created on: Jun 24, 2014
 *      Author: Dmitriy Goldyuk
 */

#ifndef BOOT_MODE_H_
#define BOOT_MODE_H_

#define POR_CONFIG_LEN	256		//Length in 32-bit words

#define POR_CONFIG\
		0x00e80000, 		/* por_cfg[0031:0000] = pllconfig0[31:00]= 32'h00e80000				*/\
		0x00000002,			/* por_cfg[0063:0032] = pllconfig1[31:00] = 32'h00000002			*/\
		0x00000000,			/* por_cfg[0095:0064] = pllconfig2[31:00] = 32'h00000000			*/\
		0x00000000,			/* por_cfg[0125:0096] = mpc_cfg_boot[29:00] = 32'h00000000			*/\
		0x00000000,			/* por_cfg[0159:0148] = mpc_filterstart_boot[31:20] = 32'h00000000	*/\
		0x00000000,			/* por_cfg[0191:0180] = mpc_filterend_boot  [31:20] = 32'h00000000	*/\
		0x3be00000,			/* por_cfg[0223:0205] = mpc_periphbase_boot [31:13] = 32'h3be00000	*/\
		0x080d1000,			/* por_cfg[0255:0236] = mpc_regfilebase_boot[31:12] = 32h080d1000	*/\
		/* 512'h5361_6D706C65_206D6573_73616765_20666F72_206B6579_6C656E3D_626C6F63_6B6C656E; 	*/\
		0x6C65206D,																				\
		0x53616D70,																				\
		/* 6573_73616765_2066     */ 															\
		0x67652066,                  															\
		0x65737361,                  															\
		/* 6F72_206B6579_6C65     */ 															\
		0x65796C65,                  															\
		0x6F72206B,                  															\
		/* 6E3D_626C6F63_6B6C     */ 															\
		0x6F636B6C,                  															\
		0x6E3D626C,                  															\
		/* 656E8000_00000000      */ 															\
		0x00000000,                  															\
		0x656E8000,                  															\
		/* 00000000_00000000      */ 															\
		0x00000000,                  															\
		0x00000000,                  															\
		/* 00000000_00000000      */   															\
		0x00000000,																				\
		0x00000000,																				\
		/* 00000000_00000310 */																	\
		0x00000310,																				\
		0x00000000,																				\
		/* 256'h8BB9A1DB_9806F20D_F7F77B82_138C7914_D174D59E_13DC4D01_69C9057B_133E1D62*/		\
		0x9806F20D,																				\
		0x8BB9A1DB,																				\
		/* F7F77B82_138C7914 */																	\
		0x138C7914,																				\
		0xF7F77B82,																				\
		/* D174D59E_13DC4D01 */																	\
		0x13DC4D01,																				\
		0xD174D59E,																				\
		/* 69C9057B_133E1D62 */																	\
		0x133E1D62,																				\
		0x69C9057B																				\
		/* End of POR Config */


extern const uint32_t por_config[];



#endif /* BOOT_MODE_H_ */
