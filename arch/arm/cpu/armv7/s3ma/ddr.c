/*
 * ddr.c
 *	Exelis Inc. 2014
 *  Created on: Jun 18, 2014
 *      Author: Dmitiry Goldyuk
 */
#include <common.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/s3ma-regs.h>
#include <asm/arch/ddr.h>
#include <asm/arch/sys_proto.h>


static int s3ma_init_dmc_main(void)				// TODO: turn into separate LPDDR2_200 and LDDR2_400 functions
{
	//int status=ARMLIB_PASS;

	// the below initialization flow is based upon that provided with the DMC block
	// level verification environment; the block level environment includes several
	// c++ functions for calculating all of the below parameters; in leu of porting
	// these function to our chip-level verification environment (takes some time),
	// we simply updated the block-level verification environment so as to "use our
	// settings" and thus calculate the parameter values for us (values pulled from
	// block-level all_trans compile transcript - 'grep "DUT."').  Our settings:
	//
	// - SDRAM speed grade     	<-- SG5 (i.e. LPDDR2_400)
	// - SDRAM latency     		<-- RL=4, WL=2
	// - SDRAM burst length    	<-- BL=16
	//

	// BEGIN DMC INITIALIZATION

	// -------------------------------
	// Initial Error Checking
	// -------------------------------

	// simple check to make sure we can talk to DMC (if fail, stop immediately as no point in continuing)
	if(0x03111200 != readl(MEMC_CONFIG))
	{
		return -1;
	}

	// -------------------------------
	// Send DMC to CONFIG State
	// -------------------------------

	if(readl(MEMC_STATUS) != 0) {
		writel(0x00000000,MEMC_CMD);				// Waiting until DMC is in config status
		while(readl(MEMC_STATUS) != 0);
	}


	// -------------------------------
	// Setup DMC Timing Parameters				// TODO: check to see if below timing params are checked by Micron U80 model
	// -------------------------------

	writel(0xc3,T_REFI);				// refresh interval timing
	writel(0x002a0012,T_RFC);			// AUTOREFRESH to another command
	writel(0x2,T_MRR);					// note: mrr burst length is fixed at 2 for LPDDR2
	writel(0x5,T_MRW);					// MRW to another commmand
	writel(0x4,T_RCD);					// ACTIVATE to READ/WRITE of same bank
	writel(0x9,T_RAS);					// ACTIVATE to PRECHARGE of same bank
	writel(0x4,T_RP);					// PRECHARGE to another command of same bank
	writel(0x5,T_RPALL);				// PRECHARGE to another command of same chip
	writel(0x2,T_RRD);					// ACTIVATE to ACTIVATE of same chip
	writel(0xa,T_FAW);					// no more than four ACTIVATE commands within tFAW window
	writel(0xa,T_RTR);					// READ to READ of different chips
	writel(0xd,T_RTW);					// READ to WRITE
	writel(0x9,T_RTP);					// READ to PRECHARGE
	writel(0xe,T_WR);					// WRITE to another command of same bank
	writel(0x0008000d,T_WTR);			// WRITE to READ of different and same chip
	writel(0x000b0000,T_WTW);			// WRITE to WRITE of different chip
	writel(0x4,READ_LATENCY);			// DRAM READ LATENCY
	writel(0x2,WRITE_LATENCY);			// DRAM WRITE LATENCY
	writel(0x2,T_RDDATA_EN);			// READ to dfi_rddata_en	** DMC block-level TB uses 0x1   ; multiPHY requires 0x2 **
	writel(0x00000102,T_PHYWRLAT);		// WRITE to dfi_wrdata_en	** DMC block-level TB uses 0x202 ; multiPHY requires 0x102 **
	writel(0x00040002,T_XP);			// exit power down
	writel(0x3,T_ESR);					// enter self refresh
	writel(0x002c002c,T_XSR);			// exit self refresh
	writel(0x5,T_SRCKD);				// self refresh to DRAM clock disable
	writel(0x5,T_CKSRD);				// DRAM clock enable to exit self refresh
	writel(0x5,T_ECKD);					// enter DRAM clock disable
	writel(0x5,T_XCKD);					// exit DRAM clock disable
	writel(0x4,T_EP);					// enter power down

	// -------------------------------
	// Setup DMC Misc Parameters
	// -------------------------------

	writel(0x00001001,RDLVL_CONTROL);			// set bit[0] for PHY Independent mode (all other bits for non PHY Independent mode)
	writel(0x0,REFRESH_CONTROL);				// start with all bank refresh, as appears to require less "overhead"
	writel(0x01030302,ADDRESS_CONTROL);			// 0 channel bits, 1 chip bits, 3 bank bits, 14 row bits, 10 column bits
	writel(0x00000000,DECODE_CONTROL);			// programmable translation off, page[1:0]=sys_addr[13:12], chan/chip/row/bank
	writel(0x22000302,FORMAT_CONTROL);			// align to 4 beat burst boundary, terminate 4 DMC cycles, burst is 4 DMC cycles, DQ[31:0]

	// -------------------------------
	// Send DMC to GO State
	// -------------------------------

	writel(0x00000003,MEMC_CMD);

	//Wait until DMC is in GO state
	while (0x3 != readl(MEMC_STATUS));

	//DMC is in GO state ...

	return 0;
}

//-----------------------------------------------------------------------------
// Function
//-----------------------------------------------------------------------------
static int s3ma_init_phy_main(void)
{

	// the below initialization flow is based upon that provided with the PHY block
	// level verification environment and as such performs PHY based SDRAM init and
	// subsequent training; for reasons similiar to those described for the DMC, we
	// choose to start with simply grabbing the resulting parameters from the block
	// level demo_wr simulation transcript (we'll port verilog tasks later).  It is
	// important to note that the values shown below are based upon the same set of
	// SDRAM options (i.e. SG5, RL=4, WL=2 and BL=16) as used to determine the DMC's
	// parameters.  Although not yet confirmed, this is likely required for the MRx
	// parameter settings (else PHY configs SDRAM different from what DMC expects);
	// meanwhile it appears that the DTPRx parameters (and underlying T_XYZ) can be
	// different between the DMC and PHY (suspect because each uses its own timings
	// to acess the SDRAM - recall that PHY can directly access SDRAM w/out DMC).
	//
	// Details will be added once available (but for now the below seems to work).

	// -------------------------------
	// Initial Error Checking
	// -------------------------------
	if(0x0020020b != readl(PUBL_RIDR))
	{
		return -1;
	}

	// -------------------------------
	// PHY Configuration
	// -------------------------------

	writel(0x0000000c,PUBL_DCR);				// LPDDR2-S4 8-Bank mode
	writel(0x00000044,PUBL_MR1);				// write recovery of 4, wrap, sequential burst, Burst Length (BL) of 16
	writel(0x00000002,PUBL_MR2);				// Read Latency (RL) of 5 and Write Latency (WL) of 2
	writel(0x00000002,PUBL_MR3);				// drive strength (not needed for simulation bring-up, but keep anyway)
	writel(0x1a49444a,PUBL_DTPR0);				// SDRAM timing parameters
	writel(0x101a1050,PUBL_DTPR1);				// SDRAM timing parameters
	writel(0x064798c8,PUBL_DTPR2);				// SDRAM timing parameters
	writel(0x018c2e02,PUBL_PGCR) ;				// passive windowing mode, strobe drift compensation disabled, rank 0 & 1 enabled for training
	writel(0x00000c40,PUBL_DXCCR);				// 500 ohm on-die pull-down/pull-up resistor for DQS/DQS#
	writel(0xfa00013f,PUBL_DSGCR);				// DQS Gate Early and DQS Gate Extension
	writel(0x3db00009,PUBL_DX0DQSTR);			// DQS Gate Timing for byte0
	writel(0x3db00009,PUBL_DX1DQSTR);			// DQS Gate Timing for byte1
	writel(0x3db00009,PUBL_DX2DQSTR);			// DQS Gate Timing for byte2
	writel(0x3db00009,PUBL_DX3DQSTR);			// DQS Gate Timing for byte3
	writel(0x0020051b,PUBL_PTR0);				// reduced DLL lock time	 ** FOR SIMULATION ONLY **
	writel(0x05000064,PUBL_PTR1);				// reduced INIT0 and INIT1	 ** FOR SIMULATION ONLY **
	writel(0x0078006e,PUBL_PTR2);				// reduced INIT2 and INIT3	 ** FOR SIMULATION ONLY **

	// -------------------------------
	// PHY INIT PHASE 1
	// -------------------------------

	writel(0x0000001f,PUBL_PIR);				// trigger ITM reset, PHY ZQ calibration, wait for DLL lock, DLL reset

	// Wait for PHY INIT PHASE 1 complete
	__udelay(1);								// wait min 10 cfg clocks (192MHz->~5ns->~50ns min) before polling status
	int phy_init1_status=0;
	while (phy_init1_status != 0x7)				// ZQ calibration done, DLL lock done, requested init routines completed
	{
		phy_init1_status = readl(PUBL_PGSR);
		if (phy_init1_status & 0x000003E0)		// TODO: confirm that error trap works
		{
			// ERROR: error during PHY INIT PHASE 1
			return 1;
		}
	}
	// PHY INIT PHASE 1 complete

	// -------------------------------
	// PHY INIT PHASE 2
	// -------------------------------

	// Write PHY PUBL_PIR
	writel(0x000000c1,PUBL_PIR);				// trigger SDRAM INIT with DQS TRAINING (note: read data eye training not supported/required by PHY, section 4.3.3 PUBL databook)

	// Waiting for PHY INIT PHASE 2 complete
	__udelay(1);								// wait min 10 cfg clocks (192MHz->~5ns->~50ns min) before polling status
	int phy_init2_status=0;
	while (phy_init2_status!=0x1f)				// SDRAM init done, DQS training done, requested init routines completed
	{
		phy_init2_status = readl(PUBL_PGSR);
		if (phy_init2_status & 0x000003E0)		// TODO: confirm that error trap works
		{
			// ERROR: error during PHY INIT PHASE 2
			return 2;
		}
	}

	// PHY INIT PHASE 2 complete

	__udelay(1);						// PHY block-level TB used small delay before continuing, thus we'll do the same

	// END PHY INITIALIZATION

	return 0;
}


void s3ma_ddr_setup(void)
{
	int status = 0;

	do{
		status = s3ma_init_dmc_main();

		if(0 != status)
		{
			//Report an error here
			debug("%s:%d: Could not initialize DMC, error %d\n", __func__, __LINE__, status);
			break;
		}

		status = s3ma_init_phy_main();
		if(0 != status)
		{
			//Report an error here
			debug("%s:%d: Could not initialize DDR PHY, error %d\n", __func__, __LINE__, status);
			break;
		}

		debug("Trying to access DDR space\n");
		debug("%x:%x\n",0x80000000, *((uint32_t*)0x80000000));
	}while(0);
}



