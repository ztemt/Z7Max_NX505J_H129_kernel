/*

SiI8620 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

/*
   @file si_8620_drv.c
*/

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>

#include "si_fw_macros.h"
#include "si_app_devcap.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_8620_internal_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "si_mhl_callback_api.h"
#include "si_8620_drv.h"
#include "mhl_linux_tx.h"
#include "si_8620_regs.h"
#include "mhl_supp.h"
#include "platform.h"
#include "../mdss_hdmi_mhl.h"

bool Sii8620_MHL_Mode = false;
/* external functions */
void	si_dump_important_regs(struct drv_hw_context *hw_context);

/* Local functions */
#ifdef USE_HW_TIMER
static int int_1_isr(struct drv_hw_context *hw_context, uint8_t int_1_status);
#endif
static int int_2_isr(struct drv_hw_context *hw_context, uint8_t int_2_status);
static int int_3_isr(struct drv_hw_context *hw_context, uint8_t int_3_status);
static int int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status);
static int int_5_isr(struct drv_hw_context *hw_context, uint8_t int_5_status);
static int int_8_isr(struct drv_hw_context *hw_context, uint8_t intr_8_status);
static int int_9_isr(struct drv_hw_context *hw_context, uint8_t int_9_status);
#ifdef DO_HDCP
static int hdcp_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status);
static int hdcp2_isr(struct drv_hw_context *hw_context, uint8_t intr_status);
#endif
static int g2wb_err_isr(struct drv_hw_context *hw_context, uint8_t intr_stat);
static int g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat);
static int mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int);
static int mhl_cbus_err_isr(struct drv_hw_context *hw_context,
							uint8_t cbus_err_int);
static int mhl3_block_isr(struct drv_hw_context *hw_context, uint8_t status);
static int coc_isr(struct drv_hw_context *hw_context,uint8_t coc_int_status);
static int tdm_isr(struct drv_hw_context *hw_context,uint8_t intr_status);
static int int_link_trn_isr(struct drv_hw_context *hw_context,uint8_t intr_status);
static void enable_intr(struct drv_hw_context *hw_context, uint8_t intr_num,
							uint8_t intr_mask);
static void switch_to_d3(struct drv_hw_context *hw_context,bool do_interrupt_clear);
static void disconnect_mhl(struct drv_hw_context *hw_context,bool do_interrupt_clear);
#ifdef DO_HDCP
static void start_hdcp(struct drv_hw_context *hw_context);
static void start_hdcp_content_type(struct drv_hw_context *hw_context);
#endif
static void stop_video(struct drv_hw_context *hw_context);
static int get_device_rev(struct drv_hw_context *hw_context);
static void unmute_video(struct drv_hw_context *hw_context);
static int set_hdmi_params(struct mhl_dev_context *dev_context);
static void hsic_init(struct drv_hw_context *hw_context);

static void disable_gen2_write_burst_rcv(struct drv_hw_context *hw_context);
static void disable_gen2_write_burst_xmit(struct drv_hw_context *hw_context);
static void si_mhl_tx_drv_start_gen2_write_burst(struct drv_hw_context *hw_context);

static void si_mhl_tx_drv_set_lowest_tmds_link_speed(struct mhl_dev_context *dev_context,
									uint32_t pixel_clock_frequency,
									uint8_t bits_per_pixel);

static int start_video(struct drv_hw_context *hw_context);
static int get_cbus_connection_status(struct drv_hw_context *hw_context);

//extern int hdmi_tx_set_mhl_hpd(struct platform_device *pdev, uint8_t on);

/* Local data */

#define DDC_ABORT_THRESHOLD	10
static int ddc_abort_count = 0;

#define MSC_ABORT_THRESHOLD	10
static int msc_abort_count = 0;

#define REG_EMSC_INTR_0_STAT	TX_PAGE_3,0x2C
#define REG_EMSC_INTR_0_MASK	TX_PAGE_3,0x2D

struct intr_tbl {
	uint8_t aggr_stat_index;
	uint8_t aggr_stat_id_bit;
	uint8_t mask;
	uint8_t mask_page;
	uint8_t mask_offset;
	uint8_t stat_page;
	uint8_t stat_offset;
	int (*isr)(struct drv_hw_context *, uint8_t status);
	char *name;
};

typedef enum {
	 FAST_INTR_STAT = 0
	,L1_INTR_STAT_0
	,L1_INTR_STAT_1
	,L1_INTR_STAT_2
	,L1_INTR_STAT_3
	,L1_INTR_STAT_4
	,L1_INTR_STAT_5
	/* this one MUST be last */
	,NUM_AGGREGATED_INTR_REGS
} l1_intr_stat_enums_t;

struct intr_tbl g_intr_tbl[] = {
	{ L1_INTR_STAT_2,0x10,0, REG_PAGE_5_CBUS_DISC_INTR0_MASK,	REG_PAGE_5_CBUS_DISC_INTR0,	int_4_isr,			"DISC"	},
	{ L1_INTR_STAT_1,0x08,0, REG_PAGE_5_MDT_INT_1_MASK,			REG_PAGE_5_MDT_INT_1,		g2wb_err_isr,		"G2WB"	},
	{ L1_INTR_STAT_1,0x04,0, REG_PAGE_5_MDT_INT_0_MASK,			REG_PAGE_5_MDT_INT_0,		g2wb_isr,			"G2WB"	},
	{ L1_INTR_STAT_5,0x08,0, REG_PAGE_7_COC_INTR_MASK,			REG_PAGE_7_COC_INTR,		coc_isr,			"COC"	},
	{ L1_INTR_STAT_4,0x04,0, REG_PAGE_1_TRXINTMH,				REG_PAGE_1_TRXINTH,			tdm_isr,			"TDM"	},
	{ L1_INTR_STAT_1,0x01,0, REG_PAGE_5_CBUS_INT_0_MASK,		REG_PAGE_5_CBUS_INT_0,		mhl_cbus_isr,		"MSC"	},
	{ L1_INTR_STAT_1,0x02,0, REG_PAGE_5_CBUS_INT_1_MASK,		REG_PAGE_5_CBUS_INT_1,		mhl_cbus_err_isr,	"MERR"	},
	{ L1_INTR_STAT_2,0x40,0, REG_PAGE_3_EMSCINTRMASK,			REG_PAGE_3_EMSCINTR,		mhl3_block_isr,		"BLOCK" },
	{ L1_INTR_STAT_2,0x80,0, REG_EMSC_INTR_0_MASK,				REG_EMSC_INTR_0_STAT,		int_link_trn_isr,	"LTRN"	},
	{ L1_INTR_STAT_0,0x20,0, REG_PAGE_0_INTR8_MASK,				REG_PAGE_0_INTR8,			int_8_isr,			"INFR"	},
#ifdef DO_HDCP
	{ L1_INTR_STAT_0,0x80,0, REG_PAGE_6_TPI_INTR_EN,			REG_PAGE_6_TPI_INTR_ST0,	hdcp_isr,			"HDCP"	},
	{ L1_INTR_STAT_3,0x01,0, REG_PAGE_3_HDCP2X_INTR0_MASK,		REG_PAGE_3_HDCP2X_INTR0,	hdcp2_isr,			"HDCP2"	},
#endif
	{ L1_INTR_STAT_0,0x40,0, REG_PAGE_2_INTR9_MASK,				REG_PAGE_2_INTR9,			int_9_isr,			"EDID"	},
	{ L1_INTR_STAT_0,0x04,0, REG_PAGE_0_INTR3_MASK,				REG_PAGE_0_INTR3,			int_3_isr,			"DDC"	},
	{ L1_INTR_STAT_0,0x08,0, REG_PAGE_0_INTR5_MASK,				REG_PAGE_0_INTR5,			int_5_isr,			"SCDT"	},
	{ L1_INTR_STAT_0,0x02,0, REG_PAGE_0_INTR2_MASK,				REG_PAGE_0_INTR2,			int_2_isr,			"INT2"	},
#ifdef USE_HW_TIMER
	{ L1_INTR_STAT_0,0x01,0, REG_PAGE_0_INTR1_MASK,				REG_PAGE_0_INTR1,			int_1_isr,			"TIMR"	},
#endif
};

typedef enum {
	INTR_DISC,
	INTR_G2WB_ERR,
	INTR_G2WB,
	INTR_COC,
	INTR_TDM,
	INTR_MSC,
	INTR_MERR,
	INTR_BLOCK,
	INTR_LINK_TRAINING,
	INTR_INFR,
	#ifdef DO_HDCP
	INTR_HDCP,
	INTR_HDCP2,
	#endif
	INTR_EDID,
	INTR_DDC,
	INTR_SCDT,
	INTR_USER,
#ifdef USE_HW_TIMER
	INTR_TIMR,
#endif
	MAX_INTR
} intr_nums_t;

#define BIT_RGND_READY_INT						BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT6
#define BIT_CBUS_MHL12_DISCON_INT				BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT5
#define BIT_CBUS_MHL3_DISCON_INT				BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT4
#define BIT_NOT_MHL_EST_INT						BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT3
#define BIT_MHL_EST_INT							BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT2
#define BIT_MHL3_EST_INT						BIT_PAGE_5_CBUS_DISC_INTR0_CBUS_DISC_INTR0_STAT1

#define BIT_RGND_READY_INT_MASK					BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK6
#define BIT_CBUS_MHL12_DISCON_INT_MASK			BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK5
#define BIT_CBUS_MHL3_DISCON_INT_MASK			BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK4
#define BIT_NOT_MHL_EST_INT_MASK				BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK3
#define BIT_MHL_EST_INT_MASK					BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK2
#define BIT_MHL3_EST_INT_MASK					BIT_PAGE_5_CBUS_DISC_INTR0_MASK_CBUS_DISC_INTR0_MASK1



#ifdef USE_HW_TIMER
#define BIT_HW_TIMER_POP						BIT_PAGE_0_INTR1_INTR1_STAT7
#endif

#define BIT_INTR_SCDT_CHANGE					BIT_PAGE_0_INTR5_INTR5_STAT0

#define BIT_INTR8_CEA_NEW_VSI					BIT_PAGE_0_INTR8_MASK_INTR8_MASK2
#define BIT_INTR8_CEA_NEW_AVI					BIT_PAGE_0_INTR8_MASK_INTR8_MASK1

#define BIT_PAGE_0_VID_OVRRD_ENABLE_AUTO_LINK_MODE_UPDATE 0x08

#define BIT_MDT_RFIFO_DATA_RDY					BIT_PAGE_5_MDT_INT_0_MDT_INT_0_0
#define BIT_MDT_IDLE_AFTER_HAWB_DISABLE			BIT_PAGE_5_MDT_INT_0_MDT_INT_0_2
#define BIT_MDT_XFIFO_EMPTY						BIT_PAGE_5_MDT_INT_0_MDT_INT_0_3

#define BIT_MDT_RCV_TIMEOUT						BIT_PAGE_5_MDT_INT_1_MDT_INT_1_0
#define BIT_MDT_RCV_SM_ABORT_PKT_RCVD			BIT_PAGE_5_MDT_INT_1_MDT_INT_1_1
#define BIT_MDT_RCV_SM_ERROR					BIT_PAGE_5_MDT_INT_1_MDT_INT_1_2

#define BIT_MDT_XMIT_TIMEOUT					BIT_PAGE_5_MDT_INT_1_MDT_INT_1_5
#define BIT_MDT_XMIT_SM_ABORT_PKT_RCVD			BIT_PAGE_5_MDT_INT_1_MDT_INT_1_6
#define BIT_MDT_XMIT_SM_ERROR					BIT_PAGE_5_MDT_INT_1_MDT_INT_1_7

#define BIT_CBUS_DDC_PEER_ABORT					BIT_PAGE_5_DDC_ABORT_INT_DDC_ABORT_INT_STAT7

#define BIT_CBUS_MSC_MT_DONE_NACK				BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK7
#define BIT_CBUS_MSC_MR_SET_INT					BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK6
#define BIT_CBUS_MSC_MR_WRITE_BURST				BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK5
#define BIT_CBUS_MSC_MR_MSC_MSG					BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK4
#define BIT_CBUS_MSC_MR_WRITE_STAT				BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK3
#define BIT_CBUS_HPD_CHG						BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK2
#define BIT_CBUS_MSC_MT_DONE					BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK1
#define BIT_CBUS_CNX_CHG						BIT_PAGE_5_CBUS_INT_0_MASK_CBUS_INT_0_MASK0

#define BIT_CBUS_CMD_ABORT						BIT_PAGE_5_CBUS_INT_1_MASK_CBUS_INT_1_MASK6
#define BIT_CBUS_MSC_ABORT_RCVD					BIT_PAGE_5_CBUS_INT_1_MASK_CBUS_INT_1_MASK3
#define BIT_CBUS_DDC_ABORT						BIT_PAGE_5_CBUS_INT_1_MASK_CBUS_INT_1_MASK2
#define BIT_CBUS_CEC_ABORT						BIT_PAGE_5_CBUS_INT_1_MASK_CBUS_INT_1_MASK1

#define BIT_INTR9_EDID_ERROR					BIT_PAGE_2_INTR9_INTR9_STAT6
#define BIT_INTR9_EDID_DONE						BIT_PAGE_2_INTR9_INTR9_STAT5
#define BIT_INTR9_DEVCAP_DONE					BIT_PAGE_2_INTR9_INTR9_STAT4
#define BIT_INTR9_EDID_ERROR_MASK				BIT_PAGE_2_INTR9_MASK_INTR9_MASK6
#define BIT_INTR9_EDID_DONE_MASK				BIT_PAGE_2_INTR9_MASK_INTR9_MASK5
#define BIT_INTR9_DEVCAP_DONE_MASK				BIT_PAGE_2_INTR9_MASK_INTR9_MASK4

#define BIT_TDM_INTR_SYNC_DATA					BIT_PAGE_1_TRXINTH_TRX_INTR8
#define BIT_TDM_INTR_SYNC_WAIT					BIT_PAGE_1_TRXINTH_TRX_INTR9

#define BIT_TDM_INTR_SYNC_DATA_MASK				BIT_PAGE_1_TRXINTMH_TRX_INTRMASK8
#define BIT_TDM_INTR_SYNC_WAIT_MASK				BIT_PAGE_1_TRXINTMH_TRX_INTRMASK9
#ifdef DO_HDCP
#define BIT_HDCP2_INTR_AUTH_DONE				BIT_PAGE_3_HDCP2X_INTR0_HDCP2X_INTR0_STAT0
#define BIT_HDCP2_INTR_AUTH_FAIL				BIT_PAGE_3_HDCP2X_INTR0_HDCP2X_INTR0_STAT1
#define BIT_HDCP2_INTR_RPTR_RCVID_CHANGE		BIT_PAGE_3_HDCP2X_INTR0_HDCP2X_INTR0_STAT4
#endif
#define REG_PAGE_3_MHL_COC_CTL3					TX_PAGE_3,0x45	/* default = 0 */
	#define BIT_PAGE_3_MHL_COC_CTL3_ECHO_CANCEL	0x01


#define REG_PAGE_3_MHL_DP_CTL6					TX_PAGE_3,0x50
#define REG_PAGE_3_MHL_DP_CTL7					TX_PAGE_3,0x51

#define REG_PAGE_7_COC_CTL17					TX_PAGE_7,0x2A	/* default = 0 */
#define REG_PAGE_7_COC_CTL18					TX_PAGE_7,0x2B	/* default = 0 */
#define REG_PAGE_7_COC_CTL19					TX_PAGE_7,0x2C	/* default = 0 */
#define REG_PAGE_7_COC_CTL1A					TX_PAGE_7,0x2D	/* default = 0 */

#define BIT_PAGE_2_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR	0x04
#define BIT_PAGE_2_TMDS_CSTAT_P3_AVIF_MANUAL_CLEAR_STROBE	0x08

#define BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT	BIT_PAGE_5_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT7
#define BIT_PAGE_CBUS_REG_MSC_MT_ABORT_INT_STAT5	BIT_PAGE_5_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT5
#define BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD		BIT_PAGE_5_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT3
#define BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT		BIT_PAGE_5_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT2
#define BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR		BIT_PAGE_5_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT1
#define BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL		BIT_PAGE_5_MSC_MT_ABORT_INT_MSC_MT_ABORT_INT_STAT0

#define BIT_PAGE_5_MDT_RCV_CONTROL_MDT_DELAY_RCV_EN 0x40
#define REG_080C_HDCP1X_LB_BIST					TX_PAGE_8,0x0C
#define BIT_080C_HDCP1X_LB_BIST 				0x01

#define	REG_CBUS_MHL_SCRPAD_BASE				0x40

#define REG_RX_HDMI_CTRL0_DEFVAL_DVI			0x14
#define REG_RX_HDMI_CTRL0_DEFVAL_HDMI			0x1C
#define REG_RX_HDMI_CTRL0_DEFVAL_HW_CTRL		0x10

#define	REG_RX_HDMI_CTRL2_DEFVAL_DVI			0x30
#define REG_RX_HDMI_CTRL2_DEFVAL_HDMI			0x38

#define TX_HW_RESET_PERIOD						10	/* 10 ms. */
#define TX_HW_RESET_DELAY						100

#define MHL_FLOW_FLAG_NONE						0x0000
#define MHL_FLOW_FLAG_MHL_IMPEDANCE				0x0001
#define MHL_FLOW_FLAG_MHL_ESTABLISHED			0x0002
#define MHL_FLOW_FLAG_DCAP_READY				0x0004
#define MHL_FLOW_FLAG_DCAP_CHANGE				0x0008
#define MHL_FLOW_FLAG_PATH_ENABLE				0x0010
#define MHL_FLOW_FLAG_RAP_CONTENT_ON			0x0020
#define MHL_FLOW_FLAG_CBUS_SET_HPD				0x0040
#define MHL_FLOW_FLAG_EDID_RTP					0x0080
#define MHL_FLOW_FLAG_INPUT_CLOCK_STABLE		0x0100
#define MHL_FLOW_FLAG_INFOFRAME_RECEIVED		0x0200
#define MHL_FLOW_FLAG_CP_AVAILABLE				0x0400
#define MHL_FLOW_FLAG_CP_AUTHENTICATED			0x0800


#define BIT_COC_PLL_LOCK_STATUS_CHANGE			0x01
#define BIT_COC_CALIBRATION_DONE				0x02
#define BIT_COC_8b_10b_ERROR					0x04
#define BIT_COC_CALIBRATION_COMMA1				0x08
#define BIT_COC_CALIBRATION_COMMA2				0x10

#define MSK_TDM_SYNCHRONIZED					0xC0
#define VAL_TDM_SYNCHRONIZED					0x80

#define BIT_COC_STAT_6_CALIBRATION_DONE			0x80
#define BITS_ES0_0_COC_STAT_F_CALIBRATION_STATE_2		0x02
#define BITS_ES1_0_COC_STAT_0_CALIBRATION_MASK			0x8F
#define BITS_ES1_0_COC_STAT_0_CALIBRATION_STATE_2		0x02
#define BITS_ES1_0_COC_STAT_0_PLL_LOCKED				0x80

#define VAL_PAGE_3_M3_CTRL_MHL3_VALUE (BIT_PAGE_3_M3_CTRL_SW_MHL3_SEL \
						|BIT_PAGE_3_M3_CTRL_M3AV_EN \
						|BIT_PAGE_3_M3_CTRL_ENC_TMDS \
						|BIT_PAGE_3_M3_CTRL_MHL3_MASTER_EN)

#define VAL_PAGE_3_M3_CTRL_PEER_VERSION_PENDING_VALUE VAL_PAGE_3_M3_CTRL_MHL3_VALUE

#define VAL_PAGE_3_M3_CTRL_MHL1_2_VALUE (BIT_PAGE_3_M3_CTRL_SW_MHL3_SEL \
						|BIT_PAGE_3_M3_CTRL_ENC_TMDS)

#define BIT_PAGE_3_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_PP         0x08
#define BIT_PAGE_3_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_NORM       0x00

static char * rgnd_value_string[] = {
	"open",
	"2k",
	"1k",
	"short"
};

#define IN_MHL3_MODE(hw_context) (hw_context->cbus_mode > CM_oCBUS_PEER_IS_MHL1_2)

static uint8_t colorSpaceTranslateInfoFrameToHw[] = {
	VAL_PAGE_6_INPUT_FORMAT_RGB,
	VAL_PAGE_6_INPUT_FORMAT_YCBCR422,
	VAL_PAGE_6_INPUT_FORMAT_YCBCR444,
	VAL_PAGE_6_INPUT_FORMAT_INTERNAL_RGB	/* reserved for future */
};

/*
	Based on module parameter "crystal_khz=xxxxx" program registers.
*/
static void program_ext_clock_regs(struct drv_hw_context *hw_context, int crystal_khz)
{
	/* preset to default crystal on SK - 19.2 MHz */
	int reg_fb_div_ctl_main = 0x04;
	int	reg_hdcp2x_tp1 = 0x5E;

	switch(crystal_khz) {
		case	38400:
			reg_fb_div_ctl_main = 0x0C;
			reg_hdcp2x_tp1 = 0xBC;
			break;
		case	30000:
			reg_fb_div_ctl_main = 0x06;
			reg_hdcp2x_tp1 = 0x92;
			break;
		case	24000:
			reg_fb_div_ctl_main = 0x05;
			reg_hdcp2x_tp1 = 0x75;
			break;
		case	20000:
			reg_fb_div_ctl_main = 0x04;
			reg_hdcp2x_tp1 = 0x62;
			break;
		case	19200:
		default:
/*			reg_fb_div_ctl_main = 0x04;
			reg_hdcp2x_tp1 = 0x5E;*/
			break;
	}
	mhl_tx_write_reg(hw_context, REG_PAGE_3_DIV_CTL_MAIN, reg_fb_div_ctl_main);	/* 3F2 default is 0x05 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_TP1, reg_hdcp2x_tp1);	/* 3B4 */
	MHL_TX_DBG_ERR("Set ext clock regs: 3F2 = %02X, 3B4 = %02X\n", reg_fb_div_ctl_main, reg_hdcp2x_tp1);
}

/*
	Per MHL specs heartbeat is required but disconnection may be handled using rxsense only.
*/
static void	enable_heartbeat(struct drv_hw_context *hw_context)
{
	/* Turn on heartbeat polling unconditionally to meet the specs */
	MHL_TX_DBG_WARN("Heartbeat polling is enabled\n");
	mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_HEARTBEAT_CONTROL, 0xA7);
	/*
	* turn on disconnection based on heartbeat (as well as RSEN) -
	* This is no longer the default behavior.
	* use_heartbeat=1 is required to enable disconnection.
	*/
	if (platform_get_flags() & PLATFORM_FLAG_USE_HEARTBEAT) {
		MHL_TX_DBG_ERR("Disconnection on heartbeat failure is enabled\n");
		mhl_tx_modify_reg(hw_context, REG_PAGE_5_DISC_CTRL1
									, BIT_PAGE_5_DISC_CTRL1_HB_EN
									, BIT_PAGE_5_DISC_CTRL1_HB_EN);
	} else {
		MHL_TX_DBG_ERR("Disconnection on heartbeat failure is disabled\n");
	}
}

/*
	For MHL 3 the auto_zone bits must be cleared.
	TODO: Check with characterization team if anything new shows up in their testing.
*/
static void clear_auto_zone_for_mhl_3(struct drv_hw_context *hw_context)
{
	/* Clear auto zone */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_TX_ZONE_CTL1, 0x0);

	/* Program PLL for 1X and clock from HSIC */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL0,
		(BIT_PAGE_3_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X
		| BIT_PAGE_3_MHL_PLL_CTL0_CRYSTAL_CLK_SEL
		| BIT_PAGE_3_MHL_PLL_CTL0_ZONE_MASK_OE));		/* 0x0337 <= 0x07 */
}

/*
	For MHL 1/2 we should use auto_zone.
*/
static void set_auto_zone_for_mhl_1_2(struct drv_hw_context *hw_context)
{
	/* Enable AUTO ZONE for MHL1/2 */
	mhl_tx_write_reg(hw_context,
		REG_PAGE_3_TX_ZONE_CTL1,
			MSK_PAGE_3_TX_ZONE_CTL1_TX_ZONE_CTRL_MODE);

	/* Program PLL for 1X and clock from HDMI */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL0,
		(BIT_PAGE_3_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X
		| BIT_PAGE_3_MHL_PLL_CTL0_ZONE_MASK_OE));		/* 0x0337 <= 0x05 */
}

int si_mhl_tx_drv_set_tdm_slot_allocation(struct drv_hw_context *hw_context,
										uint8_t *vc_slot_counts,
										bool resync)
{
	int			status = -EINVAL;
	uint16_t	slot_total = 0;
	uint8_t		idx;

	/* To the extent we can sanity check the slot allocation request */
	if (vc_slot_counts[VC_CBUS1] != 1)
		goto done;
	for (idx = 0; idx < VC_MAX; idx++)
		slot_total += vc_slot_counts[idx];

	if (slot_total > 200)
		goto done;

	if (!resync) {
		/*
		 * Since we're not being asked to perform TDM re-synchronization
		 * we don't know which eCBUS mode will be used with this slot
		 * allocation so we can't sanity check it further.  Just save
		 * it for later use.
		 */
		memcpy(hw_context->tdm_virt_chan_slot_counts, vc_slot_counts,
				sizeof(hw_context->tdm_virt_chan_slot_counts));
		status = 0;
	} else {
		switch (hw_context->cbus_mode) {
		case CM_eCBUS_S:
			if (slot_total > 25)
				goto done;
			break;

		case CM_eCBUS_D:
			break;

		default:
			goto done;
		}

		memcpy(hw_context->tdm_virt_chan_slot_counts, vc_slot_counts,
				sizeof(hw_context->tdm_virt_chan_slot_counts));
		status = 0;

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXSPINUMS,
				hw_context->tdm_virt_chan_slot_counts[VC_E_MSC]);
		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXHSICNUMS,
				hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS]);
		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXTOTNUMS, slot_total - 1);
	}

done:
	return status;
}

static int block_input_buffer_available(struct drv_hw_context *hw_context)
{
	uint16_t head,tail;
	head =hw_context->block_protocol.head;
	tail =hw_context->block_protocol.tail;
	if (head == (tail+1)) {
		/* full case */
		return 0;
	}
	return 1;
}

static int alloc_block_input_buffer(struct drv_hw_context *hw_context, uint8_t **pbuffer )
{
	uint16_t head,tail;
	int index;
	head =hw_context->block_protocol.head;
	tail =hw_context->block_protocol.tail;

	if ( ! block_input_buffer_available(hw_context)) {
		/* full case */
		return(-1);
	}
	index = tail;

	if (++tail >= NUM_BLOCK_INPUT_BUFFERS) {
		tail = 0;
	}
	hw_context->block_protocol.tail = tail;

	*pbuffer = &hw_context->block_protocol.input_buffers[index][0];
	return(index);
}
static void set_block_input_buffer_length(struct drv_hw_context *hw_context,
		int block, int length )
{
	if (( block < 0) || ( block >NUM_BLOCK_INPUT_BUFFERS ))
		return;

	hw_context->block_protocol.input_buffer_lengths[block] = length;
}

/* call this from mhl_supp.c to during processing of DRV_INTR_FLAG_EMSC_INCOMING */
int si_mhl_tx_drv_peek_block_input_buffer(struct mhl_dev_context *dev_context,
		uint8_t **buffer, int *length)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	uint16_t head,tail,index;
	head =hw_context->block_protocol.head;
	tail =hw_context->block_protocol.tail;
	if (head == tail) {
		return(-1);
	}
	index=head;

	*buffer = &hw_context->block_protocol.input_buffers[index][0];
	*length = hw_context->block_protocol.input_buffer_lengths[index];
	return(0);
}

/* call this from mhl_supp.c to during processing of DRV_INTR_FLAG_EMSC_INCOMING */
void si_mhl_tx_drv_free_block_input_buffer(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	uint16_t head;
	head =hw_context->block_protocol.head;
	if (++head >= NUM_BLOCK_INPUT_BUFFERS) {
		head = 0;
	}
	hw_context->block_protocol.head =head;
}

static int mhl3_block_isr(struct drv_hw_context *hw_context, uint8_t status)
{
	bool payload_encountered = false;

	if (BIT_PAGE_3_EMSCINTR_EMSC_XFIFO_EMPTY & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_EMSC_XMIT_ACK_TOUT & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_SPI_EMSC_READ_ERR & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_SPI_EMSC_WRITE_ERR & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_SPI_COMMA_CHAR_ERR & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_EMSC_XMITDONE & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_EMSC_XMIT_GNT_TOUT & status) {
	}
	if (BIT_PAGE_3_EMSCINTR_SPI_DVLD & status) {
		if (block_input_buffer_available(hw_context)) {
			int block_index;
			uint8_t rfifo_byte_count[2];
	#define EMSC_HEADER_SIZE (1 + STD_TRANSPORT_HDR_SIZE)
			uint16_t fifo_data_len, block_len;
			if (use_spi) {
				mhl_tx_read_reg_block_spi_emsc(hw_context
						,sizeof(rfifo_byte_count)
						,rfifo_byte_count);
			} else {
				mhl_tx_read_reg_block(hw_context,REG_PAGE_3_EMSCRFIFOBCNTL
						,sizeof(rfifo_byte_count)
						,rfifo_byte_count);
			}
			fifo_data_len = ((uint16_t)rfifo_byte_count[1] << 8) | (uint16_t)rfifo_byte_count[0];	//TODO: Lee - should this be validated against our actual FIFO size?
			MHL_TX_DBG_INFO("================= EMSCINTR_SPI_DVLD interrupt, payload length: %d\n", fifo_data_len );

			do {
				uint8_t *buffer = NULL;
				uint8_t header[EMSC_HEADER_SIZE];
				struct SI_PACK_THIS_STRUCT standard_transport_header_t *tport_hdr;

				if ( fifo_data_len < EMSC_HEADER_SIZE ) {
					MHL_TX_DBG_ERR("eMSC FIFO did not contain enough data for a EMSC_HEADER (%d < %d\n", fifo_data_len, EMSC_HEADER_SIZE );
					break;
				}
				if (use_spi) {
					mhl_tx_read_reg_block_spi_emsc(hw_context
									,EMSC_HEADER_SIZE
									,header);
				} else {
					mhl_tx_read_reg_block(hw_context
									,REG_PAGE_3_EMSC_RCV_READ_PORT
									,EMSC_HEADER_SIZE
									,header);
				}
				MHL_TX_DBG_INFO("================= EMSCINTR_SPI_DVLD interrupt, payload header: %02X %02X %02X\n",
						header[0], header[1], header[2]);
				fifo_data_len -= EMSC_HEADER_SIZE;
				tport_hdr = (struct SI_PACK_THIS_STRUCT standard_transport_header_t *)&header[1];

				MHL_TX_DBG_INFO("================= EMSCINTR_SPI_DVLD interrupt, tport_hdr->length_remaining: %d\n", tport_hdr->length_remaining);

				/* FIFO data count MUST be at LEAST enough for the current transport header remaining byte count. */
				if ( fifo_data_len < tport_hdr->length_remaining ) {
					MHL_TX_DBG_ERR("eMSC FIFO data count not enough for STD header remaining byte count: (%d < %d\n", fifo_data_len, tport_hdr->length_remaining );
					break;
				}

				block_len = 0;
				if ( tport_hdr->length_remaining > 0) {
					payload_encountered = true;
					block_index = alloc_block_input_buffer(hw_context, &buffer);
					block_len = tport_hdr->length_remaining;

					if (use_spi) {
						mhl_tx_read_reg_block_spi_emsc(hw_context,block_len,buffer);
					}else{
						mhl_tx_read_reg_block(hw_context
									,REG_PAGE_3_EMSC_RCV_READ_PORT
									,block_len
									,buffer);
					}
					set_block_input_buffer_length(hw_context, block_index, block_len);
					hw_context->block_protocol.received_byte_count += (block_len + STD_TRANSPORT_HDR_SIZE);
				}

				hw_context->block_protocol.peer_blk_rx_buffer_avail += tport_hdr->rx_unload_ack;
				MHL_TX_DBG_WARN("RD PEER Buffer Available: %d\n", hw_context->block_protocol.peer_blk_rx_buffer_avail );
				fifo_data_len -= block_len;
				if ( !block_input_buffer_available(hw_context)) {
					break;
				}
			}while (fifo_data_len > 0);
			if (payload_encountered)
				hw_context->intr_info->flags |=	DRV_INTR_FLAG_EMSC_INCOMING;
		}
	}
	return status;
}

static int coc_isr(struct drv_hw_context *hw_context,uint8_t coc_int_status)
{
	int ret_val = 0; /* Safe to clear interrupt from master handler */

	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
			struct mhl_dev_context, drv_context);
	MHL_TX_DBG_INFO("coc_isr, coc_int_status=0x%x,si_mhl_tx_drv_connection_is_mhl3(dev_context)=%d\n",coc_int_status,si_mhl_tx_drv_connection_is_mhl3(dev_context));
	
	if (BIT_COC_PLL_LOCK_STATUS_CHANGE & coc_int_status) {
		MHL_TX_DBG_INFO("COC PLL lock status change\n");
	}

	if (si_mhl_tx_drv_connection_is_mhl3(dev_context)) {

		if (BIT_COC_CALIBRATION_DONE & coc_int_status) {
			int calibration_stat,retry=0;
			uint8_t calibrated_value;
			MHL_TX_DBG_INFO("Calibration done\n");

			calibration_stat = mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_0);	// 0x0700
			calibration_stat &= BITS_ES1_0_COC_STAT_0_CALIBRATION_MASK;
			calibrated_value = BITS_ES1_0_COC_STAT_0_PLL_LOCKED|BITS_ES1_0_COC_STAT_0_CALIBRATION_STATE_2;

			if (calibrated_value == calibration_stat) {
				/* disable timeout */
				mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTLB,0x00);  /* 0x071B */
				MHL_TX_DBG_ERR("CoC in calibrated state\n");
				retry = 0;
				enable_intr(hw_context, INTR_TDM,
											BIT_TDM_INTR_SYNC_DATA_MASK |
											BIT_TDM_INTR_SYNC_WAIT_MASK);
				hw_context->cbus_mode = CM_TRANSITIONAL_TO_eCBUS_S_CALIBRATED;
				hw_context->intr_info->flags |= DRV_INTR_FLAG_COC_CAL;
			} else {
				MHL_TX_DBG_ERR("calibration state: 0x%02X\n", calibration_stat);
			}

			if (retry) {
				uint8_t coc_ctl0;
				MHL_TX_DBG_ERR("Retry calibration\n");

				coc_ctl0 = mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_CTL0);
				mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, coc_ctl0 & ~0x04);
				mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL14, 0x00);
				mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL15, 0x80);
				msleep(20);
				mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, coc_ctl0);
			}
		}
	}

	return ret_val;
}

static int tdm_isr(struct drv_hw_context *hw_context,uint8_t intr_status)
{
	int ret_val = 0; /* Safe to clear interrupt from master handler */
	uint8_t tdm_status;

	struct mhl_dev_context *dev_context;

	dev_context = container_of((void *)hw_context,
			struct mhl_dev_context, drv_context);

	if (si_mhl_tx_drv_connection_is_mhl3(dev_context)) {

		if (BIT_TDM_INTR_SYNC_DATA & intr_status) {

			MHL_TX_DBG_INFO("TDM in SYNC_DATA state.\n");
			tdm_status = mhl_tx_read_reg(hw_context, REG_PAGE_1_TRXSTA2);

			if ((tdm_status & MSK_TDM_SYNCHRONIZED) == VAL_TDM_SYNCHRONIZED) {

				MHL_TX_DBG_ERR("TDM is synchronized\n");

				if (CM_eCBUS_S > hw_context->cbus_mode) {
					hw_context->cbus_mode = CM_eCBUS_S;
					hw_context->intr_info->flags |= DRV_INTR_FLAG_TDM_SYNC;
					hw_context->block_protocol.received_byte_count = 0;

#ifdef EARLY_HSIC //(
					/*
					 * todo hsic_init configures the transmitter for USB host mode.  So
					 * really this call should be deferred until the driver has negotiated
					 * with the sink to take over the host role.  The call is placed here
					 * just for test purposes
					 */
					hsic_init(hw_context);
#endif //)
				}
			} else {
				MHL_TX_DBG_ERR("TDM not synchronized, retrying\n");
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL2, 0x00);	// 0x0339
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL2, 0x80);	// 0x0339
			}
		}

		if (BIT_TDM_INTR_SYNC_WAIT & intr_status) {
			MHL_TX_DBG_ERR("TDM in SYNC_WAIT state.\n");
		}
	}

	return ret_val;
}

static int int_link_trn_isr(struct drv_hw_context *hw_context,uint8_t intr_status)
{

	if (IN_MHL3_MODE(hw_context)) {
		MHL_TX_DBG_ERR("%sTODO: implement link training interrupt%s\n"
						,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
		return 0;
	} else {
		return 0;
	}
}

/*
 * Configure the CBUS link for the requested operating mode.
 * Returns:
 * -1 = Requested mode not available.
 *  0 = Requested mode change is complete.
 *  1 = Requested mode change is in process.  In this case completion
 *      is indicated by the interrupt flag DRV_INTR_FLAG_TDM_SYNC.
 */
int si_mhl_tx_drv_switch_cbus_mode(struct drv_hw_context *hw_context,
									enum cbus_mode_e mode_sel)
{
	int		status = 1;
	uint8_t	slot_total;

	MHL_TX_DBG_ERR("Switch cbus_mode from %x to %x\n", hw_context->cbus_mode, mode_sel);

	if (hw_context->cbus_mode < CM_oCBUS_PEER_VERSION_PENDING)
		return -1;


	switch (mode_sel) {
	case CM_oCBUS_PEER_IS_MHL1_2:
		MHL_TX_DBG_ERR("Switch to MHL1/2 oCBUS mode\n");
			#define  BIT_PAGE_5_CBUS_MSC_COMPATIBILITY_CONTROL_ENABLE_XDEVCAP	0x80
				mhl_tx_write_reg(hw_context
					,REG_PAGE_5_CBUS_MSC_COMPATIBILITY_CONTROL
					,0x02);
				mhl_tx_write_reg(hw_context,REG_PAGE_3_M3_CTRL
					,VAL_PAGE_3_M3_CTRL_MHL1_2_VALUE);
				/*
				 * disable  BIT_PAGE_0_DPD_PWRON_HSIC
				 */
				mhl_tx_write_reg(hw_context, REG_PAGE_0_DPD
								, BIT_PAGE_0_DPD_PWRON_PLL
								| BIT_PAGE_0_DPD_PDNTX12
								| BIT_PAGE_0_DPD_OSC_EN
								);
				enable_intr(hw_context, INTR_COC, 0);
				set_auto_zone_for_mhl_1_2(hw_context);			/* Programs 0x0337 */

				/* Video failed with C0 - when Phalanx->Titan->Jubilee was used for HDCP CTS 1.x */
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xBC);		/* 0x0331 this worked with Jubilee */
				//mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xC0);		/* 0x0331 */

				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL1 ,0xBB);		/* 0x0332 */
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL3 ,0x48);		/* 0x0334 */
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5 ,0x3F);		/* 0x0336 */
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2 ,0x2F);		/* 0x0333 */
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL6 ,0x2A);		/* 0x0350 */
				mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL7 ,0x04);		/* 0x0351 */
				hw_context->cbus_mode=CM_oCBUS_PEER_IS_MHL1_2;
				
		break;
	case CM_oCBUS_PEER_IS_MHL3:

		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, 0x40);		// 0x0710
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL1, 0x07);	// 0x0343
		hw_context->cbus_mode = CM_oCBUS_PEER_IS_MHL3;
		
		break;

	case CM_eCBUS_S:
		/* Disable h/w automation of WRITE_BURST until this command completes */
		disable_gen2_write_burst_rcv(hw_context);
		disable_gen2_write_burst_xmit(hw_context);

		MHL_TX_DBG_WARN("hpd status: 0x%02x\n",mhl_tx_read_reg(hw_context,REG_PAGE_5_CBUS_STATUS));
		hw_context->cbus_mode = CM_TRANSITIONAL_TO_eCBUS_S;

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXSPINUMS,
				hw_context->tdm_virt_chan_slot_counts[VC_E_MSC]);
		slot_total = hw_context->tdm_virt_chan_slot_counts[VC_E_MSC];
		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXHSICNUMS,
				hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS]);
		slot_total += hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS];
		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXTOTNUMS, slot_total);

		/* begin reset */
		mhl_tx_write_reg(hw_context, REG_PAGE_0_PWD_SRST, 0xA0);		/* 0x000E */

		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL5, 0xF9);	/* 0x0347 */
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL3, BIT_PAGE_3_MHL_COC_CTL3_ECHO_CANCEL); /* 0x0345 */
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1, 0x10);		// 0x0711
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL2, 0x18);		// 0x0712
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL17,0x61);		// 0x072A
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL18   ,0x46);		// 0x072B
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL19   ,0x15);		// 0x072C
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1A   ,0x01);		// 0x072D
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7    ,0x06);		// 0x0717
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL11   ,0xF8);		// 0x0721
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL1, 0xBD);	// 0x0343  2013-11-15 17:07 changed from 0xBA
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL4,0x2D);		// 0x0346

		/* release sw-reset */
		mhl_tx_write_reg(hw_context, REG_PAGE_0_PWD_SRST, 0x20);		// release HDCP reset later

		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLF, 0x07);		// 0x071F

		/* Enable timeout */
		mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTLB,0x01);  /* 0x071B */

		mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL14, 0x03); //TRAP 0X03 0x0724
		mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL15, 0x80); //PULSE FOR TRAP 0x0725

		msleep(50);

		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, 0x5C);		// 0x0710

		mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL14, 0x00); //TRAP 0X00 0x0724
		mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL15, 0x80); //PULSE FOR TRAP 0x0725

		mhl_tx_write_reg(hw_context, REG_PAGE_1_HRXCTRL3, 0x07);		// 0x0104
		mhl_tx_write_reg(hw_context, REG_PAGE_5_CBUS3_CNVT, 0x85);		// 0x05DC

		mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR13, 0xFC);	 // 0191
		mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR14, 0xFF);	 // 0192
		mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR15, 0xFF);  // 0193
		mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR50, 0x03);  // 01B6
		
		break;

	case CM_eCBUS_D:
		hw_context->cbus_mode = CM_TRANSITIONAL_TO_eCBUS_D;

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXSPINUMS,
				hw_context->tdm_virt_chan_slot_counts[VC_E_MSC]);
		slot_total = hw_context->tdm_virt_chan_slot_counts[VC_E_MSC];
		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXHSICNUMS,
				hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS]);
		slot_total += hw_context->tdm_virt_chan_slot_counts[VC_T_CBUS];
		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXTOTNUMS, slot_total);
		
		break;

	default:
		MHL_TX_DBG_ERR("Invalid or unsupported CBUS mode specified\n");
		status = -EINVAL;
		break;
	}
	return status;
}

enum cbus_mode_e si_mhl_tx_drv_get_cbus_mode(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	return hw_context->cbus_mode;
}

char * si_mhl_tx_drv_get_cbus_mode_str(enum cbus_mode_e cbus_mode)
{
	char *cbus_mode_str;

	switch (cbus_mode) {
	case CM_NO_CONNECTION:
		cbus_mode_str="not connected";
		break;
	case CM_oCBUS_PEER_VERSION_PENDING:
		cbus_mode_str="oCBUS (MHL?)";
		break;
	case CM_oCBUS_PEER_IS_MHL1_2:
		cbus_mode_str="oCBUS (MHL1/2)";
		break;
	case CM_oCBUS_PEER_IS_MHL3:
		cbus_mode_str="oCBUS (MHL3)";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_S:
		cbus_mode_str="-> to eCBUS_S";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_S_CALIBRATED:
		cbus_mode_str="-> to eCBUS_S (cal)";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_D:
		cbus_mode_str="-> to eCBUS_D";
		break;
	case CM_TRANSITIONAL_TO_eCBUS_D_CALIBRATED:
		cbus_mode_str="-> to eCBUS_D (cal)";
		break;
	case CM_eCBUS_S:
		cbus_mode_str="eCBUS_S";
		break;
	case CM_eCBUS_D:
		cbus_mode_str="eCBUS_D";
		break;
	default:
		cbus_mode_str="invalid";
		break;
	}

	return cbus_mode_str;
}

static void disable_gen2_write_burst_rcv(struct drv_hw_context *hw_context)
{
	if (hw_context->gen2_write_burst_rcv) {
		/* disable Gen2 Write Burst engine to allow normal CBUS traffic */
		mhl_tx_write_reg(hw_context,REG_PAGE_5_MDT_XMIT_CONTROL, 0);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_RCV_CONTROL, 0);
//						BIT_PAGE_5_MDT_RCV_CONTROL_MDT_RCV_EN);
//						BIT_PAGE_5_MDT_RCV_CONTROL_MDT_DISABLE); // TODO: Review with John
		MHL_TX_DBG_INFO("%sdisabled GEN2%s\n"
						,ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_rcv = false;
	}
}

static void disable_gen2_write_burst_xmit(struct drv_hw_context *hw_context)
{
	if (hw_context->gen2_write_burst_xmit) {
		/* disable Gen2 Write Burst engine to allow normal CBUS traffic */
		mhl_tx_write_reg(hw_context,REG_PAGE_5_MDT_XMIT_CONTROL, 0);
		MHL_TX_DBG_INFO("%sdisabled GEN2%s\n"
						,ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_xmit = false;
	}
}

static void enable_gen2_write_burst_rcv(struct drv_hw_context *hw_context)
{
	/* enable Gen2 Write Burst interrupt, MSC and EDID interrupts. */
	if ( ! hw_context->gen2_write_burst_rcv) {

		mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_RCV_TIMEOUT, 100);	/* 2 second timeout */
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_RCV_CONTROL
				, BIT_PAGE_5_MDT_RCV_CONTROL_MDT_RCV_EN
				| hw_context->delayed_hawb_enable_reg_val);

		MHL_TX_DBG_INFO("%senabled GEN2%s\n"
						,ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_rcv = true;
	}
}

static void enable_gen2_write_burst_xmit(struct drv_hw_context *hw_context)
{
	/* enable Gen2 Write Burst interrupt, MSC and EDID interrupts. */
	if ( ! hw_context->gen2_write_burst_xmit) {

		mhl_tx_write_reg(hw_context,REG_PAGE_5_MDT_XMIT_CONTROL
				,BIT_PAGE_5_MDT_XMIT_CONTROL_MDT_XMIT_EN
			//	|BIT_PAGE_5_MDT_XMIT_CONTROL_MDT_XMIT_CMD_MERGE_EN
				|BIT_PAGE_5_MDT_XMIT_CONTROL_MDT_XMIT_FIXED_BURST_LEN
				);

		MHL_TX_DBG_INFO("%senabled GEN2%s\n"
						,ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
		hw_context->gen2_write_burst_xmit = true;
	}
}

bool si_mhl_tx_drv_issue_edid_read_request(struct drv_hw_context *hw_context,
							uint8_t block_number)
{
	uint8_t reg_val;
	reg_val = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
	if (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & reg_val) {
		MHL_TX_DBG_INFO(
					"\n\tRequesting EDID block:%d\n"	\
					"\tcurrentEdidRequestBlock:%d\n"	\
					"\tedidFifoBlockNumber:%d\n",
					block_number,
					hw_context->current_edid_request_block,
					hw_context->edid_fifo_block_number);
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_MANUAL,0x00);
		mhl_tx_write_reg(hw_context,REG_080C_HDCP1X_LB_BIST,0x00);

		/* Setup auto increment and kick off read */
		mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_CTRL
				, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
				| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID
				| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
				| ((block_number & 0x01) << 2)
				| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
				);

#ifndef MANUAL_EDID_FETCH //(
#define SWWA_BZ30759
#endif //)
#ifdef SWWA_BZ30759 //(
		mhl_tx_write_reg(hw_context,REG_PAGE_5_DISC_STAT1,0x08);
		mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL5
				,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE
				,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE);
#endif //)
		/* Setup which block to read */
		if (0 == block_number) {
			/* Enable EDID interrupt */
			enable_intr(hw_context, INTR_EDID,
						   ( BIT_INTR9_DEVCAP_DONE_MASK
							| BIT_INTR9_EDID_DONE_MASK
							| BIT_INTR9_EDID_ERROR
						   ));
			mhl_tx_write_reg(hw_context, REG_PAGE_2_TPI_CBUS_START,
							 BIT_PAGE_2_TPI_CBUS_START_GET_EDID_START_0);
		} else {
			uint8_t param = (1 << (block_number-1));
			MHL_TX_DBG_INFO("EDID HW Assist: Programming "
			 				"Reg %02X:%02X to %02X\n",
			 				REG_PAGE_2_EDID_START_EXT, param);
			mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_START_EXT,
							 param);
		}

		return true;
	} else {
		MHL_TX_DBG_INFO("\n\tNo HPD for EDID block request:%d\n"	\
						"\tcurrentEdidRequestBlock:%d\n"			\
						"\tedidFifoBlockNumber:%d\n",
						block_number,
						hw_context->current_edid_request_block,
						hw_context->edid_fifo_block_number);
		return false;
	}
}

/*
 * si_mhl_tx_drv_send_block
 *
 * Write the specified Sideband Channel command to the CBUS.
 * such as READ_DEVCAP, SET_INT, WRITE_STAT, etc.
 * Command can be a MSC_MSG command (RCP/RAP/RCPK/RCPE/RAPK), or another command
 * Parameters:
 *              req     - Pointer to a cbus_req_t structure containing the
 *                        command to write
 * Returns:     true    - successful write
 *              false   - write failed
 */
void mhl_tx_drv_send_block(struct drv_hw_context * hw_context, struct block_req *req)
{
	uint8_t count;

	count = req->sub_payload_size;
	MHL_TX_DBG_INFO( " req->sub_payload_size: %d req->count: %d\n", count, req->count );

	MHL_TX_DBG_WARN("total bytes to ack: %d\n",hw_context->block_protocol.received_byte_count);
	if (hw_context->block_protocol.received_byte_count >= 256) {

		/* 'can't represent numbers >= 256 in 8 bits,so ack as much as we can ...*/
		req->payload->hdr_and_burst_id.tport_hdr.rx_unload_ack =  255;
		hw_context->block_protocol.received_byte_count -=255;
	} else {
		req->payload->hdr_and_burst_id.tport_hdr.rx_unload_ack
			= (uint8_t)hw_context->block_protocol.received_byte_count;
		hw_context->block_protocol.received_byte_count = 0;
	}

	dump_array( "Payload bytes", &req->payload->as_bytes[0], req->count );
	MHL_TX_DBG_WARN("rx_unload_ack: %d\n",req->payload->hdr_and_burst_id.tport_hdr.rx_unload_ack);
	if (use_spi) {
		mhl_tx_write_emsc_block_spi(hw_context,req);
	} else {
		// Enable I2C communication with EMSC instead of SPI
		mhl_tx_modify_reg(hw_context,REG_PAGE_3_COMMECNT
				, BIT_PAGE_3_COMMECNT_I2C_TO_EMSC_EN
				, BIT_PAGE_3_COMMECNT_I2C_TO_EMSC_EN);
		// eMSC Transmitter Write Port for I2C
		mhl_tx_write_reg_block(hw_context
							,REG_PAGE_3_EMSC_XMIT_WRITE_PORT
							, req->count
							, &req->payload->as_bytes[0]
							);
	}
}
/*
	pending hawb write burst status
*/
uint8_t si_mhl_tx_drv_get_pending_hawb_write_status(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	return hw_context->hawb_write_pending;
}

/*
si_mhl_tx_drv_hawb_xfifo_avail
*/

uint8_t si_mhl_tx_drv_hawb_xfifo_avail(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	return MSK_PAGE_5_MDT_XFIFO_STAT_MDT_XFIFO_LEVEL_AVAIL
					& mhl_tx_read_reg(hw_context,REG_PAGE_5_MDT_XFIFO_STAT);
}
#ifdef MANUAL_EDID_FETCH //(
static uint8_t fetch_edid_block(struct drv_hw_context *hw_context,uint8_t *buffer,uint8_t block_num,bool trigger_on_last)
{
	int lm_ddc,ddc_cmd,ddc_status,ddc_address,ddc_limit,step,dout_cnt,intr3_status,cbus_status;
	lm_ddc = mhl_tx_read_reg(hw_context,REG_PAGE_0_LM_DDC);
	ddc_cmd = mhl_tx_read_reg(hw_context,REG_PAGE_0_DDC_CMD);
	ddc_cmd &= ~MSK_PAGE_0_DDC_CMD_DDC_CMD;
	/* Disable EDID interrupt */
	enable_intr(hw_context, INTR_EDID, 0);

	/* disable auto edid function */
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_CTRL
			, VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			);
	// Disable HDCP2 DDC polling
	mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_POLL_CS, 0x71);		// 0x0391

	/* disable encryption */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_CTRL_0, 0x02);	// 0x03A0

	/* disable TPI mode */
	mhl_tx_write_reg(hw_context, REG_PAGE_0_LM_DDC
				,lm_ddc | VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED);

	for (step=0;step<256;++step){
		ddc_status = mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);
		if (0==(BIT_PAGE_0_DDC_STATUS_DDC_I2C_IN_PROG & ddc_status)){
			break;
		}
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_STATUS,BIT_PAGE_0_DDC_STATUS_DDC_FIFO_EMPTY);
	}
	/* set DDC slave address to EDID */

	mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_ADDR,0xA0);
	step = 16;
	ddc_limit = (block_num << 7)+EDID_BLOCK_SIZE;
	for (ddc_address=block_num<<7;ddc_address < ddc_limit ;ddc_address+=step){
		ddc_status = mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);
		mhl_tx_write_reg(hw_context, REG_PAGE_0_DDC_CMD,ddc_cmd | VAL_PAGE_0_DDC_CMD_DDC_CMD_ABORT);
		mhl_tx_write_reg(hw_context, REG_PAGE_0_DDC_CMD, ddc_cmd | VAL_PAGE_0_DDC_CMD_DDC_CMD_CLEAR_FIFO);
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_STATUS,BIT_PAGE_0_DDC_STATUS_DDC_FIFO_EMPTY);

		/* synchronize by making sure that any stale interrupt is cleared */
		intr3_status = mhl_tx_read_reg(hw_context,REG_PAGE_0_INTR3);
		mhl_tx_write_reg(hw_context,REG_PAGE_0_INTR3,intr3_status);

		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_SEGM,HIGH_BYTE_16(ddc_address));
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_OFFSET,LOW_BYTE_16(ddc_address));
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_DIN_CNT1,LOW_BYTE_16(step));
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_DIN_CNT2,HIGH_BYTE_16(step));

		mhl_tx_write_reg(hw_context, REG_PAGE_0_DDC_CMD, ddc_cmd | VAL_PAGE_0_DDC_CMD_ENH_DDC_READ_NO_ACK);


		do{
			intr3_status = mhl_tx_read_reg(hw_context,REG_PAGE_0_INTR3);
			cbus_status = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
			cbus_status &= (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD | BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED);
			if (BIT_PAGE_0_INTR3_DDC_CMD_DONE & intr3_status)
				break;
		}while (cbus_status == (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD | BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED));
		if (cbus_status !=(BIT_PAGE_5_CBUS_STATUS_CBUS_HPD | BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED)){
			break;
		}

		ddc_status	= mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);
		dout_cnt 	= mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_DOUT_CNT);

		mhl_tx_read_reg_block(hw_context, REG_PAGE_0_DDC_DATA, step, &buffer[ddc_address%EDID_BLOCK_SIZE]);

		if (0 == ddc_address){
			PEDID_block0_t p_EDID_block_0 = (PEDID_block0_t)buffer;
			if (! si_mhl_tx_check_edid_header(hw_context->intr_info->edid_parser_context, p_EDID_block_0) ){
				/* back-up by one step  to retry */
				ddc_address-=step;
			}
		}

		if ((ddc_address + step) >= ddc_limit){

			/* make sure that done is triggered for sinks with only 1 EDID block (DVI) */
			if (0 == block_num){
				PEDID_block0_t p_EDID_block_0 = (PEDID_block0_t)buffer;
				if (0 == p_EDID_block_0->extension_flag){
					trigger_on_last = true;
				}
			}
			if (trigger_on_last){
				enable_intr(hw_context,INTR_DDC,BIT_PAGE_0_INTR3_DDC_CMD_DONE);
				/* let int_3_isr or si_mhl_tx_drv_device_isr clear the interrupt */
			}else{
				mhl_tx_write_reg(hw_context,REG_PAGE_0_INTR3,intr3_status);
			}
		}else{
			mhl_tx_write_reg(hw_context,REG_PAGE_0_INTR3,intr3_status);
		}
	}

	/* restore TPI mode state */
	mhl_tx_write_reg(hw_context, REG_PAGE_0_LM_DDC,lm_ddc);
	return cbus_status;
}
#endif //)
/*
 * si_mhl_tx_drv_send_cbus_command
 *
 * Write the specified Sideband Channel command to the CBUS.
 * such as READ_DEVCAP, SET_INT, WRITE_STAT, etc.
 * Command can be a MSC_MSG command (RCP/RAP/RCPK/RCPE/RAPK), or another command
 * Parameters:
 *              req     - Pointer to a cbus_req_t structure containing the
 *                        command to write
 * Returns:		for WRITE_BURST, if an MDT XFIFO level is available, it returns non-zero, otherwise zero.
 *				for MHL_READ_EDID_BLOCK  it returns either the command type, or 0 if downstream HPD is low.
 *				for all other commands, the return value is the command type.
 *
 */
uint8_t si_mhl_tx_drv_send_cbus_command(struct drv_hw_context *hw_context,
							struct cbus_req *req)
{
	uint8_t ret_val = req->command;
	uint8_t	block_write_buffer [3];	// used for efficient block writes
//#define AGGRESSIVE_HAWB_RCV

	#ifdef AGGRESSIVE_HAWB_RCV //(
	if (MHL_WRITE_BURST != req->command) {
		/* Disable h/w automation of WRITE_BURST until this command completes */
		disable_gen2_write_burst_rcv(hw_context);
		if (MHL_SET_INT != req->command) {
			/*
			Do a complete reset of HAWB
			*/
			mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_RCV_CONTROL,
						BIT_PAGE_5_MDT_RCV_CONTROL_MDT_DISABLE);
			/* Enable h/w automation of WRITE_BURST receive*/
			hw_context->delayed_hawb_enable_reg_val = BIT_PAGE_5_MDT_RCV_CONTROL_MDT_DELAY_RCV_EN;
			enable_gen2_write_burst_rcv(hw_context);
			hw_context->cbus1_state=cbus1_msc_pending_delayed_rcv_enable;
		}
	}
	#else //)(
	switch (req->command) {
	case MHL_WRITE_BURST:
		break;
	default:
		/* Disable h/w automation of WRITE_BURST until this command completes */
		disable_gen2_write_burst_rcv(hw_context);
		disable_gen2_write_burst_xmit(hw_context);	// TODO: Review with John
	}
	#endif //)

	hw_context->current_cbus_req=*req;
	switch (req->command) {
	case MHL_SEND_3D_REQ_OR_FEAT_REQ:
		/* DO NOT RE-ORDER THIS CASE */
	#ifndef AGGRESSIVE_HAWB_RCV //(
		/*
		Do a complete reset of HAWB
		*/
/*		mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_RCV_CONTROL,
					BIT_PAGE_5_MDT_RCV_CONTROL_MDT_DISABLE); */		// TODO: Review with John
		// Insert disable of both rcv and xmit engine
		/* Enable h/w automation of WRITE_BURST receive*/
		hw_context->delayed_hawb_enable_reg_val = BIT_PAGE_5_MDT_RCV_CONTROL_MDT_DELAY_RCV_EN;
		enable_gen2_write_burst_rcv(hw_context);
		hw_context->cbus1_state=cbus1_msc_pending_delayed_rcv_enable;
	#endif //)
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_COMMAND_START,
						 BIT_PAGE_5_MSC_COMMAND_START_MSC_WRITE_STAT_CMD);
		break;

	case MHL_SET_INT:
	case MHL_WRITE_STAT:
	case MHL_WRITE_XSTAT:
		MHL_TX_DBG_INFO("->reg: 0x%02x data: 0x%02x\n",
						req->reg, req->reg_data);
#ifdef WRITE_STAT_SET_INT_COALESCE //(
		mhl_tx_write_reg_block(hw_context,REG_PAGE_5_MSC_CMD_OR_OFFSET,2,&req->reg);
#else //)(
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_1ST_TRANSMIT_DATA, req->reg_data);
#endif //)
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_COMMAND_START,
						 BIT_PAGE_5_MSC_COMMAND_START_MSC_WRITE_STAT_CMD);
		if (MHL_RCHANGE_INT==hw_context->current_cbus_req.reg) {

			if (MHL2_INT_3D_REQ & hw_context->current_cbus_req.reg_data) {
				MHL_TX_DBG_WARN("3D_REQ sent\n");
			}

			if (MHL3_INT_FEAT_REQ & hw_context->current_cbus_req.reg_data) {
				MHL_TX_DBG_WARN("FEAT_REQ sent\n");
			}

			if (MHL_INT_GRT_WRT & hw_context->current_cbus_req.reg_data) {
				MHL_TX_DBG_WARN("GRT_WRT sent\n");
			}
		}
		break;

	case MHL_READ_DEVCAP:
		MHL_TX_DBG_ERR("Read DEVCAP\n");
		/* Enable DEVCAP_DONE interrupt */
		enable_intr(hw_context, INTR_EDID, BIT_INTR9_DEVCAP_DONE);

		mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_CTRL
					,VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
					|VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_DEVCAP
					|VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
					|VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
					);

		/* read the entire DEVCAP array in one command */
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TPI_CBUS_START,
					BIT_PAGE_2_TPI_CBUS_START_GET_DEVCAP_START);
		break;
	case MHL_READ_DEVCAP_REG:
		MHL_TX_DBG_INFO("Trigger DEVCAP_REG Read\n");
		MHL_TX_DBG_INFO("Read DEVCAP (0x%02x)\n",req->reg);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_CMD_OR_OFFSET, req->reg_data);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_COMMAND_START,
					BIT_PAGE_5_MSC_COMMAND_START_MSC_READ_DEVCAP_CMD);
		break;

	case MHL_READ_XDEVCAP:
		MHL_TX_DBG_INFO("Trigger XDEVCAP Read\n");
		/* Enable DEVCAP_DONE interrupt */
		enable_intr(hw_context, INTR_EDID, BIT_INTR9_DEVCAP_DONE);

		mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_CTRL
					,VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
					|BIT_PAGE_2_EDID_CTRL_XDEVCAP_EN
					|VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_DEVCAP
					|VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
					|VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
					);
		/* read the entire DEVCAP array in one command */
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TPI_CBUS_START,
					BIT_PAGE_2_TPI_CBUS_START_GET_DEVCAP_START);
		break;
	case MHL_READ_XDEVCAP_REG:
		MHL_TX_DBG_INFO("Read XDEVCAP_REG (0x%02x)\n",req->reg);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_CMD_OR_OFFSET, req->reg);
		mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_COMMAND_START,
					BIT_PAGE_5_MSC_COMMAND_START_MSC_READ_DEVCAP_CMD);
		break;

	case MHL_READ_EDID_BLOCK:
		hw_context->current_edid_request_block = 0;
		{
		int success;

#ifdef MANUAL_EDID_FETCH //(
			uint8_t cbus_status;
			success = 1;
			cbus_status = fetch_edid_block(hw_context,hw_context->edid_block
										,hw_context->current_edid_request_block,false);
			if (cbus_status ==(BIT_PAGE_5_CBUS_STATUS_CBUS_HPD | BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED)){
				int num_extensions;

				num_extensions =si_mhl_tx_get_num_cea_861_extensions(
								hw_context->intr_info->edid_parser_context,
								hw_context->current_edid_request_block);
				if (num_extensions < 0){
					success = 0;
				} else {
					for (hw_context->current_edid_request_block=1
							;hw_context->current_edid_request_block <= num_extensions
								;hw_context->current_edid_request_block++) {
						cbus_status = fetch_edid_block(hw_context,hw_context->edid_block
							,hw_context->current_edid_request_block
							,(hw_context->current_edid_request_block == num_extensions)
								 ?true:false);
						if (cbus_status !=
							(BIT_PAGE_5_CBUS_STATUS_CBUS_HPD
							| BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED)){
							success = 0;
							break;
						}
						num_extensions =si_mhl_tx_get_num_cea_861_extensions(
								hw_context->intr_info->edid_parser_context,
								hw_context->current_edid_request_block);
						if (num_extensions < 0)	{
							MHL_TX_DBG_ERR("edid problem:%d\n",num_extensions);
							success = 0;
							break;
						}
					}
				}
			}

#else //(
			success = si_mhl_tx_drv_issue_edid_read_request(hw_context,
					hw_context->current_edid_request_block);
#endif //)
			ret_val=success?ret_val:0;
		}
		break;

	case MHL_GET_STATE:			/* 0x62 - */
	case MHL_GET_VENDOR_ID:		/* 0x63 - for vendor id */
	case MHL_SET_HPD:			/* 0x64 - Set Hot Plug Detect */
	case MHL_CLR_HPD:			/* 0x65 - Clear Hot Plug Detect */
	case MHL_GET_SC1_ERRORCODE:	/* 0x69 - Get channel 1 command error code */
	case MHL_GET_DDC_ERRORCODE:	/* 0x6A - Get DDC channel command error code */
	case MHL_GET_MSC_ERRORCODE:	/* 0x6B - Get MSC command error code */
	case MHL_GET_SC3_ERRORCODE:	/* 0x6D - Get channel 3 command error code */
		MHL_TX_DBG_INFO("Sending MSC command %02x, %02x, %02x\n",
						req->command, req->reg, req->reg_data);
		mhl_tx_write_reg(hw_context,REG_PAGE_5_MSC_CMD_OR_OFFSET, req->command);
		mhl_tx_write_reg(hw_context,REG_PAGE_5_MSC_1ST_TRANSMIT_DATA, req->reg_data);
		mhl_tx_write_reg(hw_context,REG_PAGE_5_MSC_COMMAND_START,
						 BIT_PAGE_5_MSC_COMMAND_START_MSC_PEER_CMD);
		break;

	case MHL_MSC_MSG:
		MHL_TX_DBG_INFO("MHL_MSC_MSG sub cmd: 0x%02x data: 0x%02x\n",
					req->msg_data[0], req->msg_data[1]);
		block_write_buffer[0] = req->command;
		block_write_buffer[1] = req->msg_data[0];
		block_write_buffer[2] = req->msg_data[1];

		mhl_tx_write_reg_block(hw_context,REG_PAGE_5_MSC_CMD_OR_OFFSET, 3, block_write_buffer);
		mhl_tx_write_reg(hw_context,REG_PAGE_5_MSC_COMMAND_START,
						 BIT_PAGE_5_MSC_COMMAND_START_MSC_MSC_MSG_CMD);
		break;

	case MHL_WRITE_BURST:
		MHL_TX_DBG_INFO("MHL_WRITE_BURST offset: 0x%02x length: 0x%02x\n",
											req->burst_offset, req->length);
		hw_context->hawb_write_pending = true;
		enable_gen2_write_burst_xmit(hw_context);
		mhl_tx_write_reg_block(hw_context,REG_PAGE_5_MDT_XMIT_WRITE_PORT
						, req->length, req->msg_data);
		ret_val = (MSK_PAGE_5_MDT_XFIFO_STAT_MDT_XFIFO_LEVEL_AVAIL
					& mhl_tx_read_reg(hw_context,REG_PAGE_5_MDT_XFIFO_STAT)
					);
		break;

	default:
		MHL_TX_DBG_ERR("Unsupported command 0x%02x detected!\n", req->command);
		ret_val = 0;
		break;
	}

	return ret_val;
}

void si_mhl_tx_drv_set_3d_mode(struct drv_hw_context *hw_context, bool do_3D,
							_3D_structure_e three_d_mode)
{
	if (do_3D) {
		if (tdsFramePacking == three_d_mode) {
			MHL_TX_DBG_INFO("using frame packing\n");

			mhl_tx_write_reg(hw_context, REG_PAGE_0_VID_OVRRD,
					BIT_PAGE_0_VID_OVRRD_PP_AUTO_DISABLE |
					VAL_PAGE_0_VID_OVRRD_3DCONV_EN_FRAME_PACK);
		} else {
			MHL_TX_DBG_INFO("NOT using frame packing\n");
			mhl_tx_write_reg(hw_context, REG_PAGE_0_VID_OVRRD, BIT_PAGE_0_VID_OVRRD_PP_AUTO_DISABLE);
		}
	} else {
		MHL_TX_DBG_INFO("NOT using frame packing\n");
		mhl_tx_write_reg(hw_context, REG_PAGE_0_VID_OVRRD , BIT_PAGE_0_VID_OVRRD_PP_AUTO_DISABLE);
	}
}

uint16_t si_mhl_tx_drv_get_incoming_horizontal_total(struct drv_hw_context *hw_context)
{
	uint16_t ret_val;

	ret_val = (((uint16_t)mhl_tx_read_reg(hw_context, REG_PAGE_0_H_RESH)) <<8) |
				(uint16_t)mhl_tx_read_reg(hw_context, REG_PAGE_0_H_RESL);
	return ret_val;
}

uint16_t si_mhl_tx_drv_get_incoming_vertical_total(struct drv_hw_context *hw_context)
{
	uint16_t ret_val;

	ret_val = (((uint16_t)mhl_tx_read_reg(hw_context, REG_PAGE_0_V_RESH)) <<8) |
				(uint16_t)mhl_tx_read_reg(hw_context, REG_PAGE_0_V_RESL);
	return ret_val;
}

int si_mhl_tx_drv_get_aksv(struct drv_hw_context *hw_context,
						uint8_t *buffer)
{
	memcpy(buffer, hw_context->aksv, 5);
	return 0;
}
void si_mhl_tx_drv_skip_to_next_edid_block(struct drv_hw_context *hw_context)
{
	hw_context->edid_fifo_block_number++;
}

int si_mhl_tx_drv_get_edid_fifo_partial_block(struct drv_hw_context *hw_context
										,uint8_t start
										,uint8_t length
										,uint8_t *edid_buf)
{
	int ret_val;
	uint8_t offset;

	offset = EDID_BLOCK_SIZE * (hw_context->edid_fifo_block_number & 0x01);
	offset += start;

	MHL_TX_DBG_INFO("%x %x\n",(unsigned int)hw_context,(unsigned int)edid_buf);
	if (EDID_BLOCK_SIZE==(offset + length)) {
		hw_context->edid_fifo_block_number++;
	}

#ifdef MANUAL_EDID_FETCH //(
	memcpy(edid_buf,&hw_context->edid_block[start],length);
#else //)(
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_FIFO_ADDR, offset);

	ret_val = mhl_tx_read_reg_block(hw_context
		, REG_PAGE_2_EDID_FIFO_RD_DATA
		, length
		, edid_buf);
#endif //)

	DUMP_EDID_BLOCK(0,edid_buf, length)

	ret_val = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("%d", ret_val);
		return ne_NO_HPD;
	} else if (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & ret_val) {
		MHL_TX_DBG_INFO("EDID block read complete. ret_val:0x%02x\n",ret_val);
		return ne_SUCCESS;
	} else {
		MHL_TX_DBG_INFO("No HPD ret_val:0x%02x\n",ret_val);
		return ne_NO_HPD;
	}
}

int si_mhl_tx_drv_get_edid_fifo_next_block(struct drv_hw_context *hw_context,
										uint8_t *edid_buf)
{
	int ret_val;
	uint8_t offset;

	offset = EDID_BLOCK_SIZE * (hw_context->edid_fifo_block_number & 0x01);

	MHL_TX_DBG_INFO("%x %x\n",(unsigned int)hw_context,(unsigned int)edid_buf);
	hw_context->edid_fifo_block_number++;

#ifdef MANUAL_EDID_FETCH //(
	memcpy(edid_buf,hw_context->edid_block,EDID_BLOCK_SIZE);
#else //)(
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_FIFO_ADDR, offset);

	ret_val = mhl_tx_read_reg_block(hw_context
		, REG_PAGE_2_EDID_FIFO_RD_DATA
		, EDID_BLOCK_SIZE
		, edid_buf);
#endif //)

	DUMP_EDID_BLOCK(0,edid_buf, EDID_BLOCK_SIZE)

	ret_val = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("%d", ret_val);
		return ne_NO_HPD;
	} else if (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & ret_val) {
		MHL_TX_DBG_INFO("EDID block read complete. ret_val:0x%02x\n",ret_val);
		return ne_SUCCESS;
	} else {
		MHL_TX_DBG_INFO("No HPD ret_val:0x%02x\n",ret_val);
		return ne_NO_HPD;
	}
}

int si_mhl_tx_drv_get_scratch_pad(struct drv_hw_context *hw_context,
								uint8_t start_reg, uint8_t *data,
								uint8_t length)
{
	if ((start_reg + length) > (int)MHL_SCRATCHPAD_SIZE)
		return -1;

	memcpy(data, &hw_context->write_burst_data[start_reg], length);

	return 0;
}

static	bool packed_pixel_available(struct mhl_dev_context *dev_context)
{
	if ((MHL_DEV_VID_LINK_SUPP_PPIXEL & DEVCAP_VAL_VID_LINK_MODE) &&
		(dev_context->dev_cap_cache.mdc.vid_link_mode &
		 MHL_DEV_VID_LINK_SUPP_PPIXEL)) {

		return true;
	}
	return false;
}

#define SIZE_AVI_INFOFRAME 14
static uint8_t calculate_avi_info_frame_checksum(hw_avi_payload_t *payload)
{
	uint8_t checksum;

	checksum = 0x82 + 0x02 + 0x0D;	/* these are set by the hardware */
	return calculate_generic_checksum(payload->ifData, checksum,
						SIZE_AVI_INFOFRAME);
}

static int is_valid_avi_info_frame(struct mhl_dev_context *dev_context,
						avi_info_frame_t *avif)
{
	uint8_t	checksum;

	checksum = calculate_generic_checksum((uint8_t *)avif, 0, sizeof(*avif));
	if (0 != checksum) {
		MHL_TX_DBG_ERR("AVI info frame checksum is: 0x%02x "
						"should be 0\n", checksum);
		return 0;

	} else if (0x82 != avif->header.type_code) {
		MHL_TX_DBG_ERR("Invalid AVI type code: 0x%02x\n",
						avif->header.type_code);
		return 0;

	} else if (0x02 != avif->header.version_number) {
		MHL_TX_DBG_ERR("Invalid AVI version: 0x%02x\n",
						avif->header.version_number);
		return 0;

	} else if (0x0D != avif->header.length) {
		return 0;

	} else {
		return 1;
	}
}

static int is_valid_hdmi_vsif(struct mhl_dev_context *dev_context,
										vendor_specific_info_frame_t *vsif)
{
	uint8_t	checksum;

	checksum = calculate_generic_checksum((uint8_t *)vsif, 0,
			sizeof(vsif->header) + vsif->header.length );
	if (0 != checksum) {
		MHL_TX_DBG_WARN("VSIF info frame checksum is: 0x%02x "
					   "should be 0\n", checksum);
		/*
			Try again, assuming that the header includes the checksum.
		*/
		checksum = calculate_generic_checksum((uint8_t *)vsif, 0,
			sizeof(vsif->header) + vsif->header.length
			+ sizeof(vsif->payLoad.checksum));
	if (0 != checksum) {
			MHL_TX_DBG_ERR("VSIF info frame checksum "
					"(adjusted for checksum itself) is: 0x%02x "
						"should be 0\n", checksum);
		return 0;

		}
	}
	if (0x81 != vsif->header.type_code) {
		MHL_TX_DBG_ERR("Invalid VSIF type code: 0x%02x\n",
						vsif->header.type_code);
		return 0;

	} else if (0x01 != vsif->header.version_number) {
		MHL_TX_DBG_ERR("Invalid VSIF version: 0x%02x\n",
						vsif->header.version_number);
		return 0;

	} else {
		return 1;
	}
}

static void print_vic_modes(struct drv_hw_context *hw_context,uint8_t vic)
{
	int i;
	struct vic_name {
		uint8_t vic;
		char name[10];
	} vic_name_table[] = {
				{2, "480P"}
				,{4, "720P60"}
				,{5, "1080i60"}
				,{6, "480i"}
				,{16,"1080P60"}
				,{17,"576P50"}
				,{19,"720P50"}
				,{20,"1080i50"}
				,{21,"576i50"}
				,{31,"1080P50"}
				,{32,"1080P24"}
				,{33,"1080P25"}
				,{34,"1080P30"}
				,{0,""} /* to handle the case where the VIC is not found in the table */
	};
#define	NUM_VIC_NAMES (sizeof(vic_name_table)/sizeof(vic_name_table[0]))
	/* stop before the terminator */
	for(i = 0; i < (NUM_VIC_NAMES - 1); i++) {
		if (vic == vic_name_table[i].vic) {
			break;
		}
	}
	MHL_TX_DBG_ERR("VIC = %d (%s)\n", vic, vic_name_table[i].name);
}
#define DUMP_INFO_FRAMES
#ifdef DUMP_INFO_FRAMES //(
static void dump_avif_vsif_impl(struct drv_hw_context *hw_context,const char *function,int line_num)
{
	extern int debug_level;
	if (debug_level >= DBG_MSG_LEVEL_WARN) {
		vendor_specific_info_frame_t	vsif;
		avi_info_frame_t				avif;
		int i;
		unsigned char *c;
		mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval
				| VAL_PAGE_2_RX_HDMI_CTRL2_VSI_MON_SEL_VSI);

		mhl_tx_read_reg_block(hw_context
				, REG_PAGE_2_RX_HDMI_MON_PKT_HEADER1
				, sizeof(vsif)
				, (uint8_t *)&vsif
				);

		mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval
				| VAL_PAGE_2_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);

		mhl_tx_read_reg_block(hw_context
					, REG_PAGE_2_RX_HDMI_MON_PKT_HEADER1
					, sizeof(avif)
					, (uint8_t *)&avif
					);
		print_formatted_debug_msg(NULL,function,line_num,"VSIF:");
		for (i=0,c=(unsigned char *)&vsif;i<sizeof(vsif);++i,++c) {
			printk(" %02x",*c);
		}
		printk("\n");
		print_formatted_debug_msg(NULL,function,line_num,"AVIF:");
		for (i=0,c=(unsigned char *)&avif;i<sizeof(avif);++i,++c) {
			printk(" %02x",*c);
		}
		printk("\n");
	}
}

#define DUMP_AVIF_VSIF(hw_context) dump_avif_vsif_impl(hw_context,__func__,__LINE__);
#else //(
#define DUMP_AVIF_VSIF(hw_context) /* nothing */
#endif //)

typedef enum {
		 use_avi_vic
		,use_hdmi_vic
		,use_mhl3_sequence_index
		,use_hardware_totals
} timing_info_basis_e;

static bool process_hdmi_vsif(struct drv_hw_context			*hw_context
							,avi_info_frame_data_byte_4_t	*p_input_video_code
							,timing_info_basis_e 			*p_timing_info_basis
							,enum mhl_vid_fmt_e 			*p_vid_fmt
							,uint32_t						*p_threeDPixelClockRatio
							,uint8_t                        *p_fp_3d_mode
							,enum mhl_3d_fmt_type_e 		*p_3d_fmt_type
						)
{
	uint8_t input_VIC = (uint8_t) (*p_input_video_code).VIC;
	/*
		HDMI spec. v1.4:
		"When transmitting any 2D video format of section 6.3 above, an HDMI Source shall set
		the VIC field to the Video Code for that format. See CEA-861-D section 6.4 for detatils.  The
		additional VIC values from 60 to 64 are defined in Table 8-4.
		When transmitting any 3D video format using the 3D_Structure field in the HDMI Vendor
		Specific InfoFrame, an HDMI Source shall set the AVI InfoFrame VIC field to satisfy the
		relation described in section 8.2.3.2.
		When transmitting any extended video format indicated through use of the HDMI_VIC field
		in the HDMI Vendor Specific InfoFrame or any other format which is not described in the
		above cases, and HDMI Source shall set the AVI InfoFrame VIC field to zero."
	*/
	MHL_TX_DBG_WARN("valid HDMI VSIF\n");
	print_vic_modes(hw_context, input_VIC);
	if (0 == input_VIC) {
		if (hvfExtendedResolutionFormatPresent == hw_context->current_vs_info_frame.
				payLoad.pb4.HDMI_Video_Format) {
			uint8_t vic = hw_context->current_vs_info_frame.payLoad.pb5.HDMI_VIC;
			/* HDMI_VIC should contain one of 0x01 through 0x04 */
			MHL_TX_DBG_ERR("HDMI extended resolution %d\n",vic);
			*p_timing_info_basis = use_hdmi_vic;
		} else {
			MHL_TX_DBG_ERR("AVI VIC is zero!!!\n");
			/*
			 * Instead of no video, let us attempt HDCP and if possible show video
			 * If hdcp fails due to clock difference on input (which we don't know
			 * about clearly), after 5 attempts it will anyways stabilize and use
			 * an infoframe interrupt if that arrives with a good vic.
			 *
			 * TODO: Maybe we should flag arrival of an infoframe from the time
			 * this path was executed and abort HDCP a bit earlier.
			 */
			return	false;
		}
	}
	/*
	 * From VSIF bytes, figure out if we need to perform
	 * frame packing or not. This helps decide if packed pixel
	 * (16-bit) is required or not in conjunction with the VIC.
	 */

	if (hvf3DFormatIndicationPresent == hw_context->current_vs_info_frame.
			payLoad.pb4.HDMI_Video_Format) {

		*p_vid_fmt = mhl_vid_fmt_3d_fmt_present;

		MHL_TX_DBG_INFO("VSIF indicates 3D\n");
		switch (hw_context->current_vs_info_frame.
					payLoad.pb5.ThreeDStructure.threeDStructure) {
		case tdsFramePacking:
			MHL_TX_DBG_INFO("mhl_tx: tdsFramePacking\n");
			*p_threeDPixelClockRatio = 2;
			*p_fp_3d_mode |= VAL_PAGE_0_VID_OVRRD_3DCONV_EN_FRAME_PACK;
			*p_3d_fmt_type = mhl_3d_fmt_type_frame_sequential;
			break;
		case tdsTopAndBottom:
			*p_3d_fmt_type = mhl_3d_fmt_type_top_bottom;
			break;
		case tdsSideBySide:
			*p_3d_fmt_type = mhl_3d_fmt_type_left_right;
			break;
		default:
			break;
		}
	}

	return true;
}

/*
 * This function must not be called for DVI mode.
 */
static int set_hdmi_params(struct mhl_dev_context *dev_context)
{
	extern int vic_override;
	uint32_t						pixel_clock_frequency;
	uint32_t						threeDPixelClockRatio;
	uint8_t							packedPixelNeeded=0;
	uint8_t							fp_3d_mode;
	AviColorSpace_e					input_clr_spc =acsRGB;
	uint8_t							output_clr_spc =acsRGB;
	avi_info_frame_data_byte_4_t	input_video_code;
	avi_info_frame_data_byte_4_t	output_video_code;
	struct drv_hw_context			*hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	timing_info_basis_e 			timing_info_basis=use_avi_vic;
	/* default values for MHL3 VSIF, which we will always send */
	enum mhl_vid_fmt_e vid_fmt 			= mhl_vid_fmt_no_additional;
	enum mhl_3d_fmt_type_e _3d_fmt_type = mhl_3d_fmt_type_frame_sequential;
	enum mhl_sep_audio_e sep_aud 		= mhl_sep_audio_not_available;
	enum mhl_hev_fmt_e hev_fmt			= mhl_hev_fmt_no_additional;
	uint16_t hev_fmt_type				= 0;
	uint32_t delay_sync					= 0;
	enum mhl_av_delay_dir_e	delay_dir	= mhl_av_delay_dir_audio_earlier;

	/* Extract VIC from incoming AVIF */
	input_video_code = hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.
							ifData_u.bitFields.VIC;
	threeDPixelClockRatio = 1;
	fp_3d_mode = REG_PAGE_0_VID_OVRRD_DEFVAL | BIT_PAGE_0_VID_OVRRD_M1080P_OVRRD;
MHL_TX_DBG_WARN("called\n");
	/* did we get an MHL3 vsif from the callback API? */
	if (hw_context->valid_vsif) {
		if (hw_context->callbacks.hpd_driven_high) {
			switch (hw_context->hpd_high_callback_status) {
			case hpd_high_format_HDMI_vsif_mhl3_hdcp_on:
			case hpd_high_format_HDMI_vsif_mhl3:
			case hpd_high_format_HDMI_vsif_mhl3_hdcp_on_not_repeatable:
			case hpd_high_format_HDMI_vsif_mhl3_not_repeatable:
				MHL_TX_DBG_WARN("MHL3 VSIF from callback\n");
				switch (hw_context->vsif_mhl3_or_hdmi_from_callback.mhl3_vsif.pb4 &PB4_MASK_MHL_VID_FMT) {
				case mhl_vid_fmt_no_additional:
					break;
				case mhl_vid_fmt_3d_fmt_present:
					_3d_fmt_type = hw_context->vsif_mhl3_or_hdmi_from_callback.mhl3_vsif.pb4 & PB4_MASK_MHL_3D_FMT_TYPE;
					switch (_3d_fmt_type) {
					case mhl_3d_fmt_type_top_bottom:
					case mhl_3d_fmt_type_left_right:
					case mhl_3d_fmt_type_top_bottom_left_right:
						break;
					case mhl_3d_fmt_type_frame_sequential:
					case mhl_3d_fmt_type_frame_sequential_top_bottom:
					case mhl_3d_fmt_type_frame_sequential_left_right:
						threeDPixelClockRatio = 2;
						fp_3d_mode |= VAL_PAGE_0_VID_OVRRD_3DCONV_EN_FRAME_PACK;
						break;
					}
					break;
				case mhl_vid_fmt_multi_view:
					break;
				case mhl_vid_fmt_dual_3d:
					break;
				}
				switch (hw_context->vsif_mhl3_or_hdmi_from_callback.mhl3_vsif.pb6 & PB6_MASK_MHL_HEV_FMT) {
				case mhl_hev_fmt_hev_present:
					timing_info_basis = use_mhl3_sequence_index;
					break;
				case mhl_hev_fmt_no_additional:
				case mhl_hev_fmt_reserved_2:
				case mhl_hev_fmt_reserved_3:
					/* nothing to do in these cases */
					break;
				}
				break;
			case hpd_high_format_HDMI_vsif_hdmi:
			case hpd_high_format_HDMI_vsif_hdmi_hdcp_on:
			case hpd_high_format_HDMI_vsif_hdmi_not_repeatable:
			case hpd_high_format_HDMI_vsif_hdmi_hdcp_on_not_repeatable:
				MHL_TX_DBG_WARN("HDMI VSIF from callback\n");
				if (false == process_hdmi_vsif(hw_context
								,&input_video_code
								,&timing_info_basis
								,&vid_fmt
								,&threeDPixelClockRatio
								,&fp_3d_mode
								,&_3d_fmt_type
								))
					//return false;
				break;
			default:
				MHL_TX_DBG_WARN("HDMI VSIF from data islands\n");
				if (false == process_hdmi_vsif(hw_context
								,&input_video_code
								,&timing_info_basis
								,&vid_fmt
								,&threeDPixelClockRatio
								,&fp_3d_mode
								,&_3d_fmt_type
								)) {
					//return false;
				}
				break;
			}
		} else {
			MHL_TX_DBG_WARN("HDMI VSIF from data islands\n");
			if (false == process_hdmi_vsif(hw_context
							,&input_video_code
							,&timing_info_basis
							,&vid_fmt
							,&threeDPixelClockRatio
							,&fp_3d_mode
							,&_3d_fmt_type
							)) {
				//return false;
			}
		}
	} else { /* no incoming HDMI VSIF */
		if (0 == input_video_code.VIC) {
			/*
				This routine will not be called until we positively know (from the downstream EDID)
					that the sink is HDMI.
				We do not support DVI only sources.  The upstream source is expected to choose between
					HDMI and DVI based upon the EDID that we present upstream.
				The other information in the infoframe, even if it is non-zero, is not helpful for
					determining the pixel clock frequency.
				So we try as best we can to infer the pixel clock from the HTOTAL and VTOTAL registers.
			*/
			timing_info_basis = use_hardware_totals;
			MHL_TX_DBG_ERR("no VSIF and AVI VIC (offset 0x%x) is zero!!! trying HTOTAL/VTOTAL\n"
				,(size_t)&hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.ifData_u.bitFields.VIC
				-(size_t)&hw_context->current_avi_info_frame
				);
			DUMP_AVIF_VSIF(hw_context)
			/* Return error to avoid further processing. */
			//return false;
		} else {
			print_vic_modes(hw_context, (uint8_t) input_video_code.VIC);
		}
	}
	mhl_tx_write_reg(hw_context, REG_PAGE_0_VID_OVRRD, fp_3d_mode);
	/* Do not remember previous VSIF */
	hw_context->valid_vsif = 0;
	hw_context->valid_avif = 0;

	/* make a copy of avif */
	hw_context->outgoingAviPayLoad = hw_context->current_avi_info_frame.payLoad.hwPayLoad;
	switch (timing_info_basis) {
	case use_avi_vic:

		/* compute pixel frequency */
		pixel_clock_frequency = si_edid_find_pixel_clock_from_AVI_VIC(
						dev_context->edid_parser_context, input_video_code.VIC);
		output_video_code = input_video_code;
		break;
	case use_hdmi_vic:
		output_video_code.VIC = hw_context->current_vs_info_frame.payLoad.pb5.HDMI_VIC;
		pixel_clock_frequency = si_edid_find_pixel_clock_from_HDMI_VIC(
						dev_context->edid_parser_context
						,output_video_code.VIC
						);
		output_video_code.VIC = si_edid_map_hdmi_vic_to_mhl3_vic(
						dev_context->edid_parser_context,output_video_code.VIC);
		break;
	case use_mhl3_sequence_index:
		output_video_code.VIC=0;
		pixel_clock_frequency = si_edid_find_pixel_clock_from_HEV_DTD(
				dev_context->edid_parser_context
				,hw_context->vsif_mhl3_or_hdmi_from_callback.mhl3_vsif.mhl_hev_fmt_type);
		break;
	case use_hardware_totals:
		output_video_code = input_video_code;
		pixel_clock_frequency = si_mhl_tx_find_timings_from_totals(
				dev_context->edid_parser_context);
		if (0 == pixel_clock_frequency) {
			MHL_TX_DBG_ERR("VIC was zero and totals not supported\n");
			//return false;
		}
		break;
	default:
		MHL_TX_DBG_ERR("%s'shouldn't get here, the following assignment"
					" statement exists to avoid a compiler warning%s\n"
					,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT
					);
		pixel_clock_frequency=0;
		break;
	}

	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.bitFields.VIC=
				output_video_code;
	vic_override = output_video_code.VIC;
	/* extract input color space */
	input_clr_spc = hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.
							ifData_u.bitFields.pb1.colorSpace;

	MHL_TX_DBG_INFO("input_clr_spc = %02X infoData[0]:%02X\n",
						input_clr_spc,
						hw_context->current_avi_info_frame.payLoad.hwPayLoad.namedIfData.ifData_u.
						infoFrameData[0]);

	/*
	 * decide about packed pixel mode
	 */
	pixel_clock_frequency *= threeDPixelClockRatio;
	MHL_TX_DBG_INFO("pixel clock:%u\n", pixel_clock_frequency);

	if (qualify_pixel_clock_for_mhl(dev_context->edid_parser_context,
		pixel_clock_frequency, 24)) {
		MHL_TX_DBG_INFO("OK for 24 bit pixels\n");
	} else {
		/* not enough bandwidth for uncompressed video */
		if (si_edid_sink_supports_YCbCr422(dev_context->edid_parser_context)) {
			MHL_TX_DBG_INFO("Sink supports YCbCr422\n");

			if (qualify_pixel_clock_for_mhl(
				dev_context->edid_parser_context, pixel_clock_frequency, 16)) {
				/* enough for packed pixel */
				packedPixelNeeded = 1;
			} else {
				MHL_TX_DBG_ERR("unsupported video mode."
						"pixel clock too high %s\n"
						,si_peer_supports_packed_pixel(dev_context)
							? "" :"(peer does not support packed pixel)."
						);
				//return	false;
			}
		} else {
			MHL_TX_DBG_ERR("unsupported video mode."
					"Sink doesn't support 4:2:2.\n");
			//return	false;
		}
	}

	/*
	 * Determine output color space if it needs to be 4:2:2 or same as input
	 */
	output_clr_spc = input_clr_spc;

	if (packedPixelNeeded) {
		if (packed_pixel_available(dev_context)) {
			MHL_TX_DBG_INFO("setting packed pixel mode\n");

			dev_context->link_mode = MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_PACKED_PIXEL;
			/* enforcing 4:2:2 if packed pixel. */
			output_clr_spc = BIT_EDID_FIELD_FORMAT_YCbCr422;

			if (IN_MHL3_MODE(hw_context)) {
				mhl_tx_modify_reg(hw_context, REG_PAGE_3_M3_P0CTRL
								, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PIXEL_MODE
								, VAL_PAGE_3_M3_P0CTRL_MHL3_P0_PIXEL_MODE_PACKED
								);
			} else {
				mhl_tx_write_reg(hw_context, REG_PAGE_0_VID_MODE,
											VAL_PAGE_0_VID_MODE_M1080P_ENABLE);
				mhl_tx_write_reg(hw_context,REG_PAGE_3_MHL_TOP_CTL, 0x41);

				mhl_tx_write_reg(hw_context, REG_PAGE_2_MHLTX_CTL6, 0x60);
			}

		} else {
			MHL_TX_DBG_ERR(
				"unsupported video mode. Packed Pixel not available on sink."
				"Sink's link mode = 0x%02x\n",
				dev_context->dev_cap_cache.mdc.vid_link_mode);
			//return false;
		}
	} else {
		MHL_TX_DBG_INFO("normal Mode ,Packed Pixel mode disabled\n");

		dev_context->link_mode = MHL_STATUS_PATH_ENABLED | MHL_STATUS_CLK_MODE_NORMAL;

		if (IN_MHL3_MODE(hw_context)) {
				mhl_tx_modify_reg(hw_context, REG_PAGE_3_M3_P0CTRL
								, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PIXEL_MODE
								, VAL_PAGE_3_M3_P0CTRL_MHL3_P0_PIXEL_MODE_NORMAL
								);
		} else {
			mhl_tx_write_reg(hw_context, REG_PAGE_0_VID_MODE,
										VAL_PAGE_0_VID_MODE_M1080P_DISABLE);
			mhl_tx_write_reg(hw_context,REG_PAGE_3_MHL_TOP_CTL, 0x01);
			mhl_tx_write_reg(hw_context, REG_PAGE_2_MHLTX_CTL6, 0xA0);
		}
	}

	/* Set input color space */
	mhl_tx_write_reg(hw_context
			, REG_PAGE_6_TPI_INPUT
			, colorSpaceTranslateInfoFrameToHw[input_clr_spc]);

	/* Set output color space */
	mhl_tx_write_reg(hw_context
			, REG_PAGE_6_TPI_OUTPUT
			, colorSpaceTranslateInfoFrameToHw[output_clr_spc]);

	if (IN_MHL3_MODE(hw_context)) {
		MHL_bits_per_pixel_fmt_data_t bpp_fmt;
		PMHL_bits_per_pixel_fmt_data_t p_buffer;
		size_t xfer_size;
		if (bpp_on_wb) {
			p_buffer = &bpp_fmt;
			xfer_size = sizeof(bpp_fmt);
			/* use padding for WRITE_BURSTS */
			p_buffer->descriptors[1].stream_id = 0;
			p_buffer->descriptors[1].stream_pixel_format = 0;
			p_buffer->descriptors[2].stream_id = 0;
			p_buffer->descriptors[2].stream_pixel_format = 0;
			p_buffer->descriptors[3].stream_id = 0;
			p_buffer->descriptors[3].stream_pixel_format = 0;
			p_buffer->descriptors[4].stream_id = 0;
			p_buffer->descriptors[4].stream_pixel_format = 0;
	 	} else {
			/* only one descriptor to send */
			xfer_size =	  sizeof(bpp_fmt)
						- sizeof(p_buffer->descriptors)
						+ sizeof(p_buffer->descriptors[0]);
			p_buffer= si_mhl_tx_get_sub_payload_buffer(dev_context,xfer_size);
		}

		if (p_buffer) {
			p_buffer->header.burst_id.low = LOW_BYTE_16(burst_id_BITS_PER_PIXEL_FMT);
			p_buffer->header.burst_id.high= HIGH_BYTE_16(burst_id_BITS_PER_PIXEL_FMT);
			p_buffer->header.checksum = 0;
			p_buffer->header.total_entries = 1;
			p_buffer->header.sequence_index = 1;
			p_buffer->num_entries_this_burst = 1;
			p_buffer->descriptors[0].stream_id = 0;
			switch (dev_context->link_mode & MHL_STATUS_CLK_MODE_MASK) {
			case MHL_STATUS_CLK_MODE_PACKED_PIXEL:
				p_buffer->descriptors[0].stream_pixel_format = VIEW_PIX_FMT_16BPP;
				break;
			case MHL_STATUS_CLK_MODE_NORMAL:
				p_buffer->descriptors[0].stream_pixel_format = VIEW_PIX_FMT_24BPP;
				break;
			}
			p_buffer->header.checksum=
						calculate_generic_checksum(p_buffer, 0,xfer_size);
			if (bpp_on_wb) {
				si_mhl_tx_send_write_burst(dev_context, &bpp_fmt);
			} else {
				si_mhl_tx_push_block_transactions(dev_context);
			}
		}
		/* enable TMDS */
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xF0);		/* 0x0331 */

		/* fill in values for MHL3 VSIF */
		hw_context->outgoing_mhl3_vsif.header.type_code = MHL3_VSIF_TYPE;
		hw_context->outgoing_mhl3_vsif.header.version_number = MHL3_VSIF_VERSION;
		hw_context->outgoing_mhl3_vsif.header.length = sizeof(hw_context->outgoing_mhl3_vsif);
		hw_context->outgoing_mhl3_vsif.checksum = 0;
		hw_context->outgoing_mhl3_vsif.iee_oui[0]= (uint8_t)( IEEE_OUI_MHL3     & 0xFF);
		hw_context->outgoing_mhl3_vsif.iee_oui[1]= (uint8_t)((IEEE_OUI_MHL3>> 8)& 0xFF);
		hw_context->outgoing_mhl3_vsif.iee_oui[2]= (uint8_t)((IEEE_OUI_MHL3>>16)& 0xFF);
		hw_context->outgoing_mhl3_vsif.pb4	= MHL3_VSIF_PB4(vid_fmt,_3d_fmt_type,sep_aud);
		hw_context->outgoing_mhl3_vsif.pb5_reserved = 0;
		hw_context->outgoing_mhl3_vsif.pb6  = MHL3_VSIF_PB6(hev_fmt);
		hw_context->outgoing_mhl3_vsif.mhl_hev_fmt_type.high = HIGH_BYTE_16(hev_fmt_type);
		hw_context->outgoing_mhl3_vsif.mhl_hev_fmt_type.low  = LOW_BYTE_16(hev_fmt_type);
		hw_context->outgoing_mhl3_vsif.pb9  = MHL3_VSIF_PB9(delay_sync,delay_dir);
		hw_context->outgoing_mhl3_vsif.av_delay_sync.high    = HIGH_BYTE_16(delay_sync);
		hw_context->outgoing_mhl3_vsif.av_delay_sync.low     = LOW_BYTE_16(delay_sync);

		hw_context->outgoing_mhl3_vsif.checksum =
				calculate_generic_checksum(&hw_context->outgoing_mhl3_vsif
						, 0,hw_context->outgoing_mhl3_vsif.header.length);

		/*
		 * Program TMDS link speeds
		*/
		switch (dev_context->link_mode & MHL_STATUS_CLK_MODE_MASK) {
			case MHL_STATUS_CLK_MODE_PACKED_PIXEL:
				si_mhl_tx_drv_set_lowest_tmds_link_speed(dev_context, pixel_clock_frequency, 16);
				break;
			case MHL_STATUS_CLK_MODE_NORMAL:
	 			si_mhl_tx_drv_set_lowest_tmds_link_speed(dev_context, pixel_clock_frequency, 24);
	 			break;
		}
	} else {
		/*
		 * MSC WRITE_STATUS is required to prepare sink for new mode
		 */
		si_mhl_tx_set_status(dev_context, false, MHL_STATUS_REG_LINK_MODE,
										dev_context->link_mode);
	}
	/*
	 * Prepare outgoing AVIF for later programming the registers
	 *
	 * the checksum itself is included in the calculation.
	 */
	hw_context->outgoingAviPayLoad.namedIfData.checksum = 0;
	hw_context->outgoingAviPayLoad.namedIfData.ifData_u.bitFields.pb1.colorSpace
			= output_clr_spc;
	hw_context->outgoingAviPayLoad.ifData[1] &= 0x7F;	// Ensure bit 7 of PB1 = 0.
	hw_context->outgoingAviPayLoad.ifData[4] &= 0x7F;	// Ensure bit 7 of PB4 = 0.
	//hw_context->outgoingAviPayLoad.ifData[5] &= 0x0F;	// Ensure bits 7:4 of PB5 = 0 clearing the YCC Quantization here causes MHL System/Protocol test 3.2.3.4 to fail
	hw_context->outgoingAviPayLoad.namedIfData.checksum =
			calculate_avi_info_frame_checksum(&hw_context->outgoingAviPayLoad);

	return true;
}

/*
	process_info_frame_change
		called by the MHL Tx driver when a
		new AVI info frame is received from upstream
		OR
		called by customer's SOC video driver when a mode change is desired.
*/

void process_info_frame_change(struct drv_hw_context *hw_context
									, vendor_specific_info_frame_t *hdmi_vsif
									, avi_info_frame_t *avif)
{
	bool mode_change = false;
	struct mhl_dev_context	*dev_context;

	dev_context = container_of((void *)hw_context, struct mhl_dev_context,
								drv_context);
	if (NULL != hdmi_vsif) {
		if (is_valid_hdmi_vsif(dev_context, hdmi_vsif)) {
			hw_context->current_vs_info_frame = *hdmi_vsif;
			hw_context->valid_vsif = 1;
			mode_change = true;
		}
	}
	if (NULL != avif) {
		if (is_valid_avi_info_frame(dev_context, avif)) {
			hw_context->current_avi_info_frame = *avif;
			hw_context->valid_avif = 1;
			mode_change = true;
		}
	}
	if (mode_change) {
		int cstat_p3;
		int bits_of_interest;
		cstat_p3 = mhl_tx_read_reg(hw_context, REG_PAGE_2_TMDS_CSTAT_P3);
		bits_of_interest = cstat_p3 & (BIT_PAGE_2_TMDS_CSTAT_P3_SCDT | BIT_PAGE_2_TMDS_CSTAT_P3_CKDT);

		if ((BIT_PAGE_2_TMDS_CSTAT_P3_SCDT | VAL_PAGE_2_TMDS_CSTAT_P3_CKDT_DETECTED)
			== bits_of_interest) {
			start_video(hw_context);
		}
	}
}

#define dump_edid_fifo(hw_context, block_number) /* do nothing */

#define RX_DPD_BITS (BIT_PAGE_0_DPD_PDNRX12 \
					  | BIT_PAGE_0_DPD_PDIDCK_N \
					  | BIT_PAGE_0_DPD_PD_MHL_CLK_N)
static int init_rx_regs(struct drv_hw_context *hw_context)
{
	/* power up the RX */
	mhl_tx_modify_reg(hw_context, REG_PAGE_0_DPD,
					  RX_DPD_BITS,RX_DPD_BITS);

	/* TODO: add to PR. Default for 2A4 is 0x0f */
	mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL3, 0x00);

	/* Due to packet overflow, AVIF write using TPI:0C fails.
	Until ready, we drop all packets*/
	mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_0, 0xFF);
	mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_1, 0xFF);

	mhl_tx_modify_reg(hw_context, REG_PAGE_2_RX_HDMI_CLR_BUFFER,
					BIT_PAGE_2_RX_HDMI_CLR_BUFFER_VSI_CLR_EN,
					VAL_PAGE_2_RX_HDMI_CLR_BUFFER_VSI_CLR_EN_CLEAR);

	return 0;
}

#ifdef USE_HW_TIMER
static void start_hw_timer (struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Start HW Timer.\n");
	mhl_tx_write_reg(hw_context,REG_PAGE_0_SYS_CTRL3, BIT_PAGE_0_SYS_CTRL3_SYS_CNTR);
}

static void stop_hw_timer (struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Stop HW Timer.\n");
	mhl_tx_write_reg(hw_context, REG_PAGE_0_SYS_CTRL3, 0x00);
}

/*
Time (ms)	Reg value
  5000		  2710
  3000		  1770
  2000		  0FA0
  1500		  0BB8
*/
static void setup_hw_timer (struct drv_hw_context *hw_context, uint16_t time_ms)
{
	MHL_TX_DBG_INFO("Setup HW Timer for %dms.\n", time_ms);

	// Max time is 32,767ms.
	time_ms &= 0x7FFF;

	// Divide by 0.5 for register value.
	time_ms <<= 1;

	mhl_tx_write_reg(hw_context,REG_PAGE_0_SYS_CNTR_0, (uint8_t)(time_ms & 0x00FF));
	mhl_tx_write_reg(hw_context,REG_PAGE_0_SYS_CNTR_1, (uint8_t)((time_ms >> 8) & 0x00FF));
}
#endif

/* this function is exported from the driver */
int si_mhl_tx_drv_set_display_mode(struct mhl_dev_context *dev_context
				,hpd_high_callback_status status)
{
	struct drv_hw_context *hw_context;
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	hw_context->hpd_high_callback_status = status;
	hw_context->valid_avif = true;
	MHL_TX_DBG_INFO("called\n");
	/* did the client begin sending video? */
	if (status >= 0) {
		switch (status) {
		case hpd_high_format_DVI_hdcp_on:
		case hpd_high_format_DVI:
		case hpd_high_format_DVI_hdcp_on_not_repeatable:
		case hpd_high_format_DVI_not_repeatable:
			hw_context->valid_avif = false;
			/*
			 * output video here for DVI
			 */
			start_video(hw_context);
			break;
		case hpd_high_format_HDMI_vsif_none_hdcp_on:
		case hpd_high_format_HDMI_vsif_none:
		case hpd_high_format_HDMI_vsif_none_hdcp_on_not_repeatable:
		case hpd_high_format_HDMI_vsif_none_not_repeatable:
			process_info_frame_change(hw_context
					, NULL
					, &hw_context->avif_or_dtd_from_callback.avif
					);
			break;

		case hpd_high_format_HDMI_vsif_mhl3_hdcp_on:
		case hpd_high_format_HDMI_vsif_mhl3:
		case hpd_high_format_HDMI_vsif_mhl3_hdcp_on_not_repeatable:
		case hpd_high_format_HDMI_vsif_mhl3_not_repeatable:
			hw_context->valid_vsif = true;
			process_info_frame_change(hw_context
					, NULL /* set_hdmi_params will look at the MHL3 vsif in the hw_context */
					, &hw_context->avif_or_dtd_from_callback.avif
					);
			break;
		case hpd_high_format_HDMI_vsif_hdmi_hdcp_on:
		case hpd_high_format_HDMI_vsif_hdmi:
		case hpd_high_format_HDMI_vsif_hdmi_hdcp_on_not_repeatable:
		case hpd_high_format_HDMI_vsif_hdmi_not_repeatable:
			hw_context->valid_vsif = true;
			process_info_frame_change(hw_context
					, &hw_context->vsif_mhl3_or_hdmi_from_callback.hdmi_vsif
					, &hw_context->avif_or_dtd_from_callback.avif
					);
			break;
		default:
			/* other values are not applicable */
			break;
		}
	}
	return 0;
}

#define BIT_GPIO_0_DISABLED	0x01
#define BIT_GPIO_0_HIGH 	0x02
#define BIT_GPIO_1_DISABLED	0x04
#define BIT_GPIO_1_HIGH		0x08

#define BITS_GPIO_01_HPD_HIGH	(BIT_GPIO_0_HIGH | BIT_GPIO_1_HIGH)
#define BITS_GPIO_01_HPD_LOW	0

#define BITS_HPD_CTRL_OPEN_DRAIN_HIGH (BITS_GPIO_01_HPD_HIGH | 0x70 )
#define BITS_HPD_CTRL_PUSH_PULL_HIGH  (BITS_GPIO_01_HPD_HIGH | 0x30 )

#define BITS_HPD_CTRL_OPEN_DRAIN_LOW (BITS_GPIO_01_HPD_LOW  |  0x50)
#define BITS_HPD_CTRL_PUSH_PULL_LOW  (BITS_GPIO_01_HPD_LOW  |  0x10)
/*
	drive_hpd_high -- sets upstream HPD high and returns the value of REG_PAGE_2_TMDS_CSTAT_P3
*/
static int drive_hpd_high(struct drv_hw_context *hw_context,uint8_t *edid, uint16_t length)
{
	hpd_control_mode mode;
	int ret_val = -1;
	int cstat_p3;
	MHL_TX_DBG_INFO("drive_hpd_high................\n");
	mode = platform_get_hpd_control_mode();

	/* sample REG_PAGE_2_TMDS_CSTAT_P3 __BEFORE__ driving upstream HDP high */
	cstat_p3 = mhl_tx_read_reg(hw_context, REG_PAGE_2_TMDS_CSTAT_P3);

	/* disable auto-clear */
	cstat_p3 |= BIT_PAGE_2_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR;
	mhl_tx_write_reg(hw_context,REG_PAGE_2_TMDS_CSTAT_P3,cstat_p3);

	if (HPD_CTRL_OPEN_DRAIN == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_PAGE_0_HPD_CTRL, BITS_HPD_CTRL_OPEN_DRAIN_HIGH);
	else if (HPD_CTRL_PUSH_PULL == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_PAGE_0_HPD_CTRL, BITS_HPD_CTRL_PUSH_PULL_HIGH);

	if (edid) {
		if (hw_context->callbacks.hpd_driven_high) {
			int	status;
			struct mhl_dev_context *dev_context = container_of((void *)hw_context,
									struct mhl_dev_context, drv_context);

			edid_3d_data_p p_edid_data = dev_context->edid_parser_context;

			status = hw_context->callbacks.hpd_driven_high(
							hw_context->callbacks.context
							,edid,length
							,p_edid_data->hev_dtd_list,p_edid_data->hev_dtd_info.num_items
							,p_edid_data->hev_vic_list,p_edid_data->hev_vic_info.num_items
							,p_edid_data->_3d_dtd_list,p_edid_data->_3d_dtd_info.num_items
							,p_edid_data->_3d_vic_list,p_edid_data->_3d_vic_info.num_items
							,&hw_context->avif_or_dtd_from_callback
								,sizeof(hw_context->avif_or_dtd_from_callback)
							,&hw_context->vsif_mhl3_or_hdmi_from_callback
								,sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback)
							);
			si_mhl_tx_drv_set_display_mode(dev_context ,status);
		}
	}
	if (ret_val >= 0) {
		return cstat_p3;
	}
	return ret_val;
}

static int drive_hpd_low(struct drv_hw_context *hw_context)
{
	hpd_control_mode mode;
	int ret_val=-1;
	MHL_TX_DBG_INFO("drive_hpd_low................\n");
	ddc_abort_count = 0;
	mode = platform_get_hpd_control_mode();

	mhl_tx_modify_reg(hw_context
		, REG_PAGE_2_EDID_CTRL
		, BIT_PAGE_2_EDID_CTRL_EDID_PRIME_VALID
		, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
		);

	if (HPD_CTRL_OPEN_DRAIN == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_PAGE_0_HPD_CTRL, BITS_HPD_CTRL_OPEN_DRAIN_LOW);
	else if (HPD_CTRL_PUSH_PULL == mode)
		ret_val = mhl_tx_write_reg(hw_context, REG_PAGE_0_HPD_CTRL, BITS_HPD_CTRL_PUSH_PULL_LOW);

	enable_intr(hw_context,INTR_INFR,0x00);
	hw_context->hpd_high_callback_status = hpd_high_video_not_ready;
	if (hw_context->callbacks.hpd_driven_low) {
		hw_context->callbacks.hpd_driven_low(hw_context->callbacks.context);
	}
	return ret_val;
}

#ifdef SWWA_BZ30759 //(
/* we are done with the EDID for now.  We now expect to start doing HDCP, which can
	destroy the contents of our EDID buffer, so do another EDID read, which we know will fail,
	but that will reset the DDC fifo in such a way as to leave the buffer contents alone
*/
void edid_hw_sm_clean_up(struct drv_hw_context *hw_context)
{
	uint8_t	ddc_status,cbus_status;
	mhl_tx_write_reg(hw_context,REG_080C_HDCP1X_LB_BIST,BIT_080C_HDCP1X_LB_BIST);
	mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_MANUAL,BIT_PAGE_0_DDC_MANUAL_MAN_DDC);
	mhl_tx_write_reg(hw_context,REG_PAGE_2_INTR9,0xFF);

	/* Disable EDID interrupt */
	enable_intr(hw_context, INTR_EDID, 0);

	cbus_status = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
	if (!(BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED & cbus_status)) {
		mhl_tx_write_reg(hw_context,REG_PAGE_5_DISC_STAT1,0x08);
		mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL5
				,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE
				,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE);
	}

	mhl_tx_write_reg(hw_context,REG_PAGE_2_TPI_CBUS_START,BIT_PAGE_2_TPI_CBUS_START_GET_EDID_START_0);
	ddc_status = mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);

	if (!(BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED & cbus_status)) {
		mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL5
				,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE
				,0);
	}

	mhl_tx_write_reg(hw_context,REG_PAGE_2_INTR9,0xFF);
	mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_MANUAL,0x00);
	mhl_tx_write_reg(hw_context,REG_080C_HDCP1X_LB_BIST,0x00);

	ddc_status = mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);

	mhl_tx_modify_reg(hw_context, REG_PAGE_0_LM_DDC,
					BIT_PAGE_0_LM_DDC_SW_TPI_EN,
					VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED);
	if (BIT_PAGE_0_DDC_STATUS_DDC_BUS_LOW & ddc_status) {
		MHL_TX_DBG_WARN("Clearing DDC ack status\n");
		mhl_tx_write_reg(hw_context, REG_PAGE_0_DDC_STATUS,
				ddc_status & ~BIT_PAGE_0_DDC_STATUS_DDC_BUS_LOW);
	}

	mhl_tx_modify_reg(hw_context, REG_PAGE_0_LM_DDC,
					BIT_PAGE_0_LM_DDC_SW_TPI_EN,
					VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED);

}

void edid_hw_sm_shutdown(struct drv_hw_context *hw_context)
{
	uint8_t	ddc_status;
	enable_intr(hw_context, INTR_DISC, 0);

	mhl_tx_write_reg(hw_context,REG_080C_HDCP1X_LB_BIST,BIT_080C_HDCP1X_LB_BIST);
	mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_MANUAL,BIT_PAGE_0_DDC_MANUAL_MAN_DDC);
	mhl_tx_write_reg(hw_context,REG_PAGE_2_INTR9,0xFF);

	/* Disable EDID interrupt */
	enable_intr(hw_context, INTR_EDID, 0);
	mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL3
					,BIT_PAGE_5_DISC_CTRL3_FORCE_MHL
					,BIT_PAGE_5_DISC_CTRL3_FORCE_MHL
					);
	mhl_tx_write_reg(hw_context,REG_PAGE_2_TPI_CBUS_START,BIT_PAGE_2_TPI_CBUS_START_GET_EDID_START_0);
	ddc_status = mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);

	mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL3
					,BIT_PAGE_5_DISC_CTRL3_FORCE_MHL
					,0
					);

	mhl_tx_write_reg(hw_context,REG_PAGE_5_CBUS_DISC_INTR0,~BIT_RGND_READY_INT);

	mhl_tx_write_reg(hw_context,REG_PAGE_2_INTR9,0xFF);
	mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_MANUAL,0x00);
	mhl_tx_write_reg(hw_context,REG_080C_HDCP1X_LB_BIST,0x00);

	ddc_status = mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS);

	mhl_tx_modify_reg(hw_context, REG_PAGE_0_LM_DDC,
					BIT_PAGE_0_LM_DDC_SW_TPI_EN,
					VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED);
	if (BIT_PAGE_0_DDC_STATUS_DDC_BUS_LOW & ddc_status) {
		MHL_TX_DBG_WARN("Clearing DDC ack status\n");
		mhl_tx_write_reg(hw_context, REG_PAGE_0_DDC_STATUS,
				ddc_status & ~BIT_PAGE_0_DDC_STATUS_DDC_BUS_LOW);
	}

	mhl_tx_modify_reg(hw_context, REG_PAGE_0_LM_DDC,
					BIT_PAGE_0_LM_DDC_SW_TPI_EN,
					VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED);

}
#endif //)
int si_mhl_tx_drv_set_upstream_edid(struct drv_hw_context *hw_context,
						uint8_t *edid, uint16_t length)
{
	uint8_t reg_val;

	if (MHL_SEND_3D_REQ_OR_FEAT_REQ == hw_context->current_cbus_req.command) {
		struct mhl_dev_context *dev_context;

		dev_context = container_of((void *)hw_context,
				struct mhl_dev_context, drv_context);
		MHL_TX_DBG_WARN("3D_REQ or FEAT_REQ completed\n");
		hw_context->current_cbus_req.command = 0x00;
		si_mhl_tx_msc_command_done(dev_context,0x00);
	}
	reg_val = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
	if (!(BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & reg_val)) {
		//return -1;
		MHL_TX_DBG_INFO("BIT_PAGE_5_CBUS_STATUS_CBUS_HPD isn't asserted\n");
	//return -1;
	}

	MHL_TX_DBG_INFO("presenting EDID upstream\n");

	init_rx_regs(hw_context);

#ifdef SWWA_BZ30759 //(
	/* we are done with the EDID for now.  We now expect to start doing HDCP, which can
		destroy the contents of our EDID buffer, so do another EDID read, which we know will fail,
		but that will reset the DDC fifo in such a way as to leave the buffer contents alone
	*/
	edid_hw_sm_clean_up(hw_context);
#endif //)
	/* choose EDID instead of devcap to appear at the FIFO */
	mhl_tx_write_reg(hw_context
			, REG_PAGE_2_EDID_CTRL
			, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID
			| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
			);

	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_FIFO_ADDR, 0);
	mhl_tx_write_reg_block(hw_context, REG_PAGE_2_EDID_FIFO_WR_DATA, length, edid);

	mhl_tx_write_reg(hw_context
					, REG_PAGE_2_EDID_CTRL
					, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_ENABLE
					| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_EDID
					| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
					| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
					);

	/* Turn on heartbeat polling unconditionally to meet the specs */
	MHL_TX_DBG_WARN("Heartbeat polling is enabled\n");
	mhl_tx_write_reg(hw_context, REG_PAGE_5_MSC_HEARTBEAT_CONTROL, 0xA7);
	/*
	* turn on disconnection based on heartbeat (as well as RSEN) -
	* This is no longer the default behavior.
	* use_heartbeat=1 is required to enable disconnection.
	*/
	if (platform_get_flags() & PLATFORM_FLAG_USE_HEARTBEAT) {
		MHL_TX_DBG_ERR("Disconnection on heartbeat failure is enabled\n");
		mhl_tx_modify_reg(hw_context, REG_PAGE_5_DISC_CTRL1
									, BIT_PAGE_5_DISC_CTRL1_HB_EN
									, BIT_PAGE_5_DISC_CTRL1_HB_EN);
	} else {
		MHL_TX_DBG_ERR("Disconnection on heartbeat failure is disabled\n");
	}

	/* Enable SCDT interrupt to detect stable incoming clock */
	enable_intr(hw_context, INTR_SCDT, BIT_INTR_SCDT_CHANGE);

	/* Disable EDID interrupt */
	enable_intr(hw_context, INTR_EDID, 0);

	/*
		Before exposing the EDID to upstream device, setup to drop all packets.
		This ensures we do not get Packet Overflow interrupt.
		Dropping all packets means we still get the AVIF interrupts which is crucial.
		Packet filters must be disabled until after TMDS is enabled.
	*/
	mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_0, 0xFF);
	mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_1, 0xFF);


#ifndef EARLY_HSIC //(
	if (IN_MHL3_MODE(hw_context)) {
		/*
		 * todo hsic_init configures the transmitter for USB host mode.  So
		 * really this call should be deferred until the driver has negotiated
		 * with the sink to take over the host role.  The call is placed here
		 * just for test purposes
		 */
		hsic_init(hw_context);
	}
#endif //)
	MHL_TX_DBG_ERR("Expose EDID\n");

	/* HPD was held low all this time. Now we send an HPD high */
	reg_val = drive_hpd_high(hw_context,edid, length);

	/* If SCDT is already high, then we will not get an interrupt */
	if (BIT_PAGE_2_TMDS_CSTAT_P3_SCDT & reg_val) {
		extern bool	simulate_scdt;
		if (simulate_scdt) {
			MHL_TX_DBG_INFO("simulate SCDT - no wait: %s0x%02x%s\n"
				,ANSI_ESC_YELLOW_TEXT
				,mhl_tx_read_reg(hw_context,REG_PAGE_2_RX_HDMI_CTRL0)
				,ANSI_ESC_RESET_TEXT);
			int_5_isr(hw_context,BIT_INTR_SCDT_CHANGE);
		}
	}
	return 0;
}

static void hsic_init(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Initialize USB Tunneling\n");

	/* Enable USB flow control and select host mode */
	//mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR0, 0x3);
	mhl_tx_write_reg(hw_context, REG_PAGE_1_FCGC, 0x3);

	/*
	 * strobe_stay_then_reset
	 * bit0: host=1, device=0
	 * bit1: status_en
	 */
	mhl_tx_modify_reg(hw_context, REG_PAGE_1_HRXCTRL3,
			BIT_PAGE_1_HRXCTRL3_HRX_STAY_RESET |
			BIT_PAGE_1_HRXCTRL3_STATUS_EN,
			BIT_PAGE_1_HRXCTRL3_HRX_STAY_RESET |
			BIT_PAGE_1_HRXCTRL3_STATUS_EN);

	/* tdm_tx_num_of_bits_per_symbol bit[2:0]: 4=8-bit */
	mhl_tx_modify_reg(hw_context, REG_PAGE_1_TTXNUMB,
			MSK_PAGE_1_TTXNUMB_TTX_NUMBPS_2_0, 4);

	/* tdm_rx_from_se_coc bit3:doc=0, coc=1 */
	//mhl_tx_modify_reg(hw_context, REG_PAGE_1_TRXSPINUMS, 0x08, 0x08);
	mhl_tx_modify_reg(hw_context, REG_PAGE_1_TRXCTRL, 0x08, 0x08);

	/* hsic_tx_respect_tdm_evn_if_rx_busy bit2:host=0, device=1 */
	//mhl_tx_modify_reg(hw_context, REG_PAGE_1_HTXAFFCTRL, 0x03, 0x00);
	mhl_tx_modify_reg(hw_context, REG_PAGE_1_HTXCTRL, 0x02, 0x00);

	/* keeper_mode bit[1:0]:host=0, device=2 */
	mhl_tx_modify_reg(hw_context, TX_PAGE_1, 0x81, 0x03, 0x00);

	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xFC, 0x00);

	//-- Reset USB Tunneling module: hsic_rx/hsic_tx/hsic_fc/keeper
	mhl_tx_write_reg(hw_context, REG_PAGE_1_UTSRST,
			BIT_PAGE_1_UTSRST_HRX_SRST |
			BIT_PAGE_1_UTSRST_HTX_SRST |
			BIT_PAGE_1_UTSRST_KEEPER_SRST |
			BIT_PAGE_1_UTSRST_FC_SRST);
	mhl_tx_write_reg(hw_context, REG_PAGE_1_UTSRST,
			BIT_PAGE_1_UTSRST_HRX_SRST |
			BIT_PAGE_1_UTSRST_HTX_SRST);

	//Interrupt clear
	// todo Not sure these are necessary.  Need to look into removing
	// them once HSIC is working.
	mhl_tx_write_reg(hw_context, REG_PAGE_1_HRXINTL, 0xFF);		//hsic_rx
	mhl_tx_write_reg(hw_context, REG_PAGE_1_HRXINTH, 0xFF);		//hsic_rx
	mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXINTL, 0xFF);		//tdm_tx
	mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXINTH, 0xFF);		//tdm_tx
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0x63, 0xFF);		//tdm_rx
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0x64, 0xFF);		//tdm_rx
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0x7D, 0xFF);		//hsic_tx
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0x7E, 0xFF);		//hsic_tx
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xEC, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xED, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xEE, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xEF, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xF0, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xF1, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xF2, 0xFF);		//hsic_fc
	mhl_tx_write_reg(hw_context, TX_PAGE_1, 0xF3, 0xFF);		//hsic_fc
}

#define MHL_LOGICAL_DEVICE_MAP (MHL_DEV_LD_AUDIO | MHL_DEV_LD_VIDEO | \
				MHL_DEV_LD_MEDIA | MHL_DEV_LD_GUI)
#define DEVCAP_REG(x) REG_PAGE_4_MHL_DEVCAP_0 | DEVCAP_OFFSET_##x
#define XDEVCAP_REG(x) REG_PAGE_4_MHL_EXTDEVCAP_0 | XDEVCAP_OFFSET(XDEVCAP_ADDR_##x)

// Local devcap values. Populated at init_regs time from si_app_devcap.h
uint8_t dev_cap_values[] = {
	 DEVCAP_VAL_DEV_STATE
	,DEVCAP_VAL_MHL_VERSION
	,DEVCAP_VAL_DEV_CAT
	,DEVCAP_VAL_ADOPTER_ID_H
	,DEVCAP_VAL_ADOPTER_ID_L
	,DEVCAP_VAL_VID_LINK_MODE
	,DEVCAP_VAL_AUD_LINK_MODE
	,DEVCAP_VAL_VIDEO_TYPE
	,DEVCAP_VAL_LOG_DEV_MAP
	,DEVCAP_VAL_BANDWIDTH
	,DEVCAP_VAL_FEATURE_FLAG
	,0
	,0
	,DEVCAP_VAL_SCRATCHPAD_SIZE
	,DEVCAP_VAL_INT_STAT_SIZE
	,DEVCAP_VAL_RESERVED
};
// Local xdevcap values. Populated at init_regs time from si_app_devcap.h
uint8_t xdev_cap_values[] = {
	XDEVCAP_VAL_ECBUS_SPEEDS,
	XDEVCAP_VAL_TMDS_SPEEDS,
	XDEVCAP_VAL_DEV_ROLES,
	XDEVCAP_VAL_LOG_DEV_MAPX
};

static void peer_specific_init(struct drv_hw_context *hw_context)
{
	if (IN_MHL3_MODE(hw_context)) {
		/* Even in MHL3 mode, TPI:1A[0] controls DVI vs. HDMI */
		mhl_tx_write_reg(hw_context
			,REG_PAGE_0_SYS_CTRL1
			,BIT_PAGE_0_SYS_CTRL1_BLOCK_DDC_VIA_UPSTREAM_HPD_LOW
			);
	} else {
		/* disable and clear uninteresting interrupts for MHL 1/2 */
		#ifdef DO_HDCP
		enable_intr(hw_context, INTR_HDCP2, 0x00);
		#endif
		enable_intr(hw_context, INTR_LINK_TRAINING,0x00);
		#ifdef DO_HDCP
		mhl_tx_write_reg(hw_context,REG_PAGE_3_HDCP2X_INTR0,0xFF);
		#endif
		mhl_tx_write_reg(hw_context,REG_PAGE_0_INTR1,0xFF);

		mhl_tx_write_reg(hw_context,REG_PAGE_0_SYS_CTRL1
				,BIT_PAGE_0_SYS_CTRL1_BLOCK_DDC_VIA_UPSTREAM_HPD_LOW
				|BIT_PAGE_0_SYS_CTRL1_TX_CONTROL_HDMI);
	}
}


static int init_regs(struct drv_hw_context *hw_context)
{
	int ret_val = 0;

	MHL_TX_DBG_INFO("called\n");
	peer_specific_init(hw_context);

	/* default values for flags */
	hw_context->video_ready = false;
	hw_context->video_path = 1;

	hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_DVI;
	/* Drop the Hot Plug to upstream and hide EDID */
	drive_hpd_low(hw_context);
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_CTRL, VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE);

	/*
	 * DO NOT enable wake pulses or discovery pulses until successful RGND == 1K
	 * No OTG,
	 */
	/* this is extraneous here, this register is properly set elsewhere */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL9
						, BIT_PAGE_5_DISC_CTRL9_WAKE_DRVFLT
						|BIT_PAGE_5_DISC_CTRL9_WAKE_PULSE_BYPASS
						);

	mhl_tx_write_reg(hw_context, REG_PAGE_2_TMDS0_CCTRL1, 0x90);

	/* Enable TxPLL Clock */
	mhl_tx_write_reg(hw_context, REG_PAGE_2_TMDS_CLK_EN, 0x01);

	/* Enable Tx Clock Path & Equalizer */
	mhl_tx_write_reg(hw_context, REG_PAGE_2_TMDS_CH_EN, 0x11);
	mhl_tx_write_reg(hw_context, REG_PAGE_2_BGR_BIAS, 0x87);

	mhl_tx_write_reg(hw_context, REG_PAGE_2_ALICE0_ZONE_CTRL, 0xE8);	/* 0x024C */
	mhl_tx_write_reg(hw_context, REG_PAGE_2_ALICE0_MODE_CTRL, 0x04);	/* 0x024D */

	/* Enable TPI */
	ret_val = mhl_tx_read_reg(hw_context, REG_PAGE_0_LM_DDC);
	ret_val &= ~BIT_PAGE_0_LM_DDC_SW_TPI_EN;
	ret_val |= VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED;
	mhl_tx_write_reg(hw_context, REG_PAGE_0_LM_DDC, ret_val);

	mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_COPP_DATA2, VAL_PAGE_6_TPI_COPP_PROTLEVEL_MIN);

	/*
	 * TODO: TPI:0xBB has bad default. Need to document in the PR
	 * If this initialization is not changed, we have unstable HDCP
	 * (while testing 9612 as source)
	 */
	mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_HW_OPT3, 0x76);

	/* TMDS should be enabled for both MHL1/2 and MHL3 cases. MHL3 needs it for CoC calibration. */
	mhl_tx_write_reg(hw_context, REG_PAGE_0_TMDS_CCTRL, BIT_PAGE_0_TMDS_CCTRL_TMDS_OE);

	/* set time base for one second to be 60Hz/4*5 + 4*/
	mhl_tx_write_reg(hw_context
			, REG_PAGE_6_TPI_DTD_B2
			, 79
			);

	/* setup local DEVCAP and few more CBUS registers. */

	/*
	 * Fill-in DEVCAP device ID values with those read
	 * from the transmitter.
	 */
	dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_L] =
			(uint8_t)hw_context->chip_device_id;
	dev_cap_values[DEVCAP_OFFSET_DEVICE_ID_H] =
			(uint8_t)(hw_context->chip_device_id >> 8);

	/* Setup local DEVCAP registers */
	mhl_tx_write_reg_block(hw_context, DEVCAP_REG(DEV_STATE),
			ARRAY_SIZE(dev_cap_values), dev_cap_values);

	/*
	 * Adjust XDEVCAP values to match capabilities of the transmitter.
	 */
	xdev_cap_values[XDEVCAP_OFFSET(XDEVCAP_ADDR_ECBUS_SPEEDS)] =
			MHL_XDC_ECBUS_S_075 | MHL_XDC_ECBUS_S_8BIT;
	if (si_mhl_tx_drv_support_e_cbus_d(hw_context))
		xdev_cap_values[XDEVCAP_OFFSET(XDEVCAP_ADDR_ECBUS_SPEEDS)] |=
			MHL_XDC_ECBUS_D_150 | MHL_XDC_ECBUS_D_8BIT;

	mhl_tx_write_reg_block(hw_context, XDEVCAP_REG(ECBUS_SPEEDS),
			ARRAY_SIZE(xdev_cap_values), xdev_cap_values);

	/*
	 * Make sure MDT registers are initialized and the MDT
	 * transmit/receive are both disabled.
	 */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_XMIT_TIMEOUT, 100);

	/* Clear transmit FIFOs and make sure MDT transmit is disabled */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_XMIT_CONTROL, 0x03);

	/* Disable MDT transmit preemptive handshake option */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_XFIFO_STAT, 0x00);

	mhl_tx_write_reg(hw_context, REG_PAGE_5_MDT_RCV_TIMEOUT, 100);

	/* 2013-03-01 bugzilla 27180 */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_CBUS_LINK_CONTROL_8, 0x1D);

	si_mhl_tx_drv_start_gen2_write_burst(hw_context);

	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_CTRL, 0x00);

	/* hearbeat enable/disable to be done once in run time not at every hot plug. */
	enable_heartbeat(hw_context);

	return ret_val;
}

static void si_mhl_tx_drv_start_gen2_write_burst(struct drv_hw_context *hw_context)
{
	enable_intr(hw_context,INTR_G2WB_ERR
		, BIT_MDT_RCV_TIMEOUT | BIT_MDT_RCV_SM_ABORT_PKT_RCVD | BIT_MDT_RCV_SM_ERROR
		| BIT_MDT_XMIT_TIMEOUT | BIT_MDT_XMIT_SM_ABORT_PKT_RCVD | BIT_MDT_XMIT_SM_ERROR
		);
	enable_intr(hw_context, INTR_G2WB, BIT_MDT_XFIFO_EMPTY
									 | BIT_MDT_IDLE_AFTER_HAWB_DISABLE
									 | BIT_MDT_RFIFO_DATA_RDY);

	/* Want HAWB RCV on at all times except when sending a non-WB transaction */
	hw_context->cbus1_state =cbus1_idle_rcv_enabled;
	hw_context->delayed_hawb_enable_reg_val = 0;
	enable_gen2_write_burst_rcv(hw_context);
}

void si_mhl_tx_drv_disable_video_path(struct drv_hw_context *hw_context)
{
	/* If video was already being output */
	if (hw_context->video_ready && (0 == (VAL_PAGE_6_TPI_SC_TPI_AV_MUTE_MUTED &
			 mhl_tx_read_reg(hw_context, REG_PAGE_6_TPI_SC)))) {

		/* stop hdcp and video and remember */
		stop_video(hw_context);
		hw_context->video_path = 0;
	}
}

void si_mhl_tx_drv_enable_video_path(struct drv_hw_context *hw_context)
{
	uint8_t	mask = (BIT_PAGE_6_TPI_SC_REG_TMDS_OE | BIT_PAGE_6_TPI_SC_TPI_AV_MUTE);
	uint8_t	reg;
	MHL_TX_DBG_INFO("called\n");
	/* if a path_en = 0 had stopped the video, restart it unless done already. */
	if (hw_context->video_ready && (0 == hw_context->video_path)) {
		/* remember ds has enabled our path */
		hw_context->video_path = 1;

		reg = mhl_tx_read_reg(hw_context, REG_PAGE_6_TPI_SC);

		if (mask == (mask & reg)) {
			start_video(hw_context);
		}
	}
}

void si_mhl_tx_drv_content_off(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("RAP CONTENT_OFF video %sready\n",hw_context->video_ready?"":"NOT ");
	/* If video was already being output */
	if (hw_context->video_ready && (0 == (VAL_PAGE_6_TPI_SC_TPI_AV_MUTE_MUTED &
			 mhl_tx_read_reg(hw_context, REG_PAGE_6_TPI_SC)))) {

		MHL_TX_DBG_INFO("RAP CONTENT_OFF\n");
		/* stop hdcp and video and remember */
		stop_video(hw_context);
	}
}

void si_mhl_tx_drv_content_on(struct drv_hw_context *hw_context)
{
	uint8_t	mask = (BIT_PAGE_6_TPI_SC_REG_TMDS_OE | BIT_PAGE_6_TPI_SC_TPI_AV_MUTE);
	uint8_t	reg;
	MHL_TX_DBG_INFO("called\n");
	/* if a path_en = 0 had stopped the video, restart it unless done already. */
	if (hw_context->video_ready) {

		reg = mhl_tx_read_reg(hw_context, REG_PAGE_6_TPI_SC);

		if (mask == (mask & reg)) {
			start_video(hw_context);
		}
	}
}

static void unmute_video(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("AV unmuted\n");

	if (IN_MHL3_MODE(hw_context)) {
	} else {
		extern int hdcp_content_type;
		if (hdcp_content_type == 0) {
			if (si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
				mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_SC, VAL_PAGE_6_TPI_SC_TPI_OUTPUT_MODE_0_HDMI);
			} else {
				mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_SC, VAL_PAGE_6_TPI_SC_TPI_OUTPUT_MODE_0_DVI);
			}
		} else {
			MHL_TX_DBG_INFO("HDCP Content Type 1, AV remain muted\n");
		}
	}

	/* Now we can entertain PATH_EN */
	hw_context->video_ready = 1;
}
static uint8_t reset_ddc_8620(struct drv_hw_context *hw_context)
{
	uint32_t count = 0;
	uint8_t ddc_status;
	int edid_ctrl;
	edid_ctrl = mhl_tx_read_reg(hw_context,REG_PAGE_2_EDID_CTRL);
	/* disable EDID reading module during DDC reset procedure */
	mhl_tx_write_reg(hw_context,REG_PAGE_2_EDID_CTRL,edid_ctrl &0xFE);
	/* 31664 -Must clear DDC on disconnection to free the channel if EDID read automation was in progress */
	//si_mhl_tx_drv_reset_ddc_fifo(hw_context);
	do {
		count++;

		/* TODO: Lei suggested to read bit 6 as well. Review this. */
		ddc_status = (mhl_tx_read_reg(hw_context, REG_PAGE_0_DDC_STATUS) & 0x5F);

		MHL_TX_DBG_ERR("Clear DDC FIFO #%d\n",count);

		mhl_tx_modify_reg(hw_context, REG_PAGE_0_LM_DDC,
					BIT_PAGE_0_LM_DDC_SW_TPI_EN,
					VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED);
		mhl_tx_modify_reg(hw_context, REG_PAGE_0_DDC_CMD,
					MSK_PAGE_0_DDC_CMD_DDC_CMD,
					VAL_PAGE_0_DDC_CMD_DDC_CMD_ABORT);
		mhl_tx_modify_reg(hw_context, REG_PAGE_0_DDC_CMD,
					MSK_PAGE_0_DDC_CMD_DDC_CMD,
					VAL_PAGE_0_DDC_CMD_DDC_CMD_CLEAR_FIFO);
		mhl_tx_write_reg(hw_context,REG_PAGE_0_DDC_STATUS,BIT_PAGE_0_DDC_STATUS_DDC_FIFO_EMPTY);
		mhl_tx_modify_reg(hw_context, REG_PAGE_0_LM_DDC,
					BIT_PAGE_0_LM_DDC_SW_TPI_EN,
					VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED);
		if (count >= 256) break;
	} while (ddc_status != BIT_PAGE_0_DDC_STATUS_DDC_FIFO_EMPTY);
	mhl_tx_write_reg(hw_context,REG_PAGE_2_EDID_CTRL,edid_ctrl);
	return ddc_status;
}

/*
	Sequence of operations in this function is important.
	1. Turn off HDCP interrupt
	2. Turn of TMDS output
	3. Turn off HDCP engine/encryption
	4. Clear any leftover HDCP interrupt
*/
static void stop_video(struct drv_hw_context *hw_context)
{
	if (IN_MHL3_MODE(hw_context)) {
#ifdef DO_HDCP
		uint8_t ddcm_status;
#endif
		MHL_TX_DBG_WARN("for >= MHL3.0\n");

		/* disable TMDS early */
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xC0);		/* 0x0331 */

		/* reset H2M here */
		mhl_tx_write_reg(hw_context,REG_PAGE_3_M3_CTRL
					, VAL_PAGE_3_M3_CTRL_MHL3_VALUE
					| BIT_PAGE_3_M3_CTRL_H2M_SWRST);
#ifdef DO_HDCP
		// Disable HDCP 2.2
		enable_intr(hw_context, INTR_HDCP2, 0x00);

		// Disable HDCP2 DDC polling
		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_POLL_CS, 0x71);		// 0x0391

		{
			int count=0;
			while (0 <= (ddcm_status = mhl_tx_read_reg(hw_context,REG_PAGE_3_HDCP2X_DDCM_STS))) {
				if ( 0 == (MSK_PAGE_3_HDCP2X_DDCM_STS_HDCP2X_DDCM_CTL_CS_3_0 & ddcm_status)) {
					break;
				}
				if (++count > 256) {
					break;
				}
				MHL_TX_DBG_WARN("shutting down HDCP\n");
			}
		}

		/* disable encryption */
		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_CTRL_0, 0x82);	// 0x03A0

		mhl_tx_modify_reg(hw_context, REG_PAGE_3_M3_P0CTRL,
									BIT_PAGE_3_M3_P0CTRL_MHL3_P0_HDCP_EN,
									0x00);

		MHL_TX_DBG_INFO("HDCP2 Off; Last HDCP2X_DDCM Status %02X;\n", ddcm_status);

		reset_ddc_8620(hw_context);
		/* clear any leftover hdcp2 interrupts */
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP2].stat_page, g_intr_tbl[INTR_HDCP2].stat_offset, 0xff);
#else
		reset_ddc_8620(hw_context);
#endif
	} else {

		// MHL 2/1 with HDCP 1.x
		MHL_TX_DBG_WARN(" for MHL1/2.x\n");
#ifdef DO_HDCP
		/* Turn off HDCP interrupt */
		enable_intr(hw_context, INTR_HDCP, 0x00);
		enable_intr(hw_context, INTR_HDCP2, 0x00);

		/* stop hdcp engine */
		mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_COPP_DATA2, 0);

		/* clear any leftover hdcp interrupt */
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);

		MHL_TX_DBG_WARN("hw_context = %x; ->intr_info = %x\n", hw_context, hw_context->intr_info);
#endif
		if (NULL != hw_context->intr_info) {
			if (NULL != hw_context->intr_info->edid_parser_context) {
			/* We must maintain the output bit (bit 0) to allow just one bit change
			 * later when BKSV read is triggered. This programming is required for HDCP CTS 1.x to pass */
			if (si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {

				mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_SC
					, VAL_PAGE_6_TPI_SC_REG_TMDS_OE_POWER_DOWN
					| VAL_PAGE_6_TPI_SC_TPI_AV_MUTE_MUTED
					| VAL_PAGE_6_TPI_SC_TPI_OUTPUT_MODE_0_HDMI
					);
			} else {
				mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_SC
					, VAL_PAGE_6_TPI_SC_REG_TMDS_OE_POWER_DOWN
					| VAL_PAGE_6_TPI_SC_TPI_AV_MUTE_MUTED
					| VAL_PAGE_6_TPI_SC_TPI_OUTPUT_MODE_0_DVI
					);
				}
			}
		}
	}
}
#ifdef DO_HDCP
void si_mhl_tx_drv_start_cp(struct drv_hw_context *hw_context) {
	start_hdcp(hw_context);
}

static void start_hdcp(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_INFO("Start HDCP Authentication\n");

	if (IN_MHL3_MODE(hw_context)) {

		start_hdcp_content_type(hw_context);

		// Unmask HDCP2 INTs
		enable_intr(hw_context, INTR_HDCP2, BIT_HDCP2_INTR_AUTH_DONE |
											BIT_HDCP2_INTR_AUTH_FAIL |
											BIT_HDCP2_INTR_RPTR_RCVID_CHANGE);

		// Enable HDCP 2.2
		mhl_tx_modify_reg(hw_context, REG_PAGE_3_M3_P0CTRL
								, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_HDCP_EN
								, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_HDCP_EN);

		/* enable encryption */
		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_CTRL_0, 0x83);	// 0x03A0

		// Enable HDCP2 DDC polling
		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_POLL_CS, 0x70);		// 0x0391

		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_RPT_SMNG_K, 1);

		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_CTRL_1, 0x01);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_CTRL_1, 0x00);
	} else {
		/* First stop hdcp and video */
		stop_video(hw_context);
		/* Ensure we get HDCP interrupts now onwards. Clear interrupt first. */
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page, g_intr_tbl[INTR_HDCP].stat_offset, 0xff);

		if (get_cbus_connection_status(hw_context)) {
			/* Enable HDCP interrupt */
			enable_intr(hw_context, INTR_HDCP,
					(BIT_PAGE_6_TPI_INTR_ST0_TPI_AUTH_CHNGE_STAT
					| BIT_PAGE_6_TPI_INTR_ST0_TPI_COPP_CHNGE_STAT
					| BIT_PAGE_6_TPI_INTR_ST0_READ_BKSV_BCAPS_DONE_STAT
					| BIT_PAGE_6_TPI_INTR_ST0_READ_BKSV_ERR_STAT
					));

			msleep(250);
			/*
			 * Chip requires only bit 4 to change for BKSV read
			 * No other bit should change.
			 */
			mhl_tx_modify_reg(hw_context, REG_PAGE_6_TPI_SC,
						BIT_PAGE_6_TPI_SC_REG_TMDS_OE,
						VAL_PAGE_6_TPI_SC_REG_TMDS_OE_ACTIVE);
		}
	}
}

static void start_hdcp_content_type(struct drv_hw_context *hw_context)
{
	uint8_t misc_ctrl;
	uint8_t index;
	uint8_t msg[2] = { 0x01, 0x00 }; /* One Stream */

	extern int hdcp_content_type;
	if (hdcp_content_type == 1) {
		msg[1] = 0x01;
	}

	MHL_TX_DBG_INFO("HDCP Content Type = %d\n", msg[1]);

	misc_ctrl = mhl_tx_read_reg(hw_context, REG_PAGE_3_HDCP2X_MISC_CTRL);

    /*
     * Toggle SMNG_WR_START
     */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_MISC_CTRL, misc_ctrl |
                                 BIT_PAGE_3_HDCP2X_MISC_CTRL_HDCP2X_RPT_SMNG_WR_START);
    mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_MISC_CTRL, misc_ctrl);

    // Write message
    for (index = 0; index < 2; index++) {
    	mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_RPT_SMNG_IN,
    			                     msg[index]);

        mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_MISC_CTRL, misc_ctrl |
                                     BIT_PAGE_3_HDCP2X_MISC_CTRL_HDCP2X_RPT_SMNG_WR);
        mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_MISC_CTRL, misc_ctrl);
    }
}
#endif
/*
 * start_video
 *
 *
 */
static int start_video(struct drv_hw_context *hw_context)
{
	struct mhl_dev_context	*dev_context;
	dev_context = get_mhl_device_context(hw_context);
	MHL_TX_DBG_WARN("called.. \n");
		
	if ( ! IN_MHL3_MODE(hw_context)) {
		/*
		 * stop hdcp and video
		 * this kills any hdcp thread going on already
		 */
		stop_video(hw_context);
	}

	/*
	 * if path has been disabled by PATH_EN = 0 return with error;
	 * When enabled, this function will be called again.
	 * if downstream connection has been lost (CLR_HPD), return with error.
	 */
	if ((0 == hw_context->video_path)
			|| (0 == get_cbus_connection_status(hw_context))
			|| (false == dev_context->misc_flags.flags.rap_content_on)
		) {
		MHL_TX_DBG_WARN("(0 == hw_context->video_path)	|| (0 == get_cbus_connection_status(hw_context))|| (false == dev_context->misc_flags.flags.rap_content_on)\n");
		//return false;
	}

	if (si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
		/*
		 * setup registers for packed pixel, 3D, colors and AVIF
		 * using incoming info frames. VSIF sent out by the h/w.
		 */
		mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_HDMI
				| VAL_PAGE_2_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);
		if (false == set_hdmi_params(dev_context)) {
			/* Do not disrupt an ongoing video for bad incoming infoframes */
			return false;
		}

		if (IN_MHL3_MODE(hw_context)) {
			MHL_TX_DBG_INFO("IN_MHL3_MODE(hw_context)=true\n");
			/* Assert H2M reset */
			mhl_tx_write_reg(hw_context,REG_PAGE_3_M3_CTRL
						, VAL_PAGE_3_M3_CTRL_MHL3_VALUE
						| BIT_PAGE_3_M3_CTRL_H2M_SWRST);

			/* Deassert H2M reset */
			mhl_tx_write_reg(hw_context,REG_PAGE_3_M3_CTRL
						, VAL_PAGE_3_M3_CTRL_MHL3_VALUE);
		} else {
			MHL_TX_DBG_INFO("IN_MHL3_MODE(hw_context)=false\n");
			set_auto_zone_for_mhl_1_2(hw_context);
			#ifdef DO_HDCP
			start_hdcp(hw_context);
			#else
			unmute_video(hw_context);
			#endif
		}
		/*
		 * TODO: Check if the following block has to be performed again at
		 * HDCP restart.
		 *
		 * Start sending out AVIF now.
		 * Also ensure other appropriate frames are being forwarded or dropped.
		 */
		/*
		 * Send infoframe out
		 */
		mhl_tx_write_reg_block(hw_context, REG_PAGE_6_TPI_AVI_CHSUM,
				sizeof(hw_context->outgoingAviPayLoad.ifData),
				(uint8_t*)&hw_context->outgoingAviPayLoad.ifData);
		/*
		 * Change packet filters to drop AIF and GCP.
		 * TODO: Describe this in the PR appropriately.
		 * TODO: for MHL3 mode, drop VSI packets and convert HDMI VSIF to MHL3 VSIF.
		 */
		mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_0, 0xA5);
			// Value written does not match comment above per values below
			// BIT_PAGE_2_PKT_FILTER_0_DROP_CEA_GAMUT_PKT  (0x80)
			// BIT_PAGE_2_PKT_FILTER_0_DROP_MPEG_PKT       (0x20)
			// BIT_PAGE_2_PKT_FILTER_0_DROP_AVI_PKT        (0x04)
			// BIT_PAGE_2_PKT_FILTER_0_DROP_GCP_PKT        (0x01)

		if (IN_MHL3_MODE(hw_context)) {
			enum info_sel_e {
				info_sel_avi=0
				,info_sel_spd
				,info_sel_audio
				,info_sel_mpeg
				,info_sel_generic
				,info_sel_generic2
				,info_sel_vsi
				,info_sel_reserved
			};
			uint8_t vsif_buffer[31];
			/*
			 * disable HDMI to MHL VSIF conversion (do it in software) (set bit 7)
			 * and drop generic infoframes (set bit 1)
			 */
			mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_1,
							BIT_PAGE_2_PKT_FILTER_1_VSI_OVERRIDE_DIS
							| BIT_PAGE_2_PKT_FILTER_1_DROP_GEN_PKT
							| BIT_PAGE_2_PKT_FILTER_1_DROP_VSIF_PKT);

			mhl_tx_write_reg(hw_context,REG_PAGE_6_TPI_INFO_FSEL
							,BIT_PAGE_6_TPI_INFO_FSEL_TPI_INFO_EN
							|BIT_PAGE_6_TPI_INFO_FSEL_TPI_INFO_RPT
							| info_sel_vsi);

			/* hardware takes effect on the write to TPI_INFO_B30,
				and checksum is calculated on just the first part,
				so pad the remainder of the buffer
			*/
			memset(vsif_buffer,0,sizeof(vsif_buffer));
			if (hw_context->callbacks.hpd_driven_high) {
				switch (hw_context->hpd_high_callback_status) {
				case hpd_high_format_HDMI_vsif_mhl3:
				case hpd_high_format_HDMI_vsif_mhl3_hdcp_on:
				case hpd_high_format_HDMI_vsif_mhl3_not_repeatable:
				case hpd_high_format_HDMI_vsif_mhl3_hdcp_on_not_repeatable:
					memcpy(vsif_buffer
						,(uint8_t *)&hw_context->vsif_mhl3_or_hdmi_from_callback.mhl3_vsif
						,sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback.mhl3_vsif));
					break;
				default:
					memcpy(vsif_buffer,(uint8_t *)&hw_context->outgoing_mhl3_vsif
								,sizeof(hw_context->outgoing_mhl3_vsif));
				}
			} else {
				memcpy(vsif_buffer,(uint8_t *)&hw_context->outgoing_mhl3_vsif
								,sizeof(hw_context->outgoing_mhl3_vsif));
			}
			mhl_tx_write_reg_block(hw_context,REG_PAGE_6_TPI_INFO_B0
								,sizeof(vsif_buffer)
								,(uint8_t *)vsif_buffer);
		} else {
			/*
			 * enable HDMI to MHL VSIF conversion (clear bit 7)
			 * and drop generic infoframes (set bit 1)
			 */
			mhl_tx_write_reg(hw_context, REG_PAGE_2_PKT_FILTER_1,
							BIT_PAGE_2_PKT_FILTER_1_DROP_GEN_PKT);
		}
	} else {
		/* For DVI, output video w/o infoframe; No video settings are changed. */
		mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval = REG_RX_HDMI_CTRL2_DEFVAL_DVI
				| VAL_PAGE_2_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);
		#ifdef DO_HDCP
		MHL_TX_DBG_ERR("DVI - Start HDCP\n");
		start_hdcp(hw_context);
		#else
		unmute_video(hw_context);
		#endif

	}

	return true;
}
#ifdef DO_HDCP
static int hdcp_isr(struct drv_hw_context *hw_context, uint8_t tpi_int_status)
{
	uint8_t query_data;

	query_data = mhl_tx_read_reg(hw_context, REG_PAGE_6_TPI_COPP_DATA1);
	MHL_TX_DBG_INFO("R3D= %02x R29= %02x\n", tpi_int_status, query_data);

	if (BIT_PAGE_6_TPI_INTR_ST0_READ_BKSV_BCAPS_DONE_STAT & tpi_int_status) {

		if (BIT_PAGE_6_TPI_COPP_DATA1_COPP_PROTYPE & query_data) {

			/*
				If the downstream device is a repeater, enforce a 5 second delay
				to pass HDCP CTS 1B-03.
				TODO: Describe this in the PR.
				2013-12-18 Allow the enforcement to exit early if the KSV_FIFO_LAST bit gets set,
					which, in turn, implies that BCAPS[5] (KSV_FIFO_RDY) is set.
					This fixes the occasional "Inactivity Timer Expired"/"Not Judged" result
					in 1B-02.
			*/
			if (BIT_PAGE_6_TPI_COPP_DATA1_COPP_HDCP_REP ==
				(BIT_PAGE_6_TPI_COPP_DATA1_COPP_HDCP_REP & query_data)) {
				struct timeval ksv_poll_start;
				struct timeval current_time,elapsed_time;
				do_gettimeofday(&ksv_poll_start);
				do{
					uint8_t dbg_regs[7];
					int cbus_connected_state;
					if (BIT_PAGE_6_TPI_KSV_FIFO_STAT_KSV_FIFO_LAST & mhl_tx_read_reg(hw_context,REG_PAGE_6_TPI_KSV_FIFO_STAT)){
						break;
					}
					cbus_connected_state = mhl_tx_read_reg(hw_context,REG_PAGE_5_CBUS_STATUS);
					cbus_connected_state &= (BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED | BIT_PAGE_5_CBUS_STATUS_CBUS_HPD);
					if ((BIT_PAGE_5_CBUS_STATUS_CBUS_CONNECTED | BIT_PAGE_5_CBUS_STATUS_CBUS_HPD) != cbus_connected_state){
						break;
					}
					
					mhl_tx_read_reg_block(hw_context,REG_PAGE_6_TPI_HW_DBG1,sizeof(dbg_regs),dbg_regs);

					msleep(50);
					do_gettimeofday(&current_time);
					elapsed_time.tv_usec = current_time.tv_usec - ksv_poll_start.tv_usec;
					elapsed_time.tv_sec = current_time.tv_sec - ksv_poll_start.tv_sec;
					if (elapsed_time.tv_usec < 0){
						/* handle the borrow case */
						elapsed_time.tv_usec += 1000000;
						elapsed_time.tv_sec  -=1;
					}
				}while (elapsed_time.tv_sec < 3);
			}

			// Start authentication here
			mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_COPP_DATA2,
					VAL_PAGE_6_TPI_COPP_PROTLEVEL_MAX);
		}
	} else if (BIT_PAGE_6_TPI_INTR_ST0_READ_BKSV_ERR_STAT & tpi_int_status) {
		MHL_TX_DBG_ERR("BKSV ERROR - Start HDCP\n");
		start_hdcp(hw_context);
	} else if (BIT_PAGE_6_TPI_INTR_ST0_TPI_COPP_CHNGE_STAT & tpi_int_status) {
		int link_status;

		link_status = query_data & MSK_PAGE_6_TPI_COPP_DATA1_COPP_LINK_STATUS;

		switch (link_status) {
			case VAL_TPI_COPP_LINK_STATUS_NORMAL:
				unmute_video(hw_context);
				break;

			case VAL_TPI_COPP_LINK_STATUS_LINK_LOST:
				MHL_TX_DBG_ERR("LINK LOST - Start HDCP\n");
				start_hdcp(hw_context);
				break;
			case VAL_TPI_COPP_LINK_STATUS_RENEGOTIATION_REQ:
				MHL_TX_DBG_INFO("tpi BSTATUS2: 0x%x\n"
						, mhl_tx_read_reg(hw_context, REG_PAGE_6_TPI_BSTATUS2)
						);
				/* Disabling TMDS output here will disturb the clock */
				mhl_tx_modify_reg(hw_context
						, REG_PAGE_6_TPI_SC
						, BIT_PAGE_6_TPI_SC_TPI_AV_MUTE
						, VAL_PAGE_6_TPI_SC_TPI_AV_MUTE_MUTED);

				/* send TPI hardware to HDCP_Prep state */
				mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_COPP_DATA2, 0);
				break;
			case VAL_TPI_COPP_LINK_STATUS_LINK_SUSPENDED:
				MHL_TX_DBG_ERR("LINK suspended - Start HDCP\n");
				start_hdcp(hw_context);
				break;
		}
	} else if (BIT_PAGE_6_TPI_INTR_ST0_TPI_AUTH_CHNGE_STAT & tpi_int_status) {
		uint8_t new_link_prot_level;

		new_link_prot_level = (uint8_t)
				(query_data & (BIT_PAGE_6_TPI_COPP_DATA1_COPP_GPROT |
							BIT_PAGE_6_TPI_COPP_DATA1_COPP_LPROT));

		switch (new_link_prot_level) {
			case (VAL_TPI_COPP_GPROT_NONE | VAL_TPI_COPP_LPROT_NONE):
				MHL_TX_DBG_ERR("?PROT_NONE - Start HDCP\n");
				start_hdcp(hw_context);
				break;

			case VAL_TPI_COPP_GPROT_SECURE:
			case VAL_TPI_COPP_LPROT_SECURE:
			case (VAL_TPI_COPP_GPROT_SECURE | VAL_TPI_COPP_LPROT_SECURE):
				unmute_video(hw_context);
				break;
		}
	}
	return 0;
}

static int hdcp2_isr(struct drv_hw_context *hw_context, uint8_t intr_status)
{
	uint8_t rcvr_info[3];
	uint8_t rcvr_id_list[5];

	if (intr_status & BIT_HDCP2_INTR_AUTH_DONE) {
		MHL_TX_DBG_ERR("HDCP2.2 Authentication Done.\n");
		unmute_video(hw_context);
		// Enable high-value content / disable mute
	}

	if (intr_status & BIT_HDCP2_INTR_AUTH_FAIL) {
		uint8_t ro_gp0;
		uint8_t ro_auth[2];

		// Disable high-value content / enable mute

		ro_gp0 = mhl_tx_read_reg(hw_context, REG_PAGE_3_HDCP2X_GP_OUT0);
		mhl_tx_read_reg_block(hw_context, REG_PAGE_3_HDCP2X_AUTH_STAT, sizeof(ro_auth), ro_auth);

		MHL_TX_DBG_ERR("HDCP2.2 Authentication Failed\n\tgp0 %02X, status %02X %02X\n", ro_gp0, ro_auth[0], ro_auth[1]);
	}

	if (intr_status & BIT_HDCP2_INTR_RPTR_RCVID_CHANGE) {
		MHL_TX_DBG_ERR("HDCP2.2 RCV_ID Changed.\n");

		// Read RCVR INFO and RCVR ID LIST
		mhl_tx_read_reg_block(hw_context, REG_PAGE_3_HDCP2X_RPT_RCVID_OUT, sizeof(rcvr_info), rcvr_info);
		mhl_tx_read_reg_block(hw_context, REG_PAGE_3_HDCP2X_RPT_RCVR_ID0, sizeof(rcvr_id_list), rcvr_id_list);
	}

	mhl_tx_write_reg(hw_context, REG_PAGE_3_HDCP2X_INTR0, intr_status);

	return intr_status;
}
#endif
static int int_8_isr(struct drv_hw_context *hw_context, uint8_t intr_8_status)
{
	vendor_specific_info_frame_t	vsif;
	avi_info_frame_t				avif;

	/* Clear interrupt status immediately */
	mhl_tx_write_reg(hw_context, REG_PAGE_0_INTR8, intr_8_status);

	if (hw_context->hpd_high_callback_status >= 0) {
		/* Video is already started.
			Set this flag so that we catch mode changes
		*/

		hw_context->hpd_high_callback_status = hpd_high_video_not_ready;
		return intr_8_status;
	}

	/* Resolution change interrupt (NEW_AVIF or NEW_VSIF) */
	if (BIT_INTR8_CEA_NEW_VSI & intr_8_status) {
		MHL_TX_DBG_ERR("got NEW_VSI\n");

		mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval
				| VAL_PAGE_2_RX_HDMI_CTRL2_VSI_MON_SEL_VSI);

		mhl_tx_read_reg_block(hw_context
				, REG_PAGE_2_RX_HDMI_MON_PKT_HEADER1
				, sizeof(vsif)
				, (uint8_t *)&vsif
				);
	}

	if (BIT_INTR8_CEA_NEW_AVI & intr_8_status) {

		MHL_TX_DBG_ERR("got NEW_AVIF\n");

		/* Read AVIF packet */

		mhl_tx_write_reg(hw_context, REG_PAGE_2_RX_HDMI_CTRL2
				, hw_context->rx_hdmi_ctrl2_defval
				| VAL_PAGE_2_RX_HDMI_CTRL2_VSI_MON_SEL_AVI);

		mhl_tx_read_reg_block(hw_context
					, REG_PAGE_2_RX_HDMI_MON_PKT_HEADER1
					, sizeof(avif)
					, (uint8_t *)&avif
					);

		if (0 == avif.header.type_code) {
			MHL_TX_DBG_ERR("bogus AVIF\n");
			return intr_8_status;
		}
	}

	switch (intr_8_status & (BIT_INTR8_CEA_NEW_VSI | BIT_INTR8_CEA_NEW_AVI)) {
	case BIT_INTR8_CEA_NEW_VSI:
		process_info_frame_change(hw_context, &vsif, NULL);
		break;
	case BIT_INTR8_CEA_NEW_AVI:
		process_info_frame_change(hw_context, NULL, &avif);
		break;
	case (BIT_INTR8_CEA_NEW_VSI | BIT_INTR8_CEA_NEW_AVI):
		process_info_frame_change(hw_context, &vsif, &avif);
		break;
	}

	return intr_8_status;
}

static int int_3_isr(struct drv_hw_context *hw_context, uint8_t int_3_status)
{
	if (BIT_PAGE_0_INTR3_DDC_CMD_DONE & int_3_status){
		/* Inform MHL module of manual EDID read completion */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
		hw_context->intr_info->msc_done_data =0;
		/* ensure that no other DDC interrupts occur */
		enable_intr(hw_context,INTR_DDC,0);
	}
	return 0;
}
static int int_9_isr(struct drv_hw_context *hw_context, uint8_t int_9_status)
{
	if (int_9_status)
	{
		mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_EDID].stat_page, g_intr_tbl[INTR_EDID].stat_offset, int_9_status);
		if (BIT_INTR9_DEVCAP_DONE & int_9_status) {
			hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
			hw_context->intr_info->msc_done_data = 0;
		}

		if (BIT_INTR9_EDID_DONE & int_9_status) {
			int ddcStatus;
			ddcStatus = mhl_tx_read_reg(hw_context,REG_PAGE_0_DDC_STATUS);
#ifdef SWWA_BZ30759 //(
			mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL5
					,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE
					,0);
#endif //)
			if (BIT_PAGE_0_DDC_STATUS_DDC_NO_ACK & ddcStatus) {

				/* Restart EDID read from block 0 */
				if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
									hw_context->current_edid_request_block=0)) {
						hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
						hw_context->intr_info->msc_done_data =1;
						MHL_TX_DBG_ERR("%sedid request problem%s\n"
							,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
				}
			} else {
				int num_extensions;

				MHL_TX_DBG_INFO("EDID block read complete\n");
				num_extensions =si_mhl_tx_get_num_cea_861_extensions(
								hw_context->intr_info->edid_parser_context,
								hw_context->current_edid_request_block);
				if (num_extensions < 0)	{
					MHL_TX_DBG_ERR("edid problem:%d\n",num_extensions);
					if (ne_NO_HPD == num_extensions) {
						// no HPD, so start over
						hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
						hw_context->intr_info->msc_done_data =1;
							MHL_TX_DBG_ERR("%shpd low%s\n"
								,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
					} else {
						/* Restart EDID read from block 0 */
						if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
								hw_context->current_edid_request_block=0)) {
							// Notify the component layer with error
							hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
							hw_context->intr_info->msc_done_data =1;
							MHL_TX_DBG_ERR("%sedid request problem%s\n"
								,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
						}
					}
				} else if (hw_context->current_edid_request_block < num_extensions) {
					/* EDID read next block */
					if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
							++hw_context->current_edid_request_block)) {
							/* Notify the MHL module with error */
							hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
							hw_context->intr_info->msc_done_data =1;
							MHL_TX_DBG_ERR("%sedid request problem%s\n"
								,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
					}
				} else {
					/* Inform MHL module of EDID read MSC command completion */
					hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
					hw_context->intr_info->msc_done_data =0;
				}
			}
		}

		if (BIT_INTR9_EDID_ERROR & int_9_status) {
			uint8_t cbus_status;
			/*
			 * un-comment next two lines for showing TPI debug
			 * registers when readblock is done.
			uint8_t debugStatus[7];
			mhl_tx_read_reg_block(REG_TPI_HW_DBG1,sizeof(debugStatus), debugStatus);
			 */
			cbus_status = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);

			MHL_TX_DBG_INFO("EDID read error, retrying\n");

			hw_context->edid_fifo_block_number = 0;
			hw_context->current_edid_request_block = 0;

#ifdef SWWA_BZ30759 //(
			mhl_tx_modify_reg(hw_context,REG_PAGE_5_DISC_CTRL5
					,BIT_PAGE_5_DISC_CTRL5_DSM_OVRIDE
					,0);
#endif //)
			if (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & cbus_status) {
				/* Restart EDID read from block 0 */
				if (!si_mhl_tx_drv_issue_edid_read_request(hw_context,
						hw_context->current_edid_request_block=0)) {
					/* Notify the MHL module of error */
					hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
					hw_context->intr_info->msc_done_data =1;
				}
			} else {
				/* Notify the MHL module with error */
				hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
				hw_context->intr_info->msc_done_data =1;
			}
		}
	}
	return int_9_status;
}

void si_mhl_tx_read_devcap_fifo(struct drv_hw_context *hw_context,
							MHLDevCap_u *dev_cap_buf)
{
	MHL_TX_DBG_INFO("called\n");

	/* Enable EDID interrupt */
	enable_intr(hw_context, INTR_EDID,
				(BIT_INTR9_DEVCAP_DONE_MASK
				| BIT_INTR9_EDID_DONE_MASK
				| BIT_INTR9_EDID_ERROR
				));

	/* choose devcap instead of EDID to appear at the FIFO */
	mhl_tx_write_reg(hw_context
			, REG_PAGE_2_EDID_CTRL
			, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_DEVCAP
			| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
			);
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_FIFO_ADDR, 0);

	/* Retrieve the DEVCAP register values from the FIFO */
	mhl_tx_read_reg_block(hw_context, REG_PAGE_2_EDID_FIFO_RD_DATA,
						DEVCAP_SIZE, dev_cap_buf->devcap_cache);
	MHL_TX_DBG_INFO("%sgot DEVCAP%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
	MHL_TX_DBG_INFO("%sgot DEVCAP%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
}

void si_mhl_tx_read_xdevcap_fifo(struct drv_hw_context *hw_context,
							MHLXDevCap_u *xdev_cap_buf)
{
	MHL_TX_DBG_INFO("called\n");

	/* choose XDEVCAP instead of EDID to appear at the FIFO */
	mhl_tx_write_reg(hw_context
			, REG_PAGE_2_EDID_CTRL
			, VAL_PAGE_2_EDID_CTRL_EDID_PRIME_VALID_DISABLE
			| BIT_PAGE_2_EDID_CTRL_XDEVCAP_EN
			| VAL_PAGE_2_EDID_CTRL_DEVCAP_SELECT_DEVCAP
			| VAL_PAGE_2_EDID_CTRL_EDID_FIFO_ADDR_AUTO_ENABLE
			| VAL_PAGE_2_EDID_CTRL_EDID_MODE_EN_ENABLE
			);
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_FIFO_ADDR, 0);

	/* Retrieve the XDEVCAP register values from the FIFO */
	mhl_tx_read_reg_block(hw_context, REG_PAGE_2_EDID_FIFO_RD_DATA,
						XDEVCAP_OFFSET(XDEVCAP_LIMIT), xdev_cap_buf->xdevcap_cache);

	MHL_TX_DBG_INFO("%sgot XDEVCAP%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
}

static int get_cbus_connection_status(struct drv_hw_context *hw_context)
{
	return (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS));
}

static int mhl_cbus_err_isr(struct drv_hw_context *hw_context,
							uint8_t cbus_err_int)
{
	int ret_val = 0;
	uint8_t msc_abort_reason = 0;

	/*
	 * Three possible errors could be asserted.
	 * 94[2] = DDC_ABORT
	 * 94[3] = ABORT while receiving a command - MSC_RCV_ERROR
	 * 			The ABORT reasons are in 9A
	 * 94[6] = ABORT while sending a command - MSC_SEND_ERROR
	 * 			The ABORT reasons are in 9C
	 */
#ifdef PRINT_DDC_ABORTS
	if (cbus_err_int & BIT_CBUS_DDC_ABORT) {
		uint8_t ddc_abort_reason = 0;
		/*
		 * For DDC ABORTs, options are
		 * 1. reset DDC block. This will hamper a broken HDCP or EDID.
		 * 2. if error persists, reset the chip. In any case video is not
		 *    working. So it should not cause blinks.
		 * In either case, call an API to let SoC know what happened.
		 *
		 * Only option 1 has been implemented here.
		 */

		ddc_abort_reason = mhl_tx_read_reg(hw_context,
											REG_PAGE_5_DDC_ABORT_INT);

		MHL_TX_DBG_INFO("CBUS DDC ABORT. Reason = %02X\n",
						ddc_abort_reason);

		if (DDC_ABORT_THRESHOLD < ++ddc_abort_count) {
			ddc_abort_count = 0;
		} else if (MHL_READ_EDID_BLOCK == hw_context->current_cbus_req.command) {
			hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
			// setting 1 indicates there was an error
			hw_context->intr_info->msc_done_data =1;
		}
	}
#endif	//	PRINT_DDC_ABORTS
	if (cbus_err_int & BIT_CBUS_MSC_ABORT_RCVD) {
		/*
		 * For MSC Receive time ABORTs
		 * - Defer submission of new commands by 2 seconds per MHL specs
		 * - This is not even worth reporting to SoC. Action is in the hands of peer.
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context,
								REG_PAGE_5_MSC_MR_ABORT_INT);

		++msc_abort_count;

		MHL_TX_DBG_ERR("#%d: ABORT during MSC RCV. Reason = %02X\n",
				msc_abort_count, msc_abort_reason);
	}
	if (cbus_err_int & BIT_CBUS_CMD_ABORT) {
		/*
		 * 1. Defer submission of new commands by 2 seconds per MHL specs
		 * 2. For API operations such as RCP/UCP etc., report the situation to
		 *    SoC and let the decision be there. Internal retries have been already
		 *    done.
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CBUS_ABORT;

		msc_abort_reason = mhl_tx_read_reg(hw_context,
								REG_PAGE_5_MSC_MT_ABORT_INT);

		MHL_TX_DBG_ERR("CBUS ABORT during MSC SEND. Reason = %02X\n",
						msc_abort_reason);

		mhl_tx_write_reg(hw_context,REG_PAGE_5_MSC_MT_ABORT_INT,msc_abort_reason);
	}
	/*
	 * Print the reason for information
	 */
	if (msc_abort_reason) {
		if (BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL & msc_abort_reason) {
				MHL_TX_DBG_ERR("Retry threshold exceeded\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR & msc_abort_reason) {
				MHL_TX_DBG_ERR("Protocol Error\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT & msc_abort_reason) {
				MHL_TX_DBG_ERR("Translation layer timeout\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD & msc_abort_reason) {
				MHL_TX_DBG_ERR("Undefined opcode\n");
		}
		if (BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT & msc_abort_reason) {
				MHL_TX_DBG_ERR("MSC Peer sent an ABORT\n");
		}
	}
	return ret_val;
}

/*
 * mhl_cbus_isr
 *
 * Only when MHL connection has been established. This is where we have the
 * first looks on the CBUS incoming commands or returned data bytes for the
 * previous outgoing command.
 *
 * It simply stores the event and allows application to pick up the event
 * and respond at leisure.
 *
 *	return values:
 *	0	- MHL interrupts (all of them) have been cleared
 *		- calling routine should exit
 *	1	- MHL interrupts (at least one of them) may not have been cleared
 *		- calling routine should proceed with interrupt processing.
 */
static int mhl_cbus_isr(struct drv_hw_context *hw_context, uint8_t cbus_int)
{

	/* some times a sink will send a CBUS1 message before we get
		the TDM_SYNC interrupt.  If we were in the calibrated
		state, then ignore this interrupt until we are calibrated.
	*/
	switch (hw_context->cbus_mode) {
	case CM_TRANSITIONAL_TO_eCBUS_S_CALIBRATED:
	case CM_TRANSITIONAL_TO_eCBUS_D_CALIBRATED:
		MHL_TX_DBG_ERR("%sCBUS1 message received cbus_int:0x%02x hpd status: 0x%02x%s\n"
			,ANSI_ESC_YELLOW_TEXT
			,cbus_int
			,mhl_tx_read_reg(hw_context,REG_PAGE_5_CBUS_STATUS)
			,ANSI_ESC_RESET_TEXT);

		return -1;
		break;
	default:
		break;
	}

	if (cbus1_idle_rcv_pending == hw_context->cbus1_state) {
		/* don't process these interrupts until we exit
			the cbus1_idle_rcv_pending  state */
		return -1;
	}

	if (cbus_int & ~BIT_CBUS_HPD_CHG) {
		/* bugzilla 27396
		 * Logic to detect missed HPD interrupt.
		 * Do not clear BIT_PAGE_5_CBUS_INT_0_CBUS_INT_0_STAT2 yet.
		 */
		mhl_tx_write_reg(hw_context, REG_PAGE_5_CBUS_INT_0, cbus_int &
					~BIT_CBUS_HPD_CHG);
	}

	if (BIT_CBUS_HPD_CHG & cbus_int) {
		uint8_t cbus_status;
		uint8_t status;

		/* Check if a SET_HPD came from the downstream device. */
		cbus_status = mhl_tx_read_reg(hw_context, REG_PAGE_5_CBUS_STATUS);
		status = cbus_status & BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;

		if (BIT_PAGE_5_CBUS_STATUS_CBUS_HPD & (hw_context->cbus_status ^ cbus_status)) {
			/* bugzilla 27396
			 * No HPD interrupt has been missed yet.
			 * Clear BIT_PAGE_5_CBUS_INT_0_CBUS_INT_0_STAT2.
			 */
			mhl_tx_write_reg(hw_context, REG_PAGE_5_CBUS_INT_0, BIT_PAGE_5_CBUS_INT_0_CBUS_INT_0_STAT2);
			MHL_TX_DBG_INFO("HPD change\n");
		} else {
			MHL_TX_DBG_ERR("missed HPD change\n");

			/* leave the BIT_PAGE_5_CBUS_INT_0_CBUS_INT_0_STAT2 interrupt uncleared, so that
			 * 	we get another interrupt
			 */
			/* whatever we missed, it's the inverse of what we got */
			status ^= BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
			cbus_status ^= BIT_PAGE_5_CBUS_STATUS_CBUS_HPD;
		}

		MHL_TX_DBG_INFO("DS HPD changed to %02X\n", status);

		hw_context->intr_info->flags |= DRV_INTR_FLAG_HPD_CHANGE;
		hw_context->intr_info->hpd_status = status;

		if (0 == status) {
			struct mhl_dev_context	*dev_context;
			dev_context = get_mhl_device_context(hw_context);
			MHL_TX_DBG_ERR("%sgot CLR_HPD%s\n"
				,ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
			/* Hardware  edid fifo state DOES NOT get reset on CLR_HPD
				Do not mess around with edid_fifo_block_number here!!!!!
			*/
			if (MHL_SEND_3D_REQ_OR_FEAT_REQ == hw_context->current_cbus_req.command) {

				hw_context->current_cbus_req.command = 0x00;
				hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
				hw_context->intr_info->msc_done_data = 1;
			} else if (MHL_READ_EDID_BLOCK == hw_context->current_cbus_req.command) {
				hw_context->current_cbus_req.command = 0x00;
				hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
				// setting 1 indicates there was an error
				hw_context->intr_info->msc_done_data =1;
			}
			drive_hpd_low(hw_context);
			hw_context->current_edid_request_block = 0;

			/* default values for video */
			hw_context->video_ready = false;
			hw_context->video_path = 1;

			/*
			 * This cannot wait for the upper layer to notice DRV_INTR_FLAG_HPD_CHANGE.
			 * stop_video relies on the result.
			 */
			si_edid_reset(dev_context->edid_parser_context);

			/* if DS sent CLR_HPD ensure video is not there */
			stop_video(hw_context);
		} else {
			MHL_TX_DBG_ERR("%sGot SET_HPD%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
		}

		hw_context->cbus_status = cbus_status;
	}

	if (BIT_CBUS_MSC_MT_DONE_NACK & cbus_int) {
		MHL_TX_DBG_ERR("%sGot MSC_MT_DONE_NACK%s\n"
			,ANSI_ESC_RED_TEXT
			,ANSI_ESC_RESET_TEXT);
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_NAK;
	}

	if (BIT_CBUS_MSC_MR_WRITE_STAT & cbus_int) {
		/* read status bytes */
		mhl_tx_read_reg_block(hw_context, REG_PAGE_4_MHL_STAT_0,
				ARRAY_SIZE(hw_context->intr_info->dev_status.write_stat),
				hw_context->intr_info->dev_status.write_stat);
		/* read xstatus bytes */
		mhl_tx_read_reg_block(hw_context, REG_PAGE_4_MHL_EXTSTAT_0,
				ARRAY_SIZE(hw_context->intr_info->dev_status.write_xstat),
				hw_context->intr_info->dev_status.write_xstat);
		MHL_TX_DBG_ERR("%sGot WRITE_STAT: %02x %02x %02x : %02x %02x %02x %02x%s\n"
									,ANSI_ESC_GREEN_TEXT
									,hw_context->intr_info->dev_status.write_stat[0]
									,hw_context->intr_info->dev_status.write_stat[1]
									,hw_context->intr_info->dev_status.write_stat[2]
									,hw_context->intr_info->dev_status.write_xstat[0]
									,hw_context->intr_info->dev_status.write_xstat[1]
									,hw_context->intr_info->dev_status.write_xstat[2]
									,hw_context->intr_info->dev_status.write_xstat[3]
									,ANSI_ESC_RESET_TEXT);
		if (hw_context->intr_info->dev_status.write_stat[2] >= 0x30) {
			hw_context->mhl_peer_version_stat =hw_context->intr_info->dev_status.write_stat[2];
		}
#ifdef	FORCE_OCBUS_FOR_ECTS
		/* This compile option is always enabled.
		 * It is intended to help identify code deletion by adopters who do not need this feauture.
		 * The control for forcing oCBUS works by using module parameter below.
		 * Peer version is forced to 2.0 allowing 8620 to treat the sink as if it is MHL 2.0 device
		 * and as a result never switch cbus to MHL3 eCBUS.
		*/
		{
			/* todo: what if the sink/dongle is MHL1.x? */
			extern bool force_ocbus_for_ects;
			if (force_ocbus_for_ects) {
				hw_context->mhl_peer_version_stat =
					hw_context->intr_info->dev_status.write_stat[2] = 0x20;
			}
		}
#endif // FORCE_OCBUS_FOR_ECTS
		if (MHL_STATUS_DCAP_RDY  & hw_context->intr_info->dev_status.write_stat[0]) {
			MHL_TX_DBG_INFO("DCAP_RDY in effect\n");

			switch (hw_context->cbus_mode) {
			case CM_oCBUS_PEER_VERSION_PENDING:
				if (hw_context->mhl_peer_version_stat >= 0x30) {
					if (MHL_STATUS_XDEVCAPP_SUPP & hw_context->intr_info->dev_status.write_stat[0]) {
						MHL_TX_DBG_ERR("%sdownstream device supports MHL3.0+%s\n"
										,ANSI_ESC_GREEN_TEXT
										,ANSI_ESC_RESET_TEXT);
						/*
							Our peer is an MHL3.0 or newer device.
							Here we enable DEVCAP_X and STATUS_X operations
						*/
						hw_context->cbus_mode=CM_oCBUS_PEER_IS_MHL3;
						// Per design team we need to set MHL3 manually when
						// Transitioning from oCbus to eCBUS. For bCBUS to eCBUS h/w works fine.
						mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_CTRL
												   , VAL_PAGE_3_M3_CTRL_MHL3_VALUE);

					}
				}
				break;
			default:
				break;
			}

			/* after the above does not indicate MHL3,
				then it's MHL1.x or MHL2.x
			*/
			if (CM_oCBUS_PEER_VERSION_PENDING == hw_context->cbus_mode) {
				/*
					Initialize registers to operate in oCBUS mode
				*/
				hw_context->cbus_mode=CM_oCBUS_PEER_IS_MHL1_2;
				si_mhl_tx_drv_switch_cbus_mode(hw_context, CM_oCBUS_PEER_IS_MHL1_2);
			}
			/*
				Now that we have positively identified
					the MHL version of the peer,
					we re-evaluate our settings.
			*/
			peer_specific_init(hw_context);

			/* Enable EDID interrupt */
			enable_intr(hw_context, INTR_EDID,
						(BIT_INTR9_DEVCAP_DONE_MASK
						| BIT_INTR9_EDID_DONE_MASK
						| BIT_INTR9_EDID_ERROR
						));
		}
		/*
		 * Save received write_stat info for later
		 * post interrupt processing
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_WRITE_STAT;
	}

	if ((BIT_CBUS_MSC_MR_MSC_MSG & cbus_int)) {
		/*
		 * Save received MSC message info for later
		 * post interrupt processing
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_RECVD;
		mhl_tx_read_reg_block(hw_context,
					REG_PAGE_5_MSC_MR_MSC_MSG_RCVD_1ST_DATA,
					ARRAY_SIZE(hw_context->intr_info->msc_msg),
					hw_context->intr_info->msc_msg);

		MHL_TX_DBG_INFO("MSC MSG: %02X %02X\n",
						hw_context->intr_info->msc_msg[0],
						hw_context->intr_info->msc_msg[1]);
	}

	/*
	 * don't do anything for a scratch pad write received interrupt.
	 * instead wait for the DSCR_CHG interrupt
	 */
	if (BIT_CBUS_MSC_MR_SET_INT & cbus_int) {
		MHL_TX_DBG_WARN("MHL INTR Received\n");

		/*
		 * Save received SET INT message info for later
		 * post interrupt processing
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_SET_INT;
		mhl_tx_read_reg_block(hw_context, REG_PAGE_4_MHL_INT_0,
					ARRAY_SIZE(hw_context->intr_info->int_msg),
					hw_context->intr_info->int_msg);
		/* clear the individual SET_INT bits */
		mhl_tx_write_reg_block(hw_context, REG_PAGE_4_MHL_INT_0,
					ARRAY_SIZE(hw_context->intr_info->int_msg),
					hw_context->intr_info->int_msg);

		if (MHL_INT_EDID_CHG & hw_context->intr_info->int_msg[1]) {
			int reg_val;

			MHL_TX_DBG_INFO("%sgot EDID_CHG%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);

			/* clear this bit so that BIT_PAGE_6_TPI_INFO_FSEL_TPI_INFO_EN will get cleared by the h/w */
			mhl_tx_modify_reg(hw_context,REG_PAGE_6_TPI_INFO_FSEL
					,BIT_PAGE_6_TPI_INFO_FSEL_TPI_INFO_RPT
					,0
					);

			/* MHL module will re-read EDID */
			drive_hpd_low(hw_context);

			stop_video(hw_context);

			/* prevent HDCP interrupts from coming in */
			reg_val = mhl_tx_read_reg(hw_context, REG_PAGE_0_LM_DDC);
			MHL_TX_DBG_INFO("REG_PAGE_0_LM_DDC:%02x\n",	reg_val);
			reg_val &= ~BIT_PAGE_0_LM_DDC_SW_TPI_EN;
			reg_val |= VAL_PAGE_0_LM_DDC_SW_TPI_EN_DISABLED;
			mhl_tx_write_reg(hw_context, REG_PAGE_0_LM_DDC, reg_val);

			/* SetTPIMode */
			MHL_TX_DBG_INFO("REG_PAGE_0_LM_DDC:%02x\n",	reg_val);
			reg_val &= ~BIT_PAGE_0_LM_DDC_SW_TPI_EN;
			reg_val |= VAL_PAGE_0_LM_DDC_SW_TPI_EN_ENABLED;
			mhl_tx_write_reg(hw_context, REG_PAGE_0_LM_DDC, reg_val);

			/*
			 * Clear HDCP interrupt - due to TPI enable, we may get one.
			 * TODO: Document this in the PR.
			 */
			#ifdef DO_HDCP
			mhl_tx_write_reg(hw_context, g_intr_tbl[INTR_HDCP].stat_page,
					g_intr_tbl[INTR_HDCP].stat_offset,
					0xff);
			#endif
		} else if (MHL_INT_DSCR_CHG & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_WARN("got DSCR_CHG\n");
			if (hw_context->gen2_write_burst_rcv) {
				MHL_TX_DBG_INFO("Ignored DSCR_CHG since MDT is enabled\n");
			} else {
				mhl_tx_read_reg_block(hw_context, REG_PAGE_4_MHL_SCRPAD_0,
						ARRAY_SIZE(hw_context->write_burst_data),
						hw_context->write_burst_data);
			}
		} else if (MHL_INT_DCAP_CHG & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_WARN("got DCAP_CHG\n");
		} else if (MHL_INT_REQ_WRT & hw_context->intr_info->int_msg[0]) {
			MHL_TX_DBG_WARN("got REQ_WRT\n");
		}
	}
	if (BIT_CBUS_MSC_MT_DONE & cbus_int) {
		bool completed=true;
		MHL_TX_DBG_INFO("MSC_REQ_DONE,0x%02x(0x%02x,0x%02x)\n"
									,hw_context->current_cbus_req.command
									,hw_context->current_cbus_req.reg
									,hw_context->current_cbus_req.reg_data
									);

		if (MHL_SET_INT == hw_context->current_cbus_req.command) {
			if (MHL_RCHANGE_INT==hw_context->current_cbus_req.reg) {
				if (MHL2_INT_3D_REQ &hw_context->current_cbus_req.reg_data) {
					MHL_TX_DBG_WARN("3D_REQ complete\n");
					hw_context->cbus1_state =cbus1_idle_rcv_enabled;
				}
				if (MHL_INT_GRT_WRT  &hw_context->current_cbus_req.reg_data) {
					MHL_TX_DBG_WARN("GRT_WRT complete\n");
					hw_context->cbus1_state =cbus1_idle_rcv_enabled;
				}
			}
		} else if (MHL_SEND_3D_REQ_OR_FEAT_REQ == hw_context->current_cbus_req.command) {
			/* if this command completes before we can enable HAWB,
					control will come here.  We want to hold off other
					CBUS activity until the peer finishes sending 3D_DTD and
					3D_VIC bursts.  We will call si_mhl_tx_msc_command_done directly
					from si_mhl_tx_drv_set_upstream_edid which is called by the upper layer.
			*/
			completed = false;
			hw_context->cbus1_state =cbus1_idle_rcv_enabled;
		}
		if (completed) {
			hw_context->current_cbus_req.command = 0x00;
			hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
			hw_context->intr_info->msc_done_data =
				mhl_tx_read_reg(hw_context, REG_PAGE_5_MSC_MT_RCVD_DATA0);
		}
	}
	return -1;
}

static int int_5_isr(struct drv_hw_context *hw_context, uint8_t int_5_status)
{
	int ret_val = 0;

	if (int_5_status & BIT_INTR_SCDT_CHANGE) {
		uint8_t temp;
		temp = mhl_tx_read_reg(hw_context, REG_PAGE_2_TMDS_CSTAT_P3);

		if (BIT_PAGE_2_TMDS_CSTAT_P3_SCDT & temp) {
			MHL_TX_DBG_WARN("%sGot SCDT HIGH%s\n",
									ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
			/*
			 * enable infoframe interrupt
			 */
			enable_intr(hw_context
						,INTR_INFR
						,BIT_INTR8_CEA_NEW_AVI | BIT_INTR8_CEA_NEW_VSI
						);

			if (IN_MHL3_MODE(hw_context) ||
				si_edid_sink_is_hdmi(hw_context->intr_info->edid_parser_context)) {
				// 2013-11-24 Per Lei Ming: this is OK in MHL3 mode.
				mhl_tx_write_reg(hw_context, REG_PAGE_6_TPI_SC,
									VAL_PAGE_6_TPI_SC_TPI_OUTPUT_MODE_0_HDMI);

				/*
				 * enable infoframe interrupt
				 */
				enable_intr(hw_context
							,INTR_INFR
							,BIT_INTR8_CEA_NEW_AVI | BIT_INTR8_CEA_NEW_VSI
							);
			} else {
				if (hw_context->hpd_high_callback_status >= 0) {
					/* Video is already started.
						Set this flag so that we catch mode changes
					*/
					MHL_TX_DBG_WARN("hpd_high_callback_status >= 0\n");
					hw_context->hpd_high_callback_status = hpd_high_video_not_ready;
					return ret_val;
				}
				/*
				 * output video here for DVI or
				 * if a VIC is in hand for HDMI
				 */
				//start_video(hw_context);
			}
			//start_video(hw_context);
			MHL_TX_DBG_INFO("actual_mode: %s0x%02x%s\n"
				,ANSI_ESC_YELLOW_TEXT
				,mhl_tx_read_reg(hw_context,REG_PAGE_2_RX_HDMI_CTRL0)
				,ANSI_ESC_RESET_TEXT);
		} else {
			MHL_TX_DBG_WARN("%sGot SCDT LOW%s\n",
									ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT);

			/* Clear all InfoFrame info which is now stale. */
			mhl_tx_write_reg(hw_context, REG_PAGE_2_TMDS_CSTAT_P3,
				BIT_PAGE_2_TMDS_CSTAT_P3_AVIF_MANUAL_CLEAR_STROBE |
				BIT_PAGE_2_TMDS_CSTAT_P3_DISABLE_AUTO_AVIF_CLEAR);

			hw_context->valid_vsif = 0;
			hw_context->valid_avif = 0;
			memset(&hw_context->current_vs_info_frame,0,sizeof(hw_context->current_vs_info_frame));
			memset(&hw_context->current_avi_info_frame,0,sizeof(hw_context->current_avi_info_frame));

			stop_video(hw_context);
			/*
			 * disable infoframe interrupt
			 */
			enable_intr(hw_context, INTR_INFR, 0);
		}
	}
	return ret_val;
}

#ifdef USE_HW_TIMER
static int int_1_isr(struct drv_hw_context *hw_context, uint8_t int_1_status)
{
	if (int_1_status & BIT_HW_TIMER_POP)
	{
		MHL_TX_DBG_INFO("Timer Pop\n");

		/* Check timer flag(s) and process them here */
	}

	mhl_tx_write_reg(hw_context, REG_PAGE_0_INTR1, 0x80);

	return 0;
}
#endif
#define BIT_0072_SW_INTR 0x04
static int int_2_isr(struct drv_hw_context *hw_context, uint8_t int_2_status)
{
	if (BIT_0072_SW_INTR & int_2_status) {
		MHL_TX_DBG_ERR("enabling RGND\n");
		/* Keep only RGND interrupt enabled for 8620 */
		enable_intr(hw_context, INTR_DISC, BIT_RGND_READY_INT);
		wait_for_user_intr = 0;
	}
	return 0;
}

/*
    get_device_id
    returns chip Id
 */
int get_device_id(struct drv_hw_context *hw_context)
{
	int ret_val;
	uint16_t number;

	ret_val = mhl_tx_read_reg(hw_context, REG_PAGE_0_DEV_IDH);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("I2C error 0x%x\n", ret_val);
		return ret_val;
	}
	number = ret_val << 8;

	ret_val = mhl_tx_read_reg(hw_context, REG_PAGE_0_DEV_IDL);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("I2C error 0x%x\n", ret_val);
		return ret_val;
	}
	ret_val |= number;

	return ret_val;
}

/*
    get_device_rev
    returns chip revision
 */
static int get_device_rev(struct drv_hw_context *hw_context)
{
	int ret_val;

	ret_val = mhl_tx_read_reg(hw_context, REG_PAGE_0_DEV_REV);
	if (ret_val < 0) {
		MHL_TX_DBG_ERR("I2C error\n");
		ret_val = -1;
	}

	return ret_val;
}

/*
 * clear_and_disable_on_disconnect
 */
static void clear_and_disable_on_disconnect(struct drv_hw_context *hw_context)
{
	uint8_t	intr_num;
	/* clear and mask all interrupts */
	for(intr_num = 0; intr_num < MAX_INTR; intr_num++)
	{
		if (INTR_DISC == intr_num) {
			/* clear discovery interrupts except MHL_EST before going to sleep. */
			mhl_tx_write_reg(hw_context,
					g_intr_tbl[INTR_DISC].stat_page,
					g_intr_tbl[INTR_DISC].stat_offset,
					~BIT_RGND_READY_INT);
			if (wait_for_user_intr) {
				/* Keep only USER interrupt enabled for 8620 */
				enable_intr(hw_context, INTR_DISC, 0);
			} else {
				/* Keep only RGND interrupt enabled for 8620 */
				enable_intr(hw_context, INTR_DISC, BIT_RGND_READY_INT);
			}
		} else {
			/* Clear and disable all other interrupts */
			mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].stat_page, g_intr_tbl[intr_num].stat_offset, 0xff);
			if (INTR_USER== intr_num) {
				if (wait_for_user_intr) {
					/* Keep only USER interrupt enabled for 8620 */
					enable_intr(hw_context, INTR_USER, BIT_0072_SW_INTR);
				} else {
					enable_intr(hw_context, INTR_USER, 0x00);
				}
			} else {
				enable_intr(hw_context, intr_num, 0x00);
			}
		}
	}
}
/*
 * switch_to_d3
 * This function performs s/w as well as h/w state transitions.
 */
static void switch_to_d3(struct drv_hw_context *hw_context,bool do_interrupt_clear)
{
	/*
	 * Check GPIO if chip can enter D3.
	 * to read/write register contents from SiIMon, user may prevent D3.
	 */
	 MHL_TX_DBG_WARN("switch_to_d3...........\n");
	//if (get_config(hw_context, ALLOW_D3)) 
	{
		if (do_interrupt_clear) {
			clear_and_disable_on_disconnect(hw_context);
		}
		/*
		 * Leave just enough of the transmitter powered up
		 * to detect connections.
		 * i.e. disable the following:
		 *	BIT_PAGE_0_DPD_PDNRX12, BIT_PAGE_0_DPD_PWRON_HSIC,
		 *	BIT_PAGE_0_DPD_PDIDCK_N, BIT_PAGE_0_DPD_PD_MHL_CLK_N
		 */
		/*mhl_tx_write_reg(hw_context, REG_PAGE_0_DPD
						, BIT_PAGE_0_DPD_PWRON_PLL
						| BIT_PAGE_0_DPD_PDNTX12
						| BIT_PAGE_0_DPD_OSC_EN
						);*/
		#define GPIO_BB_ID_SEL                        26
		gpio_direction_output(GPIO_BB_ID_SEL, 0);
		Sii8620_MHL_Mode = false;
	} /*else if (do_interrupt_clear) {
		clear_and_disable_on_disconnect(hw_context);
	}*/
/*	si_dump_important_regs(hw_context);*/
}

static void cbus_reset(struct drv_hw_context *hw_context)
{
	MHL_TX_DBG_WARN("Perform CBUS reset to clean MHL STAT values\n");
	mhl_tx_write_reg(hw_context, REG_PAGE_0_PWD_SRST, BIT_PAGE_0_PWD_SRST_CBUS_RST | BIT_PAGE_0_PWD_SRST_CBUS_RST_SW_EN);		//
	mhl_tx_write_reg(hw_context, REG_PAGE_0_PWD_SRST, BIT_PAGE_0_PWD_SRST_CBUS_RST_SW_EN);		//

#ifdef SWWA_BZ30759 //(
	/* switch away from connector wires using 6051 */
	//set_pin(hw_context,X02_USB_SW_CTRL,0);
	platform_mhl_tx_hw_reset(TX_HW_RESET_PERIOD, TX_HW_RESET_DELAY);
	/* switch back to connector wires using 6051 */
	//set_pin(hw_context,X02_USB_SW_CTRL,1);
#endif //)
}

/*
 * disconnect_mhl
 * This function performs s/w as well as h/w state transitions.
 */
static void disconnect_mhl(struct drv_hw_context *hw_context,bool do_interrupt_clear)
{
	disable_gen2_write_burst_rcv(hw_context);
	disable_gen2_write_burst_xmit(hw_context);

	stop_video(hw_context);
	MHL_TX_DBG_WARN("STOP_VIDEO DONE\n");

	/* Meet an MHL CTS timing - Tsrc:cbus_float. Must do before resetting CBUS */
	/* Fixes failing cases 3.3.14.1, 3.3.14.3, 3.3.22.2 */
	msleep(50);
	cbus_reset(hw_context);
	clear_auto_zone_for_mhl_3(hw_context);

	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, 0x40);		// 0x0710
	mhl_tx_write_reg(hw_context, REG_PAGE_5_CBUS3_CNVT, 0x84);

	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL5, 0x0D);	/* 0x0347 Check */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL3, 0x00); /* 0x0345 */
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1, 0x0A);		// 0x0711
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL2, 0x14);		// 0x0712
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL17,0x00);		// 0x072A
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL18,0x00);		// 0x072B
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL19,0x00);		// 0x072C
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1A,0x00);		// 0x072D
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7,0x06);		// 0x0717
	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL11,0x32);		// 0x0721
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL4,0x28);		// 0x0346

	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLF, 0x00);		// 0x071F

	/* Enable timeout */
	mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTLB,0x00);  /* 0x071B */
	//new add by kelvin,2013.11.14
	mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL14, 0x00); //TRAP 0X03 0x0724
	mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL15, 0x00); //PULSE FOR TRAP 0x0725

	mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, 0x40);		// 0x0710 - check with Lei Ming

	//new add by kelvin,2013.11.14
	mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL14, 0x00); //TRAP 0X00 0x0724
	mhl_tx_write_reg(hw_context,REG_PAGE_7_COC_CTL15, 0x00); //PULSE FOR TRAP 0x0725

	mhl_tx_write_reg(hw_context, REG_PAGE_1_HRXCTRL3, 0x07);		// 0x0104
	mhl_tx_write_reg(hw_context, REG_PAGE_5_CBUS3_CNVT, 0x84);		// 0x05DC  - check with Lei Ming

	mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR13, 0xFC);	 // 0191
	mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR14, 0xFF);	 // 0192
	mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR15, 0xFF);  // 0193
	mhl_tx_write_reg(hw_context, REG_PAGE_1_FCCTR50, 0x03);  // 01B6

	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL0,
		(BIT_PAGE_3_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X
		| BIT_PAGE_3_MHL_PLL_CTL0_CRYSTAL_CLK_SEL
		| BIT_PAGE_3_MHL_PLL_CTL0_ZONE_MASK_OE));		/* 0x0337 */

	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xC0);		/* 0x0331 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL1 ,0xBB);		/* 0x0332 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL3 ,0x48);		/* 0x0334 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5 ,0x3F);		/* 0x0336 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2 ,0x2F);		/* 0x0333 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL6 ,0x2A);		/* 0x0350 */
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL7 ,0x04);		/* 0x0351 */

	hw_context->cbus_mode = CM_NO_CONNECTION;

	/* Pull the upstream HPD line low to prevent EDID and HDCP activity. */
	drive_hpd_low(hw_context);

	mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_CTRL,
							VAL_PAGE_3_M3_CTRL_PEER_VERSION_PENDING_VALUE);

	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL1, 0x07);

	/* For proper MHL discovery and tolerance of impedance measurement,
	set 5E3[7:4]=0x01 - this turns off the CBUS pull up for discovery states
	and 20K for IDLE state before impedance is measured.
	Fixes CBUS CTS cases that measure +/-20% tolerance of 1K MHL impedance.
		Such as 3.3.5.1, 3.3.5.5, 3.3.6.4
	*/
	mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL4, 0x10);	// Default 0x8C
	mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL8, 0x00);	// Default 0x83

	/* DO NOT enable wake/discovery pulses until successful RGND == 1K */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL9
						, BIT_PAGE_5_DISC_CTRL9_WAKE_DRVFLT
						| BIT_PAGE_5_DISC_CTRL9_WAKE_PULSE_BYPASS
						);

	/* Set up discovery registers to attempt oCBUS discovery */
	mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL1, 0x25);
	if (do_interrupt_clear) {
		clear_and_disable_on_disconnect(hw_context);
	}

	/* clear this flag to fix DS hot plug issue */
	hw_context->cbus_status = 0;
	hw_context->current_cbus_req.command = 0x00;
	hw_context->hawb_write_pending = false;
	hw_context->cbus1_state =cbus1_idle_rcv_disabled;
}

/*
 * int_4_isr
 * MHL device discovery interrupt handler
 * 	1. When impedance is measured as 1k, RGND interrupt is asserted.
 * 	2. Chip sends out wake pulses and discovery pulses.
 * 	   Then asserts MHL_EST if CBUS stays high to meet MHL timings.
 * 	3. If discovery fails, NON_MHL_EST is asserted.
 *  4. If MHL cable is removed, CBUS_DIS is asserted.
 *     (Need to check this bit all the time)
 */
static int int_4_isr(struct drv_hw_context *hw_context, uint8_t int_4_status)
{
	int	ret_val = 0; /* Safe to clear interrupt from master handler */
	//bool proceed_with_wake_and_discover = false;
	//added by zhangyue for qualcomm platform start
	struct mhl_dev_context *dev_context = NULL;
	struct qualcomm_platform_data *qualcomm_p_data = NULL;
	

	dev_context = get_mhl_device_context(hw_context);
	qualcomm_p_data = dev_context->qualcomm_p_data;
	
	//added by zhangyue for qualcomm platform end
	if ((BIT_CBUS_MHL12_DISCON_INT & int_4_status) ||
		(BIT_CBUS_MHL3_DISCON_INT & int_4_status) ||
		(BIT_NOT_MHL_EST_INT & int_4_status)) {
		MHL_TX_DBG_ERR("Got CBUS_DIS. MHL disconnection\n");
		/* For proper MHL discovery and tolerance of impedance measurement,
		   set 5E3[7:4]=0x01 - this turns off the CBUS pull up for discovery states
		   and 20K for IDLE state before impedance is measured.
		   Fixes CBUS CTS cases that measure +/-20% tolerance of 1K MHL impedance.
				Such as 3.3.5.1, 3.3.5.5, 3.3.6.4
		*/
		mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL4, 0x10);	// Default 0x8C


		/* Setup termination etc. */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_DISCONNECT;
		if (BIT_CBUS_MHL12_DISCON_INT & int_4_status) {
			disconnect_mhl(hw_context,true);
			switch_to_d3(hw_context,false);
		} else { /* must be BIT_NOT_MHL_EST_INT */
			disconnect_mhl(hw_context,false);
			switch_to_d3(hw_context,true);
		}
		ret_val = 0xFF;	/* interrupt has been cleared already in disconnect_mhl */

		if (hw_context->current_cbus_req.command) {
			hw_context->current_cbus_req.command = 0x00;
			hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
			hw_context->intr_info->msc_done_data = 0;
		}
		//added by zhang yue for qualcomm platform start
		if(qualcomm_p_data->hdmi_mhl_ops && qualcomm_p_data->hdmi_mhl_ops->set_upstream_hpd){
			qualcomm_p_data->hdmi_mhl_ops->set_upstream_hpd(qualcomm_p_data->hdmi_pdev, 0);
		}
		
		power_supply_changed(&(qualcomm_p_data->mhl_psy));

		if (qualcomm_p_data->notify_usb_online){
			qualcomm_p_data->notify_usb_online(qualcomm_p_data->notify_ctx, 0);
		}
		dev_context->mhl_mode = 0;
		//added by zhang yue for qualcomm platform end

	} else if (int_4_status & BIT_RGND_READY_INT) {
		int disc_stat2;
//#define SWWA_SKIP_RGND_CHECK		
#ifndef SWWA_SKIP_RGND_CHECK
		bool proceed_with_wake_and_discover = false;
#endif
		disc_stat2 = mhl_tx_read_reg(hw_context, REG_PAGE_5_DISC_STAT2) &
												MSK_PAGE_5_DISC_STAT2_RGND;
		MHL_TX_DBG_ERR("RGND impedance detected (%s)\n",
												rgnd_value_string[disc_stat2]);
		//disc_stat2 = mhl_tx_read_reg(hw_context, REG_PAGE_5_DISC_CTRL7);
		if (VAL_PAGE_5_RGND_1K == disc_stat2) {
		MHL_TX_DBG_ERR("RGND impedance detected (%s)222222222222\n",
												rgnd_value_string[disc_stat2]);
			}
//#define SWWA_SKIP_RGND_CHECK		
#ifndef SWWA_SKIP_RGND_CHECK
		//bool proceed_with_wake_and_discover = false;
		if (VAL_PAGE_5_RGND_1K == disc_stat2) {
			proceed_with_wake_and_discover = true;
		} else {
			mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL9
						, BIT_PAGE_5_DISC_CTRL9_WAKE_DRVFLT
						| BIT_PAGE_5_DISC_CTRL9_NOMHL_EST
						|BIT_PAGE_5_DISC_CTRL9_WAKE_PULSE_BYPASS
						);

		}
	if (proceed_with_wake_and_discover)
#else
		if (1) 
#endif
		{
			MHL_TX_DBG_WARN("Cable impedance = 1k (MHL Device)\n");
			//added by zhang yue for qualcomm platform start
			MHL_TX_DBG_ERR("qualcomm_p_data->hdmi_mhl_ops->set_upstream_hpd start\n");
			//
			if(qualcomm_p_data->hdmi_mhl_ops && qualcomm_p_data->hdmi_mhl_ops->set_upstream_hpd){
				qualcomm_p_data->hdmi_mhl_ops->set_upstream_hpd(qualcomm_p_data->hdmi_pdev, 1);
				MHL_TX_DBG_ERR("qualcomm_p_data->hdmi_mhl_ops->set_upstream_hpd end\n");
			}

			#if 1
			power_supply_changed(&(qualcomm_p_data->mhl_psy));
			if (qualcomm_p_data->notify_usb_online){
				qualcomm_p_data->notify_usb_online(qualcomm_p_data->notify_ctx, 1);
			}	
			#endif
			dev_context->mhl_mode = 1;

			//added by zhang yue for qualcomm platform end

			mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL9
								, BIT_PAGE_5_DISC_CTRL9_WAKE_DRVFLT
								| BIT_PAGE_5_DISC_CTRL9_DISC_PULSE_PROCEED
								);
			/* For proper MHL discovery and tolerance of impedance measurement,
			   set 5E3[7:4]=0x90.
			   This sets the CBUS pull up for discovery states as 5k
			   and 20K for IDLE state after impedance has been measured.
			   Fixes CBUS CTS cases that measure +/-20% tolerance of 1K MHL impedance.
					Such as 3.3.5.1, 3.3.5.5, 3.3.6.4
			*/
			mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL4, 0x90);

			/* speculatively enable the MHL3 clock so that by the time
				the wake/discover sequence is complete, the MHL3 clock
				will already be stabilized LONG BEFORE the transition
				to eCBUS mode.
			*/


			/* enable HSIC earlier to enhance stability */

		/* enable remaining discovery interrupts */
		enable_intr(hw_context, INTR_DISC,
				BIT_MHL3_EST_INT_MASK
				| BIT_MHL_EST_INT_MASK
				| BIT_NOT_MHL_EST_INT_MASK
				| BIT_CBUS_MHL3_DISCON_INT_MASK
				| BIT_CBUS_MHL12_DISCON_INT_MASK
				| BIT_RGND_READY_INT_MASK
				);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL0,
			(BIT_PAGE_3_MHL_PLL_CTL0_HDMI_CLK_RATIO_1X
			| BIT_PAGE_3_MHL_PLL_CTL0_CRYSTAL_CLK_SEL
			| BIT_PAGE_3_MHL_PLL_CTL0_ZONE_MASK_OE));		/* 0x0337 */

			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xC0);		/* 0x0331 - def is bc.  */
			mhl_tx_write_reg(hw_context,REG_PAGE_3_M3_CTRL,VAL_PAGE_3_M3_CTRL_PEER_VERSION_PENDING_VALUE); /* 3E0 - def is 0A for MHl2 digital */
			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL1 ,0xA2);		/* 0x0332 */
			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2 ,0x03);		/* 0x0333 */
			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL3 ,0x35);		/* 0x0334 */
			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5 ,0x01);		/* 0x0336 */
			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL6 ,0x02);		/* 0x0350 */
			mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL7 ,0x04);		/* 0x0351 */
			mhl_tx_write_reg(hw_context, REG_PAGE_0_DPD
							, BIT_PAGE_0_DPD_PWRON_PLL
							| BIT_PAGE_0_DPD_PDNTX12
							| BIT_PAGE_0_DPD_OSC_EN
							| BIT_PAGE_0_DPD_PWRON_HSIC
							);
			enable_intr(hw_context, INTR_COC, BIT_COC_PLL_LOCK_STATUS_CHANGE | BIT_COC_CALIBRATION_DONE);
		/* Enable MSC interrupt to handle initial exchanges */
		enable_intr(hw_context, INTR_MERR,
				(
#ifdef	PRINT_DDC_ABORTS
				BIT_CBUS_DDC_ABORT
				|
#endif //	PRINT_DDC_ABORTS
				BIT_CBUS_MSC_ABORT_RCVD
				| BIT_CBUS_CMD_ABORT
				));
		enable_intr(hw_context, INTR_MSC,
				(BIT_CBUS_MSC_MT_DONE
				| BIT_CBUS_HPD_CHG
				| BIT_CBUS_MSC_MR_WRITE_STAT
				| BIT_CBUS_MSC_MR_MSC_MSG
				| BIT_CBUS_MSC_MR_WRITE_BURST
				| BIT_CBUS_MSC_MR_SET_INT
				| BIT_CBUS_MSC_MT_DONE_NACK
				));
		} else {
			/* enable remaining discovery interrupts, except MHL_EST */
			enable_intr(hw_context, INTR_DISC,
					  BIT_NOT_MHL_EST_INT_MASK
					| BIT_CBUS_MHL3_DISCON_INT_MASK
					| BIT_CBUS_MHL12_DISCON_INT_MASK
					| BIT_RGND_READY_INT_MASK
					);
			//added by zhang yur for qualcomm platform	start 
			dev_context->mhl_mode = 0;
			//added by zhang yur for qualcomm platform	end 
		}
		//added by zhang yur for qualcomm platform  start 
		complete(&dev_context->rgnd_done);
		//added by zhang yur for qualcomm platform  end 
	} else if (int_4_status & BIT_MHL_EST_INT) {
		uint8_t	msc_compat = BIT_PAGE_5_CBUS_MSC_COMPATIBILITY_CONTROL_ENABLE_XDEVCAP;
		//uint8_t m3_ctrl = VAL_PAGE_3_M3_CTRL_PEER_VERSION_PENDING_VALUE;

		/*
		 * ENABLE_DISCOVERY ensures it sends wake up and discovery pulse
		 * 	 and as result sink/dongle would respond CBUS high.
		 * 8620: ENABLE_DISCOVERY is always set. Therefore, on successful
		 * 		 discovery, 8620 will assert MHL_EST.
		 */
		MHL_TX_DBG_ERR("oCBUS Connection Established\n");

		hw_context->cbus_mode=CM_oCBUS_PEER_VERSION_PENDING;
		/* For proper MHL discovery and tolerance of impedance measurement,
		   set 5E3[7:4]=0x01 - this turns off the CBUS pull up for discovery states
		   and 20K for IDLE state after MHL sink has been discovered.
		   Fixes CBUS CTS cases that measure +/-20% tolerance of 1K MHL impedance.
				Such as 3.3.5.1, 3.3.5.5, 3.3.6.4
		*/
		mhl_tx_write_reg(hw_context, REG_PAGE_5_DISC_CTRL4, 0x10);	// Default 0x8C

		/* speculatively enable XDEVCAP reads so that MHL 3.0 sinks and dongles can
			read xdevcap registers in oCBUS mode */
		mhl_tx_write_reg(hw_context
			,REG_PAGE_5_CBUS_MSC_COMPATIBILITY_CONTROL
			,msc_compat);

		/* Setup registers and enable interrupts for DCAP_RDY, SET_HPD etc. */
		init_regs(hw_context);

		/*
		 *  Setting this flag triggers sending DCAP_RDY.
		 *  Tested RK-9296 - it works fine.
		 */
		hw_context->intr_info->flags |= DRV_INTR_FLAG_CONNECT;
	}

	return ret_val;
}
static int g2wb_err_isr(struct drv_hw_context *hw_context, uint8_t intr_stat)
{
	if (intr_stat) {
		if (BIT_MDT_RCV_TIMEOUT	& intr_stat) {
			MHL_TX_DBG_WARN("%sBIT_MDT_RCV_TIMEOUT%s\n"
				,ANSI_ESC_YELLOW_TEXT,ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_RCV_SM_ABORT_PKT_RCVD & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_RCV_SM_ABORT_PKT_RCVD%s\n"
				,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_RCV_SM_ERROR & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_RCV_SM_ERROR%s\n"
				,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_XMIT_TIMEOUT & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_XMIT_TIMEOUT %s\n"
				,ANSI_ESC_YELLOW_TEXT,ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_XMIT_SM_ABORT_PKT_RCVD & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_XMIT_SM_ABORT_PKT_RCVD - status: 0x%02x%s\n"
				,ANSI_ESC_RED_TEXT
				,mhl_tx_read_reg(hw_context,REG_PAGE_5_MDT_SM_STAT)
				,ANSI_ESC_RESET_TEXT);
		}

		if (BIT_MDT_XMIT_SM_ERROR & intr_stat) {
			MHL_TX_DBG_ERR("%sBIT_MDT_XMIT_SM_ERROR%s\n"
				,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
		}
	}
	return 0;
}


static int g2wb_isr(struct drv_hw_context *hw_context, uint8_t intr_stat)
{
	int ret_val = 0;
	if (intr_stat) {
		if (BIT_MDT_XFIFO_EMPTY & intr_stat) {
			struct cbus_req *peek_req;
			struct mhl_dev_context *dev_context;

			dev_context = container_of((void *)hw_context,
					struct mhl_dev_context, drv_context);

			MHL_TX_DBG_WARN("%sHAWB XFIFO empty%s XFIFO_STAT: 0x%02x\n"
				,ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT
				,mhl_tx_read_reg(hw_context,REG_PAGE_5_MDT_XFIFO_STAT));

			peek_req = peek_next_cbus_transaction(dev_context);
			if (NULL == peek_req) {
				disable_gen2_write_burst_xmit(hw_context);
				disable_gen2_write_burst_rcv(hw_context);
			} else if (MHL_WRITE_BURST != peek_req->command) {
				disable_gen2_write_burst_xmit(hw_context);
				disable_gen2_write_burst_rcv(hw_context);
			}
		}
		if (BIT_MDT_RFIFO_DATA_RDY & intr_stat) {
			uint8_t length;
			uint8_t mdt_buffer[20];

			/* Read all bytes */
			mhl_tx_read_reg_block(hw_context,
					REG_PAGE_5_MDT_RCV_READ_PORT,
					17,
					mdt_buffer);

			MHL_TX_DBG_WARN("%sgot G2WB incoming 0x%02x%02x%s = %02X\n"
					,ANSI_ESC_GREEN_TEXT,mdt_buffer[1],mdt_buffer[2]
					,ANSI_ESC_RESET_TEXT, intr_stat);

			/* first byte contains the length of data */
			length = mdt_buffer[0];
			/*
			 * There isn't any way to know how much of the scratch pad
			 * was written so we have to read it all.  The app. will have
			 * to parse the data to know how much of it is valid.
			 */
	/*		mhl_tx_read_reg_block(hw_context, REG_CBUS_MDT_RCV_READ_PORT,
					ARRAY_SIZE(hw_context->write_burst_data),
					hw_context->write_burst_data);
	*/
			memcpy(hw_context->write_burst_data, &mdt_buffer[1], 16);

			/* Signal upper layer of this arrival */
			hw_context->intr_info->flags |= DRV_INTR_FLAG_WRITE_BURST;

			/*
			 * Clear current level in the FIFO.
			 * Moves pointer to the next keep RSM enabled
			 */
			mhl_tx_write_reg(hw_context,
					REG_PAGE_5_MDT_RCV_CONTROL,
					BIT_PAGE_5_MDT_RCV_CONTROL_MDT_RFIFO_CLR_CUR
					| BIT_PAGE_5_MDT_RCV_CONTROL_MDT_RCV_EN
					| hw_context->delayed_hawb_enable_reg_val
					);
			{
				/* don't let the caller clear BIT_MDT_RFIFO_DATA_RDY until
					the receive buffer is empty
				*/
				int rfifo_stat;
				rfifo_stat = mhl_tx_read_reg(hw_context,REG_PAGE_5_MDT_RFIFO_STAT);
				if (rfifo_stat & MSK_PAGE_5_MDT_RFIFO_STAT_MDT_RFIFO_CNT) {
					ret_val=BIT_MDT_RFIFO_DATA_RDY;
				} else {
					switch (hw_context->cbus1_state) {
					case cbus1_idle_rcv_enabled:
					case cbus1_idle_rcv_disabled:
						break;
					case cbus1_idle_rcv_pending:
						hw_context->cbus1_state = cbus1_idle_rcv_enabled;
						break;
					case cbus1_msc_pending_delayed_rcv_enable:
						MHL_TX_DBG_ERR("%sCan't get here%s\n"
							,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT)
						break;
					case cbus1_msc_pending_rcv_disabled:
						break;
					case cbus1_xmit_pending_xmit_and_rcv_enabled:
						break;
					case cbus1_xmit_pending_xmit_and_rcv_pending:
						hw_context->cbus1_state = cbus1_xmit_pending_xmit_and_rcv_enabled;
						break;
					}
				}
			}
		}
		if (BIT_MDT_IDLE_AFTER_HAWB_DISABLE & intr_stat) {
			MHL_TX_GENERIC_DBG_PRINT(DBG_MSG_LEVEL_WARN,
				"%shawb_idle%s\n", ANSI_ESC_GREEN_TEXT, ANSI_ESC_RESET_TEXT)
			if (MHL_WRITE_BURST == hw_context->current_cbus_req.command) {
				hw_context->current_cbus_req.command = 0x00;
				hw_context->intr_info->flags |= DRV_INTR_FLAG_MSC_DONE;
				hw_context->intr_info->msc_done_data = 0;
				hw_context->hawb_write_pending = false;
				hw_context->cbus1_state = cbus1_idle_rcv_enabled;
			}
		}
	}
	return ret_val;
}

static void enable_intr(struct drv_hw_context *hw_context,
							uint8_t intr_num,
							uint8_t intr_mask)
{
	g_intr_tbl[intr_num].mask = intr_mask;
	mhl_tx_write_reg(hw_context, g_intr_tbl[intr_num].mask_page,
				g_intr_tbl[intr_num].mask_offset, intr_mask);
}

	/*
	 * Enable interrupts and turn on the engine
	 *
	 * Clearly this must be moved to si_8620_drv.c because all
	 * the definitions I need are there.
	 */
void si_mhl_tx_drv_enable_emsc_block(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context 			*hw_context;
	uint8_t	intStatus;
	MHL_TX_DBG_INFO("Enabling EMSC and EMSC interrupts\n");

	hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	mhl_tx_modify_reg( hw_context, REG_PAGE_3_GENCTL,
		BIT_PAGE_3_GENCTL_EMSC_EN |
		BIT_PAGE_3_GENCTL_CLR_EMSC_RFIFO |
		BIT_PAGE_3_GENCTL_CLR_EMSC_XFIFO,
		BIT_PAGE_3_GENCTL_EMSC_EN |
		BIT_PAGE_3_GENCTL_CLR_EMSC_RFIFO |
		BIT_PAGE_3_GENCTL_CLR_EMSC_XFIFO );
	mhl_tx_modify_reg( hw_context, REG_PAGE_3_GENCTL,
		BIT_PAGE_3_GENCTL_CLR_EMSC_RFIFO | BIT_PAGE_3_GENCTL_CLR_EMSC_XFIFO,
		0 );
	mhl_tx_modify_reg( hw_context, REG_PAGE_3_COMMECNT,
		BIT_PAGE_3_COMMECNT_I2C_TO_EMSC_EN,
		BIT_PAGE_3_COMMECNT_I2C_TO_EMSC_EN );
	intStatus = mhl_tx_read_reg( hw_context, REG_PAGE_3_EMSCINTR );
	mhl_tx_write_reg( hw_context, REG_PAGE_3_EMSCINTR, intStatus );
	enable_intr(hw_context, 7 /*INTR_BLOCK */, BIT_PAGE_3_EMSCINTR_SPI_DVLD );
}

void si_mhl_tx_drv_device_isr(struct drv_hw_context *hw_context,
										struct interrupt_info *intr_info)
{
	uint8_t	intr_num;
	//uint8_t aggregated_intr_status[NUM_AGGREGATED_INTR_REGS];

	hw_context->intr_info = intr_info;
	MHL_TX_DBG_INFO(" si_mhl_tx_drv_device_isr\n");

	MHL_TX_DBG_INFO("%sgot INTR COC_STAT_0:0x%02x%s\n"
			,ANSI_ESC_GREEN_TEXT
			,mhl_tx_read_reg(hw_context,REG_PAGE_7_COC_STAT_0)
			,ANSI_ESC_RESET_TEXT);
/*
	mhl_tx_read_reg_block(hw_context, REG_PAGE_0_FAST_INTR_STAT
						, sizeof(aggregated_intr_status)
						, aggregated_intr_status);
	MHL_TX_DBG_INFO("%s aggr intr status:"
					 " %02x %02x %02x %02x %02x %02x %02x%s\n"
				,ANSI_ESC_GREEN_TEXT
				,aggregated_intr_status[FAST_INTR_STAT]
				,aggregated_intr_status[L1_INTR_STAT_0]
				,aggregated_intr_status[L1_INTR_STAT_1]
				,aggregated_intr_status[L1_INTR_STAT_2]
				,aggregated_intr_status[L1_INTR_STAT_3]
				,aggregated_intr_status[L1_INTR_STAT_4]
				,aggregated_intr_status[L1_INTR_STAT_5]
				,ANSI_ESC_RESET_TEXT
				);
*/
	/* Skip checking interrupts if GPIO pin is not asserted anymore */
	for (intr_num = 0;
			(intr_num < MAX_INTR);// && (is_interrupt_asserted());
				intr_num++) {
		if (g_intr_tbl[intr_num].mask) {
			/*
			uint8_t aggregated_index,aggregated_id_bit,aggregated_status;
			aggregated_index = g_intr_tbl[intr_num].aggr_stat_index;
			aggregated_id_bit  = g_intr_tbl[intr_num].aggr_stat_id_bit;
			aggregated_status= aggregated_intr_status[aggregated_index];
			if (aggregated_status & aggregated_id_bit) */{
				int reg_value;
				uint8_t	intr_stat;

				reg_value = mhl_tx_read_reg(hw_context,
						g_intr_tbl[intr_num].stat_page,
						g_intr_tbl[intr_num].stat_offset);
				
				if (reg_value < 0) {
					return;
				}

				intr_stat = (uint8_t)reg_value;

				/* Process only specific interrupts we have enabled. Ignore others*/
				intr_stat = intr_stat & g_intr_tbl[intr_num].mask;
				if (intr_stat) {
					int already_cleared;

	#ifdef	PRINT_ALL_INTR //(
					MHL_TX_DBG_ERR("INTR-%s = %02X\n",
						g_intr_tbl[intr_num].name, intr_stat);
	#else	//)( PRINT_ALL_INTR
					MHL_TX_DBG_INFO("INTR-%s = %02X\n",
						g_intr_tbl[intr_num].name, intr_stat);
	#endif	//) PRINT_ALL_INTR

					already_cleared = g_intr_tbl[intr_num].isr(hw_context, intr_stat);
					if (already_cleared >=0) {
						/*
						 * only clear the interrupts that were not cleared by the specific ISR.
						 */
						intr_stat &= ~already_cleared;
						if (intr_stat) {
							/* Clear interrupt since specific ISR did not. */
							mhl_tx_write_reg(hw_context,
								g_intr_tbl[intr_num].stat_page,
								g_intr_tbl[intr_num].stat_offset,
								intr_stat);
						}
					}
				}
			}
		}
#ifdef	PRINT_ALL_INTR //(
		/* Print all interrupt status irrespective of mask */
		else {
			uint8_t	intr_stat;
			/* Only for debugging
			 * Print other masked interrupts
			 * do not clear
			 */
			intr_stat = mhl_tx_read_reg(hw_context,
					g_intr_tbl[intr_num].stat_page,
					g_intr_tbl[intr_num].stat_offset);
			MHL_TX_DBG_ERR("INTN-%s = %02X\n",
					g_intr_tbl[intr_num].name, intr_stat);

		}
#endif	//) PRINT_ALL_INTR
	}
}

/* default callbacks */
int cb_enum_begin(void *context)
{
	MHL_TX_DBG_WARN("%senum begin%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
	return 0;
}

int cb_enum_item(void *context
					,uint16_t columns
					,uint16_t rows
					,uint8_t  bits_per_pixel
					,uint32_t  vertical_refresh_rate_in_milliHz
					,uint16_t  burst_id
					,video_burst_descriptor_u *p_descriptor)
{
	MHL_TX_DBG_WARN("%senum item%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
	return 0;
}

int cb_enum_end(void *context)
{
	MHL_TX_DBG_WARN("%senum end%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
	return 0;
}

void cb_hpd_driven_low(void *context)
{
	MHL_TX_DBG_WARN("%sdriven_low%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
}


hpd_high_callback_status cb_hpd_driven_high(
		 void *context
		,uint8_t *p_edid,size_t edid_length
		,MHL3_hev_dtd_item_t *p_hev_dtd		, size_t num_hev_dtds
		,MHL3_hev_vic_item_t *p_hev_vic		, size_t num_hev_vics
		,MHL3_3d_dtd_item_t *p_3d_dtd_items	, size_t num_3d_dtds
		,MHL3_3d_vic_item_t *p_3d_vic		, size_t num_3d_vics
		,avif_or_cea_861_dtd_u *p_avif_or_dtd, size_t avif_or_dtd_max_length
		,vsif_mhl3_or_hdmi_u *p_vsif, size_t vsif_max_length
		)
{
	MHL_TX_DBG_WARN("%sdriven_high%s\n",ANSI_ESC_GREEN_TEXT,ANSI_ESC_RESET_TEXT);
	return hpd_high_video_not_ready;
}

/*
 * si_mhl_tx_chip_initialize
 *
 * Chip specific initialization.
 * This function resets and initializes the transmitter and puts chip into sleep.
 * MHL Detection interrupt setups up the chip for video.
 */
int si_mhl_tx_chip_initialize(struct drv_hw_context *hw_context)
{
	int ret_val;
	int status = -1;

	//udelay(50);
	//mdelay(50);

	//set_pin(hw_context, TX_FW_WAKE, 1);/* inverter on board */
	platform_mhl_tx_hw_reset(TX_HW_RESET_PERIOD, TX_HW_RESET_DELAY);
	//mdelay(150);
	/* Power up the device */
	if(0>mhl_tx_write_reg(hw_context, REG_PAGE_0_DPD
					, BIT_PAGE_0_DPD_PWRON_PLL
					| BIT_PAGE_0_DPD_PDNTX12
					| BIT_PAGE_0_DPD_OSC_EN
					))
	{
		MHL_TX_DBG_ERR("write error i2c first time\n");
		if(0>mhl_tx_write_reg(hw_context, REG_PAGE_0_DPD
					, BIT_PAGE_0_DPD_PWRON_PLL
					| BIT_PAGE_0_DPD_PDNTX12
					| BIT_PAGE_0_DPD_OSC_EN
					))
		{
			MHL_TX_DBG_ERR("write error i2c the second time\n");
		}
	}
	MHL_TX_DBG_ERR("si_mhl_tx_chip_initialize.............\n");

	ret_val = get_device_rev(hw_context);
	hw_context->chip_rev_id = (uint8_t)ret_val;
	if (0 == hw_context->chip_rev_id) {
		MHL_TX_DBG_ERR("ES%s0.0%s is not supported\n"
						,ANSI_ESC_RED_TEXT
						,ANSI_ESC_RESET_TEXT);
		//return status;
	}

	ret_val = get_device_id(hw_context);
	if (ret_val > 0)
	{
		extern int	crystal_khz;

		hw_context->chip_device_id = (uint16_t)ret_val;

		MHL_TX_DBG_ERR("%x%s: Found SiI%04X rev: %01X.%01X%s\n"
				,hw_context
				,ANSI_ESC_GREEN_TEXT
				,hw_context->chip_device_id
				,hw_context->chip_rev_id >> 4
				,(hw_context->chip_rev_id & 0x0F)
				,ANSI_ESC_RESET_TEXT
				);
		/* Based on module parameter "crystal_khz=xxxxx" program registers. */
		program_ext_clock_regs(hw_context, crystal_khz);

		/*
		 * Store transmitter's KSV in case it's requested by
		 * someone else (probably the video source driver)
		 */
		mhl_tx_read_reg_block(hw_context, REG_PAGE_0_AKSV_1,
							5, hw_context->aksv);

		/* Move to disconnected state. Let RGND/MHL connection event start the driver */
		disconnect_mhl(hw_context,true);
		switch_to_d3(hw_context,false);
		hw_context->callbacks.display_timing_enum_begin = cb_enum_begin;
		hw_context->callbacks.display_timing_enum_item  = cb_enum_item;
		hw_context->callbacks.display_timing_enum_end   = cb_enum_end;
		hw_context->callbacks.hpd_driven_low			= cb_hpd_driven_low;
		hw_context->callbacks.hpd_driven_high			= cb_hpd_driven_high;

		status = 0;
	}

	return status;
}

void si_mhl_tx_drv_shutdown(struct drv_hw_context *hw_context)
{
	//set_pin(hw_context,TX_FW_WAKE, 1);
}

int si_mhl_tx_drv_connection_is_mhl3(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	return IN_MHL3_MODE(hw_context)?1:0;
}

int si_mhl_tx_drv_get_highest_tmds_link_speed(struct mhl_dev_context *dev_context)
{
	int link_speed = MHL_XDC_TMDS_000;

	if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_600) {
		link_speed = MHL_XDC_TMDS_600;
	} else if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_300) {
		link_speed = MHL_XDC_TMDS_300;
	} else if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_150) {
		link_speed = MHL_XDC_TMDS_150;
	}

	return link_speed;
}

void si_mhl_tx_drv_set_lowest_tmds_link_speed(struct mhl_dev_context *dev_context,
									uint32_t pixel_clock_frequency,
									uint8_t bits_per_pixel)
{
	uint32_t link_clock_frequency;
	uint32_t top_clock_frequency;
	uint8_t reg_val;
	bool found_fit;

	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	link_clock_frequency = pixel_clock_frequency * ((uint32_t)(bits_per_pixel >> 3));
	found_fit = false;

	/* find lowest fit giving up only if module parameter forces us into a supported link speed */
	if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_150) {
		MHL_TX_DBG_INFO("XDEVCAP TMDS Link Speed = 1.5Gbps is supported\n");
		top_clock_frequency = 147000000;	// 1500000 * 98;

		if (platform_get_flags() & PLATFORM_FLAG_1_5GBPS) {
			MHL_TX_DBG_INFO("Module parameter forcing TMDS Link Speed = 1.5Gbps\n");
			reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_1_5GBPS;
			goto set_tmds_speed;
		} else if ((link_clock_frequency <= 150000000) && (!found_fit)) {
   			MHL_TX_DBG_INFO("Mode fits TMDS Link Speed = 1.5Gbps (%d)\n", link_clock_frequency);
			reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_1_5GBPS;
			found_fit = true;
		}
	}
	if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_300) {
		MHL_TX_DBG_INFO("XDEVCAP TMDS Link Speed = 3.0Gbps is supported\n");
		top_clock_frequency = 294000000;	// 3000000 * 98;

		if (platform_get_flags() & PLATFORM_FLAG_3GBPS) {
			MHL_TX_DBG_INFO("Module parameter forcing TMDS Link Speed = 3.0Gbps\n");
			reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_3GBPS;
			goto set_tmds_speed;
		} else if ((link_clock_frequency <= 300000000) && (!found_fit)) {
			MHL_TX_DBG_INFO("Mode fits TMDS Link Speed = 3.0Gbps (%d)\n", link_clock_frequency);
			reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_3GBPS;
			found_fit = true;
		}
	}
	if (dev_context->xdev_cap_cache.mxdc.tmds_speeds & MHL_XDC_TMDS_600) {
		MHL_TX_DBG_INFO("XDEVCAP TMDS Link Speed = 6.0Gbps is supported\n");
		top_clock_frequency = 588000000; // 6000000 * 98;

		if (platform_get_flags() & PLATFORM_FLAG_6GBPS) {
			MHL_TX_DBG_INFO("Module parameter forcing TMDS Link Speed = 6.0Gbps\n");
			reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_6GBPS;
			goto set_tmds_speed;
		} else if ((link_clock_frequency <= 600000000) && (!found_fit)) {
			MHL_TX_DBG_INFO("Mode fit TMDS Link Speed = 6.0Gbps (%d)\n", link_clock_frequency);
			reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_6GBPS;
			found_fit = true;
		}
	}
	if (!found_fit) {
		MHL_TX_DBG_INFO("Cannot fit mode to any supported TMDS Link Speeds\n");
		MHL_TX_DBG_INFO("Forcing TMDS Link Speed = 6.0Gbps\n");
		reg_val = VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_6GBPS;
	}

	/* set tmds link speed */
	set_tmds_speed:
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL3_TX_ZONE_CTL, reg_val);

	/* set unlimited packet size only if not enough overhead */
	MHL_TX_DBG_WARN("lcf = %d, tcf = %d\n",link_clock_frequency ,top_clock_frequency);
	if (link_clock_frequency >= top_clock_frequency) {
		MHL_TX_DBG_WARN("Program 3E1[3] = 1 --- UNLIMITED MODE ON\n");
		mhl_tx_modify_reg(hw_context, REG_PAGE_3_M3_P0CTRL
						, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PORT_EN
						| BIT_PAGE_3_M3_P0CTRL_MHL3_P0_UNLIMIT_EN
						, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PORT_EN
						| BIT_PAGE_3_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_PP);
	}
	else {
		MHL_TX_DBG_WARN("Program 3E1[3] = 0 --- UNLIMITED MODE OFF\n");
		mhl_tx_modify_reg(hw_context, REG_PAGE_3_M3_P0CTRL
						, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PORT_EN
						| BIT_PAGE_3_M3_P0CTRL_MHL3_P0_UNLIMIT_EN
						, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PORT_EN
						| BIT_PAGE_3_M3_P0CTRL_MHL3_P0_UNLIMIT_EN_NORM);
	}

	/* set write stat indicating this change */
	switch (reg_val) {
		case VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_6GBPS:
			si_mhl_tx_set_status(dev_context, true, MHL_STATUS_REG_AV_LINK_MODE_CONTROL, 0x02); // 0b000 = 1.5Gbps, 0b001 = 3Gbps, 0b010 = 6.0Gbps
			break;
		case VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_3GBPS:
			si_mhl_tx_set_status(dev_context, true, MHL_STATUS_REG_AV_LINK_MODE_CONTROL, 0x01); // 0b000 = 1.5Gbps, 0b001 = 3Gbps, 0b010 = 6.0Gbps
			break;
		case VAL_PAGE_3_TX_ZONE_CTL3_TX_ZONE_1_5GBPS:
			si_mhl_tx_set_status(dev_context, true, MHL_STATUS_REG_AV_LINK_MODE_CONTROL, 0x00); // 0b000 = 1.5Gbps, 0b001 = 3Gbps, 0b010 = 6.0Gbps
			break;
	}
}

bool si_mhl_tx_drv_support_e_cbus_d(struct drv_hw_context *hw_context)
{
	return false;
}

int si_mhl_tx_drv_cbus_ready_for_edid(struct mhl_dev_context *dev_context)
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	switch (hw_context->cbus_mode) {
    case CM_oCBUS_PEER_IS_MHL1_2:
    case CM_eCBUS_S:
    case CM_eCBUS_D:
		return 1;
	default:
		return 0;
	}
}

uint16_t si_mhl_tx_drv_get_blk_rcv_buf_size(void)
{
	return 288;
}

int si_mhl_tx_drv_start_avlink_bist(struct drv_hw_context *hw_context,
								struct bist_setup_info *test_info)
{
	hpd_control_mode mode;
	uint32_t	frame_count = 0;
	uint8_t		start_cmd;
	uint8_t		bist_mode_sel;

	switch (test_info->avlink_video_mode) {
	case 4: /* 1280 X 720 (720P) */
		MHL_TX_DBG_ERR("AV LINK_VIDEO_MODE 720p60\n");
		bist_mode_sel = 5;
		break;
	case 3: /* 720 X 480 (480P) */
		MHL_TX_DBG_ERR("AV LINK_VIDEO_MODE 480p60\n");
		bist_mode_sel = 6;
		break;
	default:
		MHL_TX_DBG_ERR("Unsupported VIC received!\n");
		return -1;
	}
	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_VIDEO_MODE, bist_mode_sel);

	mhl_tx_write_reg(hw_context,TX_PAGE_0, 0x0B, 0xFE); // DPD Register
	mhl_tx_write_reg(hw_context,TX_PAGE_0, 0xF7, 0x00); // Test Control Register
	mhl_tx_write_reg(hw_context,TX_PAGE_0, 0x0D, 0x02); // Dual link Control Register
	mhl_tx_write_reg(hw_context,TX_PAGE_3, 0x39, 0x00); // disable clock detect
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0 ,0xF0);		/* 0x0331 */
	mhl_tx_write_reg(hw_context,TX_PAGE_3, 0xE1, 0x01); // HDMI2MHL3 Port0 Control Register
	mhl_tx_write_reg(hw_context,TX_PAGE_7, 0x17, 0x01); // CoC 8th Ctl Register
	mhl_tx_write_reg(hw_context,TX_PAGE_7, 0x21, 0xF8); // CoC 18th Ctl Register
	mhl_tx_write_reg(hw_context,TX_PAGE_3, 0x39, 0x80); // enable clock detect
	mhl_tx_write_reg(hw_context,TX_PAGE_7, 0x10, 0x19); // enable auto calibration, 0x58 disable, LFP enable

	switch (test_info->avlink_data_rate) {
	case 1:	/* 1.5 Gbps */
		MHL_TX_DBG_ERR("AV LINK_DATA_RATE 1.5Gbps\n");
		bist_mode_sel = 0x02;
		break;
	case 2: /* 3.0 Gbps */
		MHL_TX_DBG_ERR("AV LINK_DATA_RATE 3.0Gbps\n");
		bist_mode_sel = 0x01;
		break;
	case 3: /* 6.0 Gbps */
		MHL_TX_DBG_ERR("AV LINK_DATA_RATE 6.0Gbps\n");
		bist_mode_sel = 0x00;
		break;
	default:
		MHL_TX_DBG_ERR("Unsupported AVLINK_DATA_RATE %02X\n",
												test_info->avlink_data_rate);
		return -1;
	}
	mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL3_TX_ZONE_CTL, bist_mode_sel);

	switch (test_info->avlink_pattern) {
	case BS_AV_PATTERN_UNSPECIFIED:
	case BS_AV_PATTERN_PRBS:
		MHL_TX_DBG_ERR("AV LINK_PATTERN PRBS\n");
		mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_TEST_SEL, 0x03);
		break;

	case BS_AV_PATTERN_FIXED_8:
		MHL_TX_DBG_ERR("AV LINK_PATTERN Fixed8\n");
		mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_8BIT_PATTERN,
						 test_info->avlink_fixed_l);

		mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_TEST_SEL, 0x09);
		break;

	case BS_AV_PATTERN_FIXED_10:
		MHL_TX_DBG_ERR("AV LINK_PATTERN Fixed10\n");
		mhl_tx_write_reg(hw_context,TX_PAGE_2, 0xF3, 0x01); // //reg_txinstruction[7:0]
				mhl_tx_write_reg(hw_context,TX_PAGE_2, 0xF4, 0x07); // //reg_txinstruction[10:8]
		mhl_tx_write_reg(hw_context,TX_PAGE_2, 0xF5, test_info->avlink_fixed_l); // //reg_txpattern[7:0]
		{
			uint8_t STX_1,STX_0,STX,E98_pre, pat_lsb, pat_msb;
			pat_lsb = test_info->avlink_fixed_l;
			pat_msb = test_info->avlink_fixed_h;
			STX_1 = ((pat_lsb>>1)^(pat_lsb>>3)^(pat_lsb>>4)^(pat_lsb>>5)^(pat_lsb>>7))&0x01;
			STX_0 = ((pat_lsb>>0)^(pat_lsb>>2)^(pat_lsb>>4)^(pat_lsb>>6))&0x01;
			STX = (STX_1<<1)|STX_0;
			E98_pre = ((pat_msb)^STX)&0x3;
					mhl_tx_write_reg(hw_context,TX_PAGE_2, 0xF6, E98_pre); // //reg_txpattern[9:8]
		}
		mhl_tx_write_reg(hw_context,TX_PAGE_2, 0xF2, 0x03); // //Start the BIST
		break;

	default:
		MHL_TX_DBG_ERR("Unrecognized test pattern detected!\n");
		return -1;
	}

	mode = platform_get_hpd_control_mode();

	if (HPD_CTRL_OPEN_DRAIN == mode)
		mhl_tx_write_reg(hw_context, REG_PAGE_0_HPD_CTRL, BITS_HPD_CTRL_OPEN_DRAIN_HIGH);
	else if (HPD_CTRL_PUSH_PULL == mode)
		mhl_tx_write_reg(hw_context, REG_PAGE_0_HPD_CTRL, BITS_HPD_CTRL_PUSH_PULL_HIGH);



	if (test_info->avlink_duration) {
		if (test_info->avlink_pattern == BS_AV_PATTERN_FIXED_8)
			frame_count = test_info->avlink_duration * 32;
	}

	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_DURATION_0,
					 (uint8_t)frame_count);
	frame_count >>= 8;
	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_DURATION_1,
					 (uint8_t)frame_count);
	frame_count >>= 8;
	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_DURATION_2,
					 (uint8_t)frame_count);

	start_cmd = BIT_PAGE_0_BIST_CTRL_BIST_EN |
				BIT_PAGE_0_BIST_CTRL_BIST_ALWAYS_ON |
				BIT_PAGE_0_BIST_CTRL_BIST_TRANS;
	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_CTRL, start_cmd);

	start_cmd |= BIT_PAGE_0_BIST_CTRL_BIST_START_BIT;
	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_CTRL, start_cmd);

	return 0;
}

int si_mhl_tx_drv_stop_avlink_bist(struct drv_hw_context *hw_context)
{
	mhl_tx_write_reg(hw_context, REG_PAGE_0_BIST_CTRL, 0x00);
	//Stop IPBIST Fix10.
	mhl_tx_modify_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CNTLSTA,
							BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
							BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_EN, 0x00);

	return 0;
}

int si_mhl_tx_drv_start_ecbus_bist(struct drv_hw_context *hw_context,
									struct bist_setup_info *test_info)
{
	uint8_t	start_cmd;

	if (test_info->bist_trigger_cmd & BIST_TRIGGER_TEST_E_CBUS_D) {
		/* Clear eCBUS-D BIST error counter and status */
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL7, 0x60);

		switch (test_info->e_cbus_pattern) {
		case BS_PATTERN_UNSPECIFIED:
		case BS_PATTERN_PRBS:
			start_cmd = 0x01;
			break;

		case BS_PATTERN_FIXED_8:
			mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL9,
							 test_info->e_cbus_fixed_l);

			if (test_info->e_cbus_fixed_h & 0x80) {
				mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLA,
								 ~test_info->e_cbus_fixed_l);
			} else {
				mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLA,
								 test_info->e_cbus_fixed_l);
			}
			start_cmd = 0x02;
			break;

		case BS_PATTERN_FIXED_10:
			mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL9,
							 test_info->e_cbus_fixed_l);
			mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL8,
							 test_info->e_cbus_fixed_h & 0x03);

			start_cmd = test_info->e_cbus_fixed_h << 2;
			if (test_info->e_cbus_fixed_h & 0x80) {
				mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLA,
								 ~test_info->e_cbus_fixed_l);
				mhl_tx_modify_reg(hw_context, REG_PAGE_7_DOC_CTL8,
								  0x0C, ~start_cmd);
			} else {
				mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLA,
								 test_info->e_cbus_fixed_l);
				mhl_tx_modify_reg(hw_context, REG_PAGE_7_DOC_CTL8,
								  0x0C, start_cmd);
			}
			start_cmd = 0x03;
			break;
		default:
			MHL_TX_DBG_ERR("Unrecognized test pattern detected!\n");
			return -1;
		}

		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL7, start_cmd);

	} else {

		/* Clear eCBUS-S BIST error counter and status */
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7, 0x60);

		switch (test_info->e_cbus_pattern) {
		case BS_PATTERN_UNSPECIFIED:
		case BS_PATTERN_PRBS:
			start_cmd = 0x01;
			break;

		case BS_PATTERN_FIXED_8:
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL9,
							 test_info->e_cbus_fixed_l);
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLA,
							 test_info->e_cbus_fixed_l);

			start_cmd = 0x02;
			break;

		case BS_PATTERN_FIXED_10:
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL9,
							 test_info->e_cbus_fixed_l);
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL8,
							 test_info->e_cbus_fixed_h & 0x03);

			start_cmd = test_info->e_cbus_fixed_h << 2;
			if (test_info->e_cbus_fixed_h & 0x80) {
				mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLA,
								 ~test_info->e_cbus_fixed_l);
				mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL8,
								  0x0C, ~start_cmd);
			} else {
				mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLA,
								 test_info->e_cbus_fixed_l);
				mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL8,
								  0x0C, start_cmd);
			}
			start_cmd = 0x03;
			break;
		default:
			MHL_TX_DBG_ERR("Unrecognized test pattern detected!\n");
			return -1;
		}
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7, start_cmd);

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXNUMB,0x05);

		mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL0,BIT_PAGE_7_COC_CTL0_COC_CONTROL0_2,0x00);
		if (test_info->bist_trigger_cmd & BIST_TRIGGER_E_CBUS_RX)
		{
			mhl_tx_write_reg(hw_context,TX_PAGE_0, 0x0E, 0xA0); // COC_DOC s/w reset
			mhl_tx_write_reg(hw_context,TX_PAGE_0, 0x0E, 0x20); // COC_DOC s/w reset
#ifdef CTS_WORKAROUND
			msleep(10);
#endif
			mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL0,BIT_PAGE_7_COC_CTL0_COC_CONTROL0_2,BIT_PAGE_7_COC_CTL0_COC_CONTROL0_2);
			MHL_TX_DBG_ERR("REG_PAGE_7_COC_CTL0 = 0x%02x \n", mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_CTL0));
		}
		else
		{
			mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL14,MSK_PAGE_7_COC_CTL14_COC_CONTROL14_3_0,0x02);
			mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL15,BIT_PAGE_7_COC_CTL15_COC_CONTROL15_7,BIT_PAGE_7_COC_CTL15_COC_CONTROL15_7);
		}
	}
	return 0;
}

int si_mhl_tx_drv_stop_ecbus_bist(struct drv_hw_context *hw_context,
									struct bist_setup_info *test_info)
{
	int	err_cnt;

	if (test_info->bist_trigger_cmd & BIST_TRIGGER_TEST_E_CBUS_D) {
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL7, 0x00);
		err_cnt = mhl_tx_read_reg(hw_context, REG_PAGE_7_DOC_STAT_9)
				& MSK_PAGE_7_DOC_STAT_9_DOC_STATUS9_3_0;
		err_cnt <<= 8;
		err_cnt |= mhl_tx_read_reg(hw_context, REG_PAGE_7_DOC_STAT_8);
	} else {
		//Trigger to update error counter
		mhl_tx_modify_reg(hw_context, REG_PAGE_7_COC_CTL15, MSK_PAGE_7_COC_CTL15_COC_CONTROL15_6_4,0x00);
		err_cnt = mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_4);

		err_cnt <<= 8;
		err_cnt |= mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_5);
		MHL_TX_DBG_INFO("eCBUS State Machine: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n",
			mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_0),
			mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_1),
			mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_2),
			mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_3),
			mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_4),
			mhl_tx_read_reg(hw_context, REG_PAGE_7_COC_STAT_5));

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXNUMB, 0x0C);

		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7, 0x00);
	}

	return err_cnt;
}

int si_mhl_tx_drv_start_impedance_bist(struct drv_hw_context *hw_context,
										struct bist_setup_info *test_info)
{
	int	status = 0;

	switch (test_info->impedance_mode) {
	case IMP_MODE_AV_LINK_TX_LOW:
	case IMP_MODE_AV_LINK_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_INST_LOW, 0x01);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_INST_HIGH, 0x07);
		if (test_info->impedance_mode == IMP_MODE_AV_LINK_TX_LOW) {
			mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_PAT_LOW, 0x00);
			mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_PAT_HIGH, 0x00);
		} else {
			mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_PAT_LOW, 0xFF);
			mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_PAT_HIGH, 0x01);

		}
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CNTLSTA,
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_EN);
		break;

	case IMP_MODE_E_CBUS_S_TX_LOW:
	case IMP_MODE_E_CBUS_S_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_PAGE_0_TEST_TXCTRL, 0x81);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL2, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_CTRL, 0x0E);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2, 0x0B);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5, 0x3D);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0, 0xF0);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL0, 0xC5);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, 0x18);

		if (test_info->impedance_mode == IMP_MODE_E_CBUS_S_TX_LOW) {
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1, 0x00);
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL2, 0x00);
		} else {
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1, 0x28);
			mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL2, 0x28);
		}

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXNUMB, 0x05);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLB, 0xFF);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLC, 0xFF);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLD, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLE, 0x18);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_ALICE0_ZONE_CTRL, 0xE8);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_BGR_BIAS, 0x87);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_ALICE0_ZONE_CTRL, 0xD0);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL3, 0x3F);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL6, 0x10);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL1, 0xCD);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL4, 0x2A);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL2, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_SCTRL, 0x40);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_P0CTRL, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_PORT_EN);

		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_INST_LOW, 0x01);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_INST_HIGH, 0x03);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_PAT_LOW, 0xFF);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_PAT_HIGH, 0x03);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CONF_LOW, 0x08);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CONF_HIGH, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CNTLSTA,
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_EN);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CNTLSTA,
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_SEL |
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_EN |
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_RUN |
						 BIT_PAGE_2_TX_IP_BIST_CNTLSTA_TXBIST_ON);

		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL11, 0xF0);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7, 0x01);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL14, 0x0C);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL15, 0x80);
		break;

	case IMP_MODE_E_CBUS_D_TX_LOW:
	case IMP_MODE_E_CBUS_D_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_PAGE_0_TEST_TXCTRL, 0x01);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL0, 0x02);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_CTRL, 0x0E);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_TOP_CTL, 0x82);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0, 0xF3);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL1, 0x07);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DOC_CTL0, 0x81);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL3_TX_ZONE_CTL, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2, 0x0B);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5, 0x3D);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL0, 0x02);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL7, 0x06);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLE, 0x0F);

		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CFG4, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CFG4, 0x08);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CFG4, 0x09);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CFG4, 0x0A);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CFG4, 0x0B);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CFG4, 0x02);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x80);
		if (test_info->impedance_mode == IMP_MODE_E_CBUS_D_TX_HIGH)
			mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL6, 0x60);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL7, 0x1E);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLE, 0x0F);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL0, 0x60);
		break;

	default:
		MHL_TX_DBG_ERR("Invalid value 0x%02x specified in "\
						"IMPEDANCE_MODE field\n",
						test_info->impedance_mode);
		status = -EINVAL;
	}
	return status;
}

void si_mhl_tx_drv_stop_impedance_bist(struct drv_hw_context *hw_context,
										struct bist_setup_info *test_info)
{
	switch (test_info->impedance_mode) {
	case IMP_MODE_AV_LINK_TX_LOW:
	case IMP_MODE_AV_LINK_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CNTLSTA, 0);
		break;

	case IMP_MODE_E_CBUS_S_TX_LOW:
	case IMP_MODE_E_CBUS_S_TX_HIGH:
		/* Restore previous values */
		mhl_tx_write_reg(hw_context, REG_PAGE_0_TEST_TXCTRL, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL2, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_CTRL, 0x07);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2, 0x2F);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5, 0x3F);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0, 0xBC);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL0, 0xC3);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL0, 0x5C);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL1, 0x0A);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL2, 0x14);

		mhl_tx_write_reg(hw_context, REG_PAGE_1_TTXNUMB, 0x04);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLB, 0xFF);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLC, 0xFF);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLD, 0x21);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTLE, 0x18);

		mhl_tx_write_reg(hw_context, REG_PAGE_2_ALICE0_ZONE_CTRL, 0xE8);
		mhl_tx_write_reg(hw_context, REG_PAGE_2_BGR_BIAS, 0x87);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL3, 0x40);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL6, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL1, 0xC7);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_COC_CTL4, 0x2A);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL2, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_SCTRL, 0x41);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_P0CTRL, BIT_PAGE_3_M3_P0CTRL_MHL3_P0_HDCP_EN);

		mhl_tx_write_reg(hw_context, REG_PAGE_2_TX_IP_BIST_CNTLSTA, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL7, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_COC_CTL14, 0x00);
		break;

	case IMP_MODE_E_CBUS_D_TX_LOW:
	case IMP_MODE_E_CBUS_D_TX_HIGH:
		mhl_tx_write_reg(hw_context, REG_PAGE_0_TEST_TXCTRL, 0x80);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_PLL_CTL0, 0x07);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_M3_CTRL, 0x07);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_TOP_CTL, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL0, 0xBC);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DOC_CTL0, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL2, 0x2F);
		mhl_tx_write_reg(hw_context, REG_PAGE_3_MHL_DP_CTL5, 0x3F);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL0, 0x40);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTL7, 0x00);
		mhl_tx_write_reg(hw_context, REG_PAGE_7_DOC_CTLE, 0x00);
	}
}

void si_mhl_tx_drv_leave_bist_mode(struct drv_hw_context *hw_context)
{
	disconnect_mhl(hw_context,true);
}

int si_mhl_tx_drv_sample_edid_buffer(struct drv_hw_context *hw_context,uint8_t *edid_buf)
{
	mhl_tx_write_reg(hw_context, REG_PAGE_2_EDID_FIFO_ADDR, 0);
	mhl_tx_read_reg_block(hw_context
		, REG_PAGE_2_EDID_FIFO_RD_DATA
		, 256
		, edid_buf);
	return 0;
}

void	si_dump_important_regs(struct drv_hw_context *hw_context)
{
	int i=0;
	uint8_t	registers[64] = {
		 REG_PAGE_0_DEV_REV
		,REG_PAGE_0_OTP_DBYTE510
		,REG_PAGE_0_DPD
		,REG_PAGE_0_TMDS_CCTRL
		,REG_PAGE_0_TMDS_CTRL4
		,REG_PAGE_1_TRXCTRL
		,REG_PAGE_0_OTP_DBYTE510
		,REG_PAGE_5_MHL_INT_0_MASK
		,REG_PAGE_5_MHL_INT_1_MASK
		,REG_PAGE_5_MHL_INT_2_MASK
		,REG_PAGE_5_MHL_INT_3_MASK
		,REG_PAGE_5_MDT_RCV_TIMEOUT
		,REG_PAGE_5_MDT_XMIT_TIMEOUT
		,REG_PAGE_5_MDT_RCV_CONTROL
		,REG_PAGE_5_MDT_XMIT_CONTROL
		,REG_PAGE_5_MDT_RFIFO_STAT
		,REG_PAGE_5_MDT_XFIFO_STAT
		,REG_PAGE_5_MDT_INT_0
		,REG_PAGE_5_MDT_INT_0_MASK
		,REG_PAGE_5_MDT_INT_1_MASK
		,REG_PAGE_5_CBUS_STATUS
		,REG_PAGE_5_DDC_ABORT_INT_MASK
		,REG_PAGE_5_MSC_MT_ABORT_INT_MASK
		,REG_PAGE_5_MSC_MR_ABORT_INT_MASK
		,REG_PAGE_5_CBUS_RX_DISC_INT0_MASK
		,0x00,0x00
	};
	printk("Important Registers:\n");
	while(registers[i]) {
		printk("%d:%02X = %02X\n", registers[i], registers[i+1], mhl_tx_read_reg(hw_context, registers[i], registers[i+1]));
		i += 2;
	}
	printk("\nDump PAGE_1:\n");
	for(i=0;i<256;i++) {
		printk("%02X  ", mhl_tx_read_reg(hw_context,TX_PAGE_1,(uint8_t)i));
		if(0x0f == (0x0f & i)) {
			printk("\n");
		}
	}
	printk("\nDump PAGE_3:\n");
	for(i=0;i<256;i++) {
		printk("%02X  ", mhl_tx_read_reg(hw_context,TX_PAGE_3,(uint8_t)i));
		if(0x0f == (0x0f & i)) {
			printk("\n");
		}
	}
	printk("\nDump PAGE_5:\n");
	for(i=0;i<256;i++) {
		printk("%02X  ", mhl_tx_read_reg(hw_context,TX_PAGE_5,(uint8_t)i));
		if(0x0f == (0x0f & i)) {
			printk("\n");
		}
	}
	printk("\nDump PAGE_7:\n");
	for(i=0;i<256;i++) {
		printk("%02X  ", mhl_tx_read_reg(hw_context,TX_PAGE_7,(uint8_t)i));
		if(0x0f == (0x0f & i)) {
			printk("\n");
		}
	}
}
