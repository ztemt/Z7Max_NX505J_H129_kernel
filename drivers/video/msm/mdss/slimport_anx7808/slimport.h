/*
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SLIMPORT_H
#define _SLIMPORT_H

#define DEBUG
#define DEV_TAG "slimport: "


#ifdef DEBUG
#ifndef DEV_DBG
#define DEV_DBG(args...) pr_notice(DEV_TAG args)
#endif
#else
#ifndef DEV_DBG
#define DEV_DBG(args...) (void)0
#endif
#endif

#ifndef DEV_NOTICE
#define DEV_NOTICE(args...) pr_notice(DEV_TAG args)
#endif
#ifndef DEV_ERR
#define DEV_ERR(args...) pr_notice(DEV_TAG args)
#endif


#define SSC_EN

#if 0
#define SSC_1
#define EYE_TEST
#define EDID_DEBUG_PRINT
#endif

#define AUX_ERR 1
#define AUX_OK 0

extern bool sp_tx_hw_lt_done;
extern bool sp_tx_hw_lt_enable;
extern bool	sp_tx_link_config_done ;
extern enum SP_TX_System_State sp_tx_system_state;
extern enum RX_CBL_TYPE sp_tx_rx_type;
extern enum RX_CBL_TYPE sp_tx_rx_type_backup;
extern unchar sp_tx_pd_mode;
extern enum SP_TX_System_State sp_tx_system_state_bak;

#ifdef CONFIG_SLIMPORT_FAST_CHARGE
enum CHARGING_STATUS {
	NO_CHARGING = 0x00,
	NORMAL_CHARGING = 0x01,
	FAST_CHARGING = 0x02
};
/*for ccharging status*/
extern enum CHARGING_STATUS downstream_charging_status;
enum CHARGING_STATUS sp_get_ds_charge_type(void);
void sp_set_ds_charge_type(enum CHARGING_STATUS chg_type);
#endif

extern unchar bedid_break;
extern unchar sp_tx_hw_hdcp_en;
/*extern struct i2c_client *anx7808_client;*/

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
void sp_tx_hardware_poweron(void);
void sp_tx_hardware_powerdown(void);
int slimport_read_edid_block(void *edid_ctrl, int block, uint8_t *edid_buf);
enum SP_LINK_BW slimport_get_link_bw(void);
enum RX_CBL_TYPE sp_get_ds_cable_type(void);
#endif
