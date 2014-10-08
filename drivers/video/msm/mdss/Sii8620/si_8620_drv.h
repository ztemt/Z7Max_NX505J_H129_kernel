/*

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT. See
the GNU General Public License for more details at
http://www.gnu.org/licenses/gpl-2.0.html.

*/

#if !defined(SI_8620_DRV_H)
#define SI_8620_DRV_H

struct drv_hw_context {
	struct interrupt_info *intr_info;
	uint8_t		chip_rev_id;
	uint16_t	chip_device_id;
	uint8_t		cbus_status;
	uint8_t		gen2_write_burst_rcv;
	uint8_t		gen2_write_burst_xmit;
	uint8_t		hawb_write_pending;
	enum{
		 cbus1_idle_rcv_disabled
		,cbus1_idle_rcv_enabled
		,cbus1_idle_rcv_pending
		,cbus1_msc_pending_delayed_rcv_enable
		,cbus1_msc_pending_rcv_disabled
		,cbus1_xmit_pending_xmit_and_rcv_enabled
		,cbus1_xmit_pending_xmit_and_rcv_pending
	}cbus1_state;
	uint8_t		delayed_hawb_enable_reg_val;
	uint8_t		video_path;
	uint8_t		video_ready;
	uint8_t		mhl_peer_version_stat;
	enum cbus_mode_e cbus_mode;
	uint8_t		current_edid_request_block;
	uint8_t		edid_fifo_block_number;
	uint8_t		valid_vsif;
	uint8_t		valid_avif;
	uint8_t		rx_hdmi_ctrl2_defval;
	uint8_t		aksv[5];
	avi_info_frame_t		current_avi_info_frame;
	vendor_specific_info_frame_t	current_vs_info_frame;
	hw_avi_payload_t	outgoingAviPayLoad;
	mhl3_vsif_t			outgoing_mhl3_vsif;
	uint8_t		write_burst_data[MHL_SCRATCHPAD_SIZE];
	struct 		cbus_req current_cbus_req;
	uint8_t		tdm_virt_chan_slot_counts[VC_MAX];
	struct		{
		uint16_t received_byte_count;
		unsigned long peer_blk_rx_buffer_avail;

#define NUM_BLOCK_INPUT_BUFFERS 8
		uint8_t		input_buffers[NUM_BLOCK_INPUT_BUFFERS][256];
		int			input_buffer_lengths[NUM_BLOCK_INPUT_BUFFERS];
		uint16_t	head;
		uint16_t	tail;
	}block_protocol;
	si_mhl_callback_api_t	callbacks;
	avif_or_cea_861_dtd_u	avif_or_dtd_from_callback;
	vsif_mhl3_or_hdmi_u 	vsif_mhl3_or_hdmi_from_callback;
	int			hpd_high_callback_status;
#ifdef MANUAL_EDID_FETCH //(
	uint8_t		edid_block[EDID_BLOCK_SIZE];
#endif //)
};

bool si_mhl_tx_set_status(struct mhl_dev_context *dev_context,
					bool xstat, uint8_t reg_to_write, uint8_t value);

void *si_mhl_tx_get_sub_payload_buffer(struct mhl_dev_context *dev_context
									,uint8_t size);

bool si_mhl_tx_send_write_burst(struct mhl_dev_context *dev_context, void *buffer);

int si_mhl_tx_drv_cbus_ready_for_edid(struct mhl_dev_context *dev_context);

int si_mhl_tx_drv_set_display_mode(struct mhl_dev_context *dev_context
				,hpd_high_callback_status status);

#endif /* if !defined(SI_8620_DRV_H) */
