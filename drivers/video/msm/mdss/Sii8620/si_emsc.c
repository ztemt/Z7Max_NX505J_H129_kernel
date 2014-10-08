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

/**
   @file si_emsc.c
*/
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/hrtimer.h>
#include "si_fw_macros.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#include "si_mdt_inputdev.h"
#include "mhl_linux_tx.h"
#include "platform.h"
#ifdef KERNEL_2_6_38_AND_LATER
#include <linux/input/mt.h>
#endif
#ifdef DEBUG
#include <linux/kernel.h>
#endif

#include "mhl_supp.h"
#include "si_mhl_callback_api.h"		//TODO: This is a terrible header file
#include "si_8620_drv.h"

/*
 * Send Host role request or relinquish it.  This should be called with
 * MHL_RHID_REQUEST_HOST for a source during MHL connection and when the
 * want_host flag is set (after the sink has relinquished the Host role that
 * we let them have by sending MHL_RHID_RELIQUISH_HOST).
 */
void mhl_tx_hid_host_role_request( struct mhl_dev_context *context, int request)
{
	/* During a request, we are in limbo	*/
	context->mhl_emsc.is_host = false;
	context->mhl_emsc.is_device = false;
	context->mhl_emsc.want_host = false;

	si_mhl_tx_send_msc_msg(context, MHL_MSC_MSG_RHID, request );
}

/*
 * Handle host-device negotiation request messages received from peer.
 *
 * A sink requesting Host role is typically just trying to determine
 * if the source has already requested the Host role, which is required
 * of the sink before starting the Device role (14.3.1.1).
 * As a source, we should have sent a host role request to the sink
 * before this (dev_context->mhl_emsc.is_host == true), so host
 * requests should be refused.
 *
 * When the sink relinquishes the Host role (assuming we let the sink have
 * it for some reason), we immediately want it back.
 */
void mhl_tx_hid_host_negotiation( struct mhl_dev_context *context )
{
	uint8_t	rhidk_status = MHL_RHID_NO_ERR;

	if ( context->msc_msg_sub_command == MHL_MSC_MSG_RHID ) {
		if ( context->msc_msg_data == MHL_RHID_REQUEST_HOST ) {
			if ( context->mhl_emsc.is_host )
				rhidk_status = MHL_RHID_DENY;
		} else if ( context->msc_msg_data == MHL_RHID_RELINQUISH_HOST ) {
			/* Since we're a source, assume that we want the host role back */
			context->mhl_emsc.want_host = true;
		}
		else {
			rhidk_status = MHL_RHID_INVALID;
		}

		MHL_TX_DBG_INFO("RHID: Received HID Host role %s result: %d\n",
				(context->msc_msg_data == MHL_RHID_REQUEST_HOST) ?
						"REQUEST" : "RELINQUISH", rhidk_status );

		/* Always RHIDK to the peer */
		si_mhl_tx_send_msc_msg(context, MHL_MSC_MSG_RHIDK, rhidk_status);

	} else if ( context->msc_msg_sub_command == MHL_MSC_MSG_RHIDK ) {
		if (context->msc_msg_data == MHL_RHID_NO_ERR){
			if (context->msc_msg_last_data == MHL_RHID_REQUEST_HOST ) {
				context->mhl_emsc.is_host = true;
				context->mhl_emsc.is_device = false;
				MHL_TX_DBG_INFO("RHIDK: HID HOST role granted\n");
			}
			else {
				context->mhl_emsc.is_host = false;
				context->mhl_emsc.is_device = true;
				MHL_TX_DBG_INFO("RHIDK: HID DEVICE role granted\n");
			}
		}
		else if (context->msc_msg_last_data == MHL_RHID_REQUEST_HOST )
		{
			/* In case we are denied; but this should not happen for a source */
			context->mhl_emsc.is_host = false;
			context->mhl_emsc.is_device = true;
			MHL_TX_DBG_INFO("RHIDK: HID HOST role DENIED\n");
		}
	}
}

/*
 * Add a BLOCK command to the driver eMSC queue
 */

#define HID_BURST_ID_LEN			(2 + 1)		// BURST_ID + HID_PAYLOAD_LEN
#define STD_TRANSPORT_HEADER_LEN	(2)			// unload/remaining counts
#define HID_MSG_HEADER_LEN			(2)			// Device ID/Ack req/msg_cnt
#define HID_FRAG_HEADER_LEN			(1)
#define HID_MSG_CHKSUM_LEN			(2)			// message checksum
#define BLOCK_CMD_LEN_MAX			(256)
#define BLOCK_CMD_PAYLOAD_LEN_MAX	(256 - STD_TRANSPORT_HEADER_LEN)

/*
 * Add a HID message to the eMSC output queue using one or more
 * eMSC BLOCK commands.
 */
int si_mhl_tx_emsc_add_hid_message( struct mhl_dev_context *context,
		uint8_t hb0, uint8_t hb1, uint8_t *msg_body, int msg_body_length )
{
	int 	i, cmd_size, fragment_count, msg_index, payload_index;
	uint8_t	*payload;
	uint8_t	payload_size;
	uint16_t	accum;
	uint16_t	*pchksum;
	bool	first_fragment;

	printk( "Lee: %s\n", __func__ );
	dump_array( "Submitted message", msg_body, msg_body_length );

	/* Assume message will require its own BLOCK command buffer	*/
	cmd_size =
		STD_TRANSPORT_HEADER_LEN +
		HID_BURST_ID_LEN +
		HID_FRAG_HEADER_LEN +
		HID_MSG_HEADER_LEN +
		HID_MSG_CHKSUM_LEN +
		msg_body_length;

	/* If message exceeds one (empty) BLOCK command buffer, we must
	 * break it into fragments.  Since the first fragment will be
	 * the full command buffer size, the current request (if any) will be
	 * sent before starting to add this one.
	 */
	fragment_count = 0;
	/* Determine fragment count	*/
	while (cmd_size > 0) {
		if ( cmd_size > BLOCK_CMD_LEN_MAX ) {
			fragment_count++;
			cmd_size -= BLOCK_CMD_LEN_MAX;
			if ( cmd_size > 0 ) {
				cmd_size += STD_TRANSPORT_HEADER_LEN +
							HID_BURST_ID_LEN +
							HID_FRAG_HEADER_LEN +
							HID_MSG_CHKSUM_LEN;
			}
		} else {
			cmd_size = 0;
		}
	}

	/* This time don't include the standard header in the message length. */
	cmd_size =
		HID_BURST_ID_LEN +
		HID_FRAG_HEADER_LEN +
		HID_MSG_HEADER_LEN +
		HID_MSG_CHKSUM_LEN +
		msg_body_length;

	/* Add one or more fragments to outgoing requests. */
	first_fragment = true;
	msg_index = 0;
	payload_index = 0;
	accum = 0;

	payload_index = 0;
	while ( fragment_count >= 0) {

		payload_size = (cmd_size > BLOCK_CMD_PAYLOAD_LEN_MAX) ?
				BLOCK_CMD_PAYLOAD_LEN_MAX  : cmd_size;

		payload = (uint8_t	*)si_mhl_tx_get_sub_payload_buffer(context, payload_size );
		if (payload == NULL){
			MHL_TX_DBG_ERR("%ssi_mhl_tx_get_sub_payload_buffer failed%s\n",
					ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
			//TODO: Should be handled with an error code and exit, but need to clean up any buffers thatmay have been successfully allocated.
		} else {
			payload[payload_index++] = (uint8_t)(burst_id_HID_PAYLOAD >> 8);
			payload[payload_index++] = (uint8_t)(burst_id_HID_PAYLOAD);
			payload[payload_index++] = payload_size - HID_BURST_ID_LEN;
			payload[payload_index++] = fragment_count;
			payload_size -= (HID_BURST_ID_LEN + HID_FRAG_HEADER_LEN);
			if ( fragment_count == 0 )
				payload_size -= HID_MSG_CHKSUM_LEN;

			if (first_fragment) {
				first_fragment = false;
				payload[payload_index++] = hb0;
				payload[payload_index++] = hb1;
				payload_size -= 2;

				/* Calculate the checksum on the entire message.	*/
				accum = ((((uint16_t)hb1) << 8) | hb0);
				pchksum = (uint16_t *)&msg_body[0];
				for ( i = 0; i < (msg_body_length / 2); i++ )
					accum += pchksum[i];
				if ( msg_body_length & 0x01 )
					accum += ((uint16_t)msg_body[msg_body_length - 1]);
				accum = ~(accum - 1);
			}
			memcpy( &payload[payload_index], &msg_body[msg_index], payload_size );
			msg_index += payload_size;
			payload_index += payload_size;

			if ( fragment_count == 0 ) {
				payload[payload_index++] = (uint8_t)accum;
				payload[payload_index++] = (uint8_t)(accum >> 8);
			}
			dump_array( "Message fragment", &payload[0], payload_index );
		}
		fragment_count--;
	}
	return( 0 );
}

/*
 * Accumulate fragments of a HID message until the entire message has
 * been received, then dispatch it properly.
 */
static void build_received_hid_message ( struct mhl_dev_context *context, uint8_t *pmsg, int length )
{
	int 			i, fragment_count = 0;
	uint16_t		*pchksum;
	uint16_t		accum;
	struct mhl3_emsc_data *emsc = &context->mhl_emsc;

	switch ( emsc->hid_receive_state ) {

	case 0:			// A new message is being received.

		if ( length < 4 )	//TODO: Lee - Need to handle this correctly
			return;

		fragment_count = pmsg[0];
		emsc->last_fragment_count = fragment_count;
		emsc->hb0 = pmsg[1];
		emsc->hb1 = pmsg[2];
		length -= 3;
		memcpy( &emsc->in_buffer[0], &pmsg[3], length);
		emsc->msg_length = length;
		emsc->hid_receive_state = 1;
		break;

	case 1:
		if ( length < 2 )	//TODO: Lee - Need to handle this correctly
			return;

		fragment_count = pmsg[0];
		length--;
		if (( emsc->last_fragment_count - 1 ) != fragment_count) {
			return;		//TODO: Lee - Need to handle this correctly
		}
		emsc->last_fragment_count = fragment_count;
		memcpy( &emsc->in_buffer[ emsc->msg_length], &pmsg[1], length );
		emsc->msg_length += length;
		break;
	default:
		/* This is an error, so don't dispatch anything.	*/
		fragment_count = 1;
		break;
	}

	/* If the end of the message, dispatch it.	*/
	if (fragment_count == 0) {
		dump_array( "Received eMSC HID message", &emsc->in_buffer[0], emsc->msg_length );
		accum = 0;
		pchksum = (uint16_t *)&emsc->in_buffer[0];
		emsc->msg_length -= 2;	/* Ignore the checksum	*/
		for ( i = 0; i < (emsc->msg_length / 2); i++ ){
			accum += pchksum[i];
		}
		accum += ((uint16_t)emsc->hb0 | (((uint16_t)emsc->hb1) << 8));
		if ( emsc->msg_length & 0x01 ){
			accum += ((uint16_t)emsc->in_buffer[emsc->msg_length - 1]);
		}
		/* Add the checksum to get 0.	*/
		accum += *((uint16_t*)&emsc->in_buffer[emsc->msg_length]);
		if ( accum != 0 ) {
			//TODO: Lee - Need to handle this
			MHL_TX_DBG_ERR("Zero-sum checksum fail: %04X\n", accum);
		}
		emsc->hid_receive_state = 0;	/* Expect a new command	*/

		/* Send to the completion handler.	*/
		mhl3_hid_message_processor( context,
				emsc->hb0, emsc->hb1, emsc->in_buffer, emsc->msg_length );
	}
}


/*
 * Called from the Titan interrupt handler to parse data
 * received from the eSMC BLOCK hardware. If multiple messages are
 * sent in the same BLOCK Command buffer, they are sorted out here also.
 */
void si_mhl_tx_emsc_received( struct mhl_dev_context *context )
{
	struct drv_hw_context *hw_context = (struct drv_hw_context *)(&context->drv_context);
	int ret, length, index;
	uint16_t burst_id;
	uint8_t *prbuf, *pmsg;

	ret = si_mhl_tx_drv_peek_block_input_buffer(context, &prbuf, &length );
	dump_array( "Received eMSC BLOCK", prbuf, length );
	if (ret == 0) {
		index = 0;
		do {
			pmsg = &prbuf[index];
			burst_id = (((uint16_t)pmsg[0]) << 8) | pmsg[1];
			index += 2;	/* Move past the BURST ID	*/

			switch (burst_id) {
			case burst_id_HID_PAYLOAD:
				build_received_hid_message( context, &pmsg[3], pmsg[2] );
				index += (1 + pmsg[2]); /* Include the payload length	*/
				break;

			case burst_id_BLK_RCV_BUFFER_INFO:
				hw_context->block_protocol.peer_blk_rx_buffer_avail = pmsg[3];
				hw_context->block_protocol.peer_blk_rx_buffer_avail <<= 8;
				hw_context->block_protocol.peer_blk_rx_buffer_avail |= pmsg[2];
				index += 2;
				break;

			case burst_id_BITS_PER_PIXEL_FMT:	//TODO: Lee - What do we do with this data?
				index += (4 + (2 * pmsg[5]));	//TODO: Lee - Use #define values
				break;
			default:
				MHL_TX_DBG_ERR("Unsupported BURST ID: %04X\n", burst_id);
				index = length;
				break;
			}

			length -= index;
		} while(length > 0);
		si_mhl_tx_drv_free_block_input_buffer(context);
	}
}


