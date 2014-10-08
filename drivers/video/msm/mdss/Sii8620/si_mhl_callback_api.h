typedef union {
	detailed_timing_descriptor_t	cea_861_dtd;
	avi_info_frame_t				avif;
}avif_or_cea_861_dtd_u;

typedef union {
	vendor_specific_info_frame_t	hdmi_vsif;
	mhl3_vsif_t						mhl3_vsif;
}vsif_mhl3_or_hdmi_u;

typedef enum {
	/* successful return values for hpd_driven_high(): */
	 hpd_high_format_DVI 									= 0x00000000 /* a DTD has been written to the p_avif_or_dtd buffer instead of an AVI info frame */
	,hpd_high_format_HDMI_vsif_none		 					= 0x00000001 /* No Vendor Specific Info Frame provided */
	,hpd_high_format_HDMI_vsif_hdmi							= 0x00000002 /*	HDMI vsif has been filled into p_vsif */
	,hpd_high_format_HDMI_vsif_mhl3							= 0x00000003 /*	MHL3 vsif has been filled into p_vsif */
	,hpd_high_format_DVI_hdcp_on 							= 0x00000004 /* a DTD has been written to the DTD buffer instead of an AVI info frame and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_HDMI_vsif_none_hdcp_on					= 0x00000005 /* No Vendor Specific Info Frame provided and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_HDMI_vsif_hdmi_hdcp_on					= 0x00000006 /*	HDMI vsif has been filled into p_vsif and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_HDMI_vsif_mhl3_hdcp_on					= 0x00000007 /*	MHL3 vsif has been filled into p_vsif and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_DVI_not_repeatable						= 0x00000008 /* a DTD has been written to the DTD buffer instead of an AVI info frame */
	,hpd_high_format_HDMI_vsif_none_not_repeatable		 	= 0x00000009 /* No Vendor Specific Info Frame provided */
	,hpd_high_format_HDMI_vsif_hdmi_not_repeatable			= 0x0000000A/*	HDMI vsif has been filled into p_vsif */
	,hpd_high_format_HDMI_vsif_mhl3_not_repeatable			= 0x0000000B /*	MHL3 vsif has been filled into p_vsif */
	,hpd_high_format_DVI_hdcp_on_not_repeatable 			= 0x0000000C /* a DTD has been written to the DTD buffer instead of an AVI info frame and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_HDMI_vsif_none_hdcp_on_not_repeatable	= 0x0000000D /* No Vendor Specific Info Frame provided and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_HDMI_vsif_hdmi_hdcp_on_not_repeatable	= 0x0000000E /*	HDMI vsif has been filled into p_vsif and  8620 shall expect HDCP enabled on its HDMI input */
	,hpd_high_format_HDMI_vsif_mhl3_hdcp_on_not_repeatable	= 0x0000000F /*	MHL3 vsif has been filled into p_vsif and  8620 shall expect HDCP enabled on its HDMI input */

	/* failure return values for hpd_driven_high(): */
	,hpd_high_insufficient_avi_or_dtd_buffer 		= 0x80000001 /* avi_max_length not large enough for AVI info frame data. */ 
	,hpd_high_insufficient_vsif_buffer				= 0x80000002 /*	vsif_max_length not large enough for info frame data. */	
	,hpd_high_video_not_ready						= 0x80000004 /* The callee is not ready to start video */ 
}hpd_high_callback_status;

typedef struct {
	void	*context;
#if 1 //(
	int	(*display_timing_enum_begin)(void *context);
	int	(*display_timing_enum_item)(void *context
					,uint16_t columns
					,uint16_t rows
					,uint8_t  bits_per_pixel
					,uint32_t  vertical_refresh_rate_in_milliHz
					,uint16_t  burst_id
					,video_burst_descriptor_u *p_descriptor);
	int (*display_timing_enum_end)(void *context);
#endif //)
	/*
		hpd_driven_low:
			This gets called in response to CLR_HPD messages from the MHL sink.
			The upstream client that registers this callback should disable 
			video and all DDC access before returning.
	*/
	void (*hpd_driven_low) (void *context);

	/* 
	hpd_driven_high:
		This gets called when the driver is ready for upstream video activity.
		The upstream client that registers this callback should respond in the
			same way in which it would respond to a rising HPD 
			signal (which happens prior to the call).
	Parameters:
		*p_edid:

			The processed EDID block(s) that the MHL driver has derived
			from the downstream EDID and WRITE_BURST info associated with the
			sink's responses to 3D_REQ (MHL 2.x) or FEAT_REQ (MHL 3.x and newer).

		edid_length:

			The length, in bytes, of the data in *p_edid


		*p_hev_dtd:
			The processed result of all the HEV_DTDA/HEV_DTDB WRITE_BURSTs including
				the associated 3D_DTD VDI for each HEV_DTD pair.

		num_hev_dtds:
			The number of MHL3_hev_dtd_t elements in *p_hev_dtd.

		p_hev_vic:
			The processed result of all the HEV_VIC WRITE_BURSTs including
				the associated 3D_VIC VDI for each HEV_DTD pair.

		num_hev_vic_items:
			The number of MHL3_hev_vic_item_t elements in p_hev_vic.

		*p_3d_dtd_items:
			The processed result of all the 3D_DTD WRITE_BURSTs including
				the associated DTD from the EDID when VDI_H.HEV_FMT is zero.

		num_3d_dtd_items:
			The number of MHL3_3d_dtd_item_t elements in p_3d_dtd_items;

		*p_3d_vic:
			The processed result of all the 3D_VIC WRITE_BURSTs including
				the associated VIC code from the EDID.

		num_3d_vic_items:
			The number of MHL3_3d_vic_item_t elements in p_3d_vic.

		p_avif_or_dtd:

			If the callee sends HDMI content, it shall fill in *p_avif_or_dtd with
			the contents of its outgoing AVI info frame, including the checksum byte,
			and return one of the values described under the parameter p_vsif.

			If the callee, sends DVI content, is shall fill in *p_avif_or_dtd with
			a Detailed Timing Descriptor (defined in CEA-861D) that accurately
			describes the timing parameters of the video which is presented at the
			8620's HDMI input and return one of:
				hpd_high_format_DVI
				hpd_high_format_DVI_hdcp_on
				hpd_high_format_DVI_not_repeatable
				hpd_high_format_DVI_hdcp_on_not_repeatable.

			This buffer will be pre-initialized to zeroes prior to the call. 

		avi_max_length:

			The length of the buffer pointed to by p_avif_or_dtd.

		p_vsif:

			A buffer into which the upstream driver should
			write the contents of its outgoing vendor specific
			info frame, if any, including the checksum byte.  This 
			buffer will be pre-initialized to zeroes prior to the call.

			If the callee chooses to write an HDMI vendor specific info frame
			into p_vsif, it shall return one of:
				hpd_high_format_HDMI_vsif_hdmi
				hpd_high_format_HDMI_vsif_hdmi_hdcp_on
				hpd_high_format_HDMI_vsif_hdmi_not_repeatable
				hpd_high_format_HDMI_vsif_hdmi_hdcp_on_not_repeatable.

			If the callee chooses to write an MHL3 vendor specific info frame
			into p_vsif, it shall return one of:
				hpd_high_format_HDMI_vsif_mhl3
				hpd_high_format_HDMI_vsif_mhl3_hdcp_on
				hpd_high_format_HDMI_vsif_mhl3_not_repeatable
				hpd_high_format_HDMI_vsif_mhl3_hdcp_on_not_repeatable.

			If the callee does not write a vendor specific
			info frame into this buffer, the callee shall return one of:
				hpd_high_format_HDMI_vsif_none
				hpd_high_format_HDMI_vsif_none_hdcp_on
				hpd_high_format_HDMI_vsif_none_not_repeatable
				hpd_high_format_HDMI_vsif_none_hdcp_on_not_repeatablei
			and the 8620 will infer the contents of the outgoing MHL3 VSIF from
			the contents of the HDMI VSIF (if any) presented at the 8620's HDMI input.

		vsif_max_length:

			The length of the buffer pointed to by p_vsif.


	Return values for hpd_driven_high():		

		If the callee enabled video during the duration of this call, then
			the callee shall return one of the values in
			hpd_high_callback_status that do not have the sign bit set,
			indicating the usage of parameters.
					   
		If the callee did not enable video during the duration of this call,
			then the callee shall indicate specific reasons for not starting
			video by returning the bitwise OR of the values in
			hpd_high_callback_status that do have the sign bit set.

	*/
	hpd_high_callback_status (*hpd_driven_high)(void *context
					,uint8_t *p_edid,size_t edid_length
					,MHL3_hev_dtd_item_t *p_hev_dtd		, size_t num_hev_dtds
					,MHL3_hev_vic_item_t *p_hev_vic		, size_t num_hev_vic_items
					,MHL3_3d_dtd_item_t *p_3d_dtd_items	, size_t num_3d_dtd_items
					,MHL3_3d_vic_item_t *p_3d_vic		, size_t num_3d_vic_items
					,avif_or_cea_861_dtd_u *p_avif_or_dtd, size_t avif_or_dtd_max_length
					,vsif_mhl3_or_hdmi_u *p_vsif, size_t vsif_max_length
					);
}si_mhl_callback_api_t;
/* call this function to register the callback structure */
int si_8620_register_callbacks(si_mhl_callback_api_t *p_callbacks);

/* call this function to change video modes */
int si_8620_info_frame_change(
			 hpd_high_callback_status status
			,avif_or_cea_861_dtd_u *p_avif_or_dtd, size_t avif_or_dtd_max_length
			,vsif_mhl3_or_hdmi_u *p_vsif, size_t vsif_max_length);
