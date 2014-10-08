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

#include <linux/module.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/stringify.h>
#include <asm/uaccess.h>

#include "si_fw_macros.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "mhl_rcp_inputdev.h"
#include "mhl_linux_tx.h"
#include "mhl_supp.h"
#include "platform.h"

#include <linux/kthread.h>
//added by zhang yue on 2014-05-23 for qualcomm platform start
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include "../mdss_hdmi_mhl.h"
#include <linux/of_platform.h>
//added by zhang yue on 2014-05-23 for qualcomm platform end

void si_dump_important_regs(struct drv_hw_context *hw_context); /* from the driver */

#define MHL_DRIVER_MINOR_MAX 1

/* Convert a value specified in milliseconds to nanoseconds */
#define MSEC_TO_NSEC(x)		(x * 1000000UL)

static char *white_space = "' ', '\t'";
static dev_t dev_num;

static struct class *mhl_class;

static void mhl_tx_destroy_timer_support(struct  mhl_dev_context *dev_context);

/* Define SysFs attribute names */
#define SYS_ATTR_NAME_CONN					connection_state
#define SYS_ATTR_NAME_I2C_REGS				i2c_regs
#define SYS_ATTR_NAME_SPAD					spad
#define SYS_ATTR_NAME_I2C_REGISTERS			i2c_registers
#define SYS_ATTR_NAME_DEBUG_LEVEL			debug_level
#define SYS_ATTR_NAME_REG_DEBUG_LEVEL		reg_debug_level
#define SYS_ATTR_NAME_AKSV					aksv
#define SYS_ATTR_NAME_EDID					edid
#define SYS_ATTR_NAME_HEV_3D_DATA			hev_3d_data
#define SYS_ATTR_NAME_GPIO_INDEX			gpio_index
#define SYS_ATTR_NAME_GPIO_VALUE			gpio_value

#ifdef DEBUG
#define SYS_ATTR_NAME_TX_POWER			tx_power
#define SYS_ATTR_NAME_STARK_CTL			set_stark_ctl
#endif

#ifdef BIST_INITIATOR
#define SYS_ATTR_NAME_BIST				bist
extern void start_bist_initiator_test(struct mhl_dev_context *dev_context);
#endif

#define 	SYS_ATTR_NAME_IN			in
#define 	SYS_ATTR_NAME_IN_STATUS		in_status
#define		SYS_ATTR_NAME_OUT			out
#define		SYS_ATTR_NAME_OUT_STATUS	out_status
#define		SYS_ATTR_NAME_INPUT_DEV		input_dev

/* define SysFs object names */
#define SYS_OBJECT_NAME_RAP				rap
#define 	SYS_ATTR_NAME_RAP_IN			SYS_ATTR_NAME_IN
#define 	SYS_ATTR_NAME_RAP_IN_STATUS		SYS_ATTR_NAME_IN_STATUS
#define		SYS_ATTR_NAME_RAP_OUT			SYS_ATTR_NAME_OUT
#define		SYS_ATTR_NAME_RAP_OUT_STATUS	SYS_ATTR_NAME_OUT_STATUS
#define		SYS_ATTR_NAME_RAP_INPUT_DEV		SYS_ATTR_NAME_INPUT_DEV

#define SYS_OBJECT_NAME_RCP				rcp
#define 	SYS_ATTR_NAME_RCP_IN			SYS_ATTR_NAME_IN
#define 	SYS_ATTR_NAME_RCP_IN_STATUS		SYS_ATTR_NAME_IN_STATUS
#define		SYS_ATTR_NAME_RCP_OUT			SYS_ATTR_NAME_OUT
#define		SYS_ATTR_NAME_RCP_OUT_STATUS	SYS_ATTR_NAME_OUT_STATUS
#define		SYS_ATTR_NAME_RCP_INPUT_DEV		SYS_ATTR_NAME_INPUT_DEV

#define SYS_OBJECT_NAME_UCP				ucp
#define 	SYS_ATTR_NAME_UCP_IN			SYS_ATTR_NAME_IN
#define 	SYS_ATTR_NAME_UCP_IN_STATUS		SYS_ATTR_NAME_IN_STATUS
#define		SYS_ATTR_NAME_UCP_OUT			SYS_ATTR_NAME_OUT
#define		SYS_ATTR_NAME_UCP_OUT_STATUS	SYS_ATTR_NAME_OUT_STATUS
#define		SYS_ATTR_NAME_UCP_INPUT_DEV		SYS_ATTR_NAME_INPUT_DEV

#define SYS_OBJECT_NAME_DEVCAP		devcap
#define 	SYS_ATTR_NAME_DEVCAP_LOCAL_OFFSET	local_offset
#define		SYS_ATTR_NAME_DEVCAP_LOCAL			local
#define		SYS_ATTR_NAME_DEVCAP_REMOTE_OFFSET	remote_offset
#define		SYS_ATTR_NAME_DEVCAP_REMOTE			remote


#define SYS_OBJECT_NAME_SWWA		swwa
#define		SYS_ATTR_NAME_SWWA_VIC_OVERRIDE		vic_override

#define SYS_OBJECT_NAME_HDCP		hdcp
#define 	SYS_ATTR_NAME_HDCP_CONTENT_TYPE		hdcp_content_type

/*
 * show_connection_state() - Handle read request to the connection_state
 * 							 attribute file.
 */
ssize_t show_connection_state(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);

	if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {
		return scnprintf(buf, PAGE_SIZE, "connected");
	} else {
		return scnprintf(buf, PAGE_SIZE, "not connected");
	}
}



/*
 * send_scratch_pad() - Handle write request to the spad attribute file.
 *
 * This file is used to either initiate a write to the scratch pad registers
 * of an attached device, or to set the offset and byte count for a subsequent
 * read from the local scratch pad registers.
 *
 * The format of the string in buf must be:
 * 	offset=<offset_value> length=<Length_value> \
 * 	data=data_byte_0 ... data_byte_length-1
 * 	where:	<offset_value>	specifies the starting register offset to begin
 * 							read/writing within the scratch pad register space
 * 			<length_value>	number of scratch pad registers to be written/read
 * 			data_byte		space separated list of <length_value> data bytes
 * 							to be written.  If no data bytes are present then
 * 							the write to this file will only be used to set
 * 							the offset and length for a subsequent read from
 * 							this file.
 */
ssize_t send_scratch_pad(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long	offset = 0x100;		/* initialize with invalid values */
	unsigned long	length = 0x100;
	unsigned long	value;
	u8				data[MAX_SCRATCH_PAD_TRANSFER_SIZE];
	u8				idx;
	char			*str;
	char			*endptr;
	enum scratch_pad_status	scratch_pad_status;
	int				status = -EINVAL;

	MHL_TX_DBG_ERR("received string: ""%s""\n", buf);

	/*
	 * Parse the input string and extract the scratch pad register selection
	 * parameters
	 */
	str = strstr(buf, "offset=");
	if (str != NULL) {
		offset = simple_strtoul(str + 7, NULL, 0);
		if (offset > SCRATCH_PAD_SIZE) {
			MHL_TX_DBG_ERR("Invalid offset value entered\n");
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't "\
				"find ""offset"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "length=");
	if (str != NULL) {
		length = simple_strtoul(str + 7, NULL, 0);
		if (length > MAX_SCRATCH_PAD_TRANSFER_SIZE) {
			MHL_TX_DBG_ERR("Transfer length too large\n");
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't "
				"find ""length"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "data=");
	if (str != NULL) {

		str += 5;
		endptr = str;
		for(idx = 0; idx < length; idx++) {

			endptr += strspn(endptr, white_space);
			str = endptr;
			if (*str == 0) {
				MHL_TX_DBG_ERR("Too few data values provided\n");
				goto err_exit_2;
			}

			value = simple_strtoul(str, &endptr, 0);
			if (value > 0xFF) {
				MHL_TX_DBG_ERR("Invalid scratch pad data detected\n");
				goto err_exit_2;
			}

			data[idx] = value;
		}

	} else {
		idx = 0;
	}

	if ((offset + length) > SCRATCH_PAD_SIZE) {
		MHL_TX_DBG_ERR("Invalid offset/length combination" \
				"entered");
		goto err_exit_2;
	}

	dev_context->spad_offset = offset;
	dev_context->spad_xfer_length = length;

	if (idx == 0) {
		MHL_TX_DBG_INFO("No data specified, storing offset "\
				"and length for subsequent scratch pad read\n");

		goto err_exit_2;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit_1;
	}

	/*
	 * Make sure there is an MHL connection and that the requested
	 * data transfer parameters don't exceed the address space of
	 * the scratch pad.  NOTE: The address space reserved for the
	 * Scratch Pad registers is 64 bytes but sources and sink devices
	 * are only required to implement the 1st 16 bytes.
	 */
	if (!(dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) ||
		(length < ADOPTER_ID_SIZE) ||
		(offset > (SCRATCH_PAD_SIZE - ADOPTER_ID_SIZE)) ||
		(offset + length > SCRATCH_PAD_SIZE)) {
		status = -EINVAL;
		goto err_exit_1;
	}

	dev_context->mhl_flags |= MHL_STATE_FLAG_SPAD_SENT;
	dev_context->spad_send_status = 0;

	scratch_pad_status = si_mhl_tx_request_write_burst(dev_context,
											offset, length, data);

	switch (scratch_pad_status) {
		case SCRATCHPAD_SUCCESS:
			/* On success return the number of bytes written to this file */
			status = count;
			break;

		case SCRATCHPAD_BUSY:
			status = -EAGAIN;
			break;

		default:
			status = -EFAULT;
			break;
	}

err_exit_1:
	up(&dev_context->isr_lock);

err_exit_2:
	return status;
}


/*
 * show_scratch_pad() - Handle read request to the spad attribute file.
 *
 * Reads from this file return one or more scratch pad register values
 * in hexadecimal string format.  The registers returned are specified
 * by the offset and length values previously written to this file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 *
 * The format of the string returned in buf is:
 * 	"offset:<offset> length:<lenvalue> data:<datavalues>
 * 	where:	<offset>	is the last scratch pad register offset
 * 						written to this file
 * 			<lenvalue>	is the last scratch pad register transfer length
 * 						written to this file
 * 			<datavalue>	space separated list of <lenvalue> scratch pad
 * 						register values in OxXX format
 */
ssize_t show_scratch_pad(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	u8						data[MAX_SCRATCH_PAD_TRANSFER_SIZE];
	u8						idx;
	enum scratch_pad_status	scratch_pad_status;
	int						status = -EINVAL;


	MHL_TX_DBG_INFO("called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {

		scratch_pad_status  = si_get_scratch_pad_vector(dev_context,
												dev_context->spad_offset,
												dev_context->spad_xfer_length,
												data);

		switch (scratch_pad_status) {
			case SCRATCHPAD_SUCCESS:
				status = scnprintf(buf, PAGE_SIZE, "offset:0x%02x " \
								   "length:0x%02x data:",
								   dev_context->spad_offset,
								   dev_context->spad_xfer_length);

				for (idx = 0; idx < dev_context->spad_xfer_length; idx++) {
					status += scnprintf(&buf[status], PAGE_SIZE, "0x%02x ",
										data[idx]);
				}
				break;

			case SCRATCHPAD_BUSY:
				status = -EAGAIN;
				break;

			default:
				status = -EFAULT;
				break;
		}
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * send_debug() - Handle write request to the debug attribute file.
 *
 * This file is used to either perform a write to registers of the transmitter
 * or to set the address, offset and byte count for a subsequent from the
 * register(s) of the transmitter.
 *
 * The format of the string in buf must be:
 * 	address=<pageaddr> offset=<offset_value> length=<Length_value> \
 * 	data=data_byte_0 ... data_byte_length-1
 * 	where: <pageaddr>		specifies the I2C register page of the register(s)
 * 							to be written/read
 * 			<offset_value>	specifies the starting register offset within the
 * 							register page to begin writing/reading
 * 			<length_value>	number registers to be written/read
 * 			data_byte		space separated list of <length_value> data bytes
 * 							to be written.  If no data bytes are present then
 * 							the write to this file will only be used to set
 * 							the  page address, offset and length for a
 * 							subsequent read from this file.
 */
ssize_t set_i2c(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long	address = 0x100;		/* initialize with invalid values */
	unsigned long	offset = 0x100;
	unsigned long	length = 0x100;
	unsigned long	value;
	u8				data[MAX_DEBUG_TRANSFER_SIZE];
	u8				idx;
	char			*str;
	char			*endptr;
	int				status = -EINVAL;

	MHL_TX_COMM_INFO("received string: ""%s""\n", buf);

	/*
	 * Parse the input string and extract the scratch pad register selection
	 * parameters
	 */
	str = strstr(buf, "address=");
	if (str != NULL) {
		address = simple_strtoul(str + 8, NULL, 0);
		if (address > 0xFF) {
			MHL_TX_DBG_ERR("Invalid page address: 0x%02lx" \
					"specified\n", address);
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't "\
				"find ""address"" parameter\n");
		goto err_exit_2;
	}

	str = strstr(buf, "offset=");
	if (str != NULL) {
		offset = simple_strtoul(str + 7, NULL, 0);
		if (offset > 0xFF) {
			MHL_TX_DBG_ERR("Invalid page offset: 0x%02lx" \
					"specified\n", offset);
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't "\
				"find ""offset"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "length=");
	if (str != NULL) {
		length = simple_strtoul(str + 7, NULL, 0);
		if (length > MAX_DEBUG_TRANSFER_SIZE) {
			MHL_TX_DBG_ERR("Transfer size 0x%02lx is too "\
					"large\n", length);
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't "\
				"find ""length"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "data=");
	if (str != NULL) {

		str += 5;
		endptr = str;
		for(idx = 0; idx < length; idx++) {
			endptr += strspn(endptr, white_space);
			str = endptr;
			if (*str == 0) {
				MHL_TX_DBG_ERR("Too few data values provided\n");
				goto err_exit_2;
			}

			value = simple_strtoul(str, &endptr, 0);

			if (value > 0xFF) {
				MHL_TX_DBG_ERR("Invalid register data "\
						"value detected\n");
				goto err_exit_2;
			}

			data[idx] = value;
		}


	} else {
		idx = 0;
	}

	dev_context->debug_i2c_address = address;
	dev_context->debug_i2c_offset = offset;
	dev_context->debug_i2c_xfer_length = length;

	if (idx == 0) {
		MHL_TX_COMM_INFO("No data specified, storing address "\
				 "offset and length for subsequent debug read\n");
		goto err_exit_2;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit_1;
	}

	status =  dev_context->drv_info->mhl_device_dbg_i2c_reg_xfer(
												&dev_context->drv_context,
												address, offset, length,
												DEBUG_I2C_WRITE, data);
	if (status == 0)
		status = count;

err_exit_1:
	up(&dev_context->isr_lock);

err_exit_2:
	return status;
}

/*
 * show_debug()	- Handle read request to the debug attribute file.
 *
 * Reads from this file return one or more transmitter register values in
 * hexadecimal string format.  The registers returned are specified by the
 * address, offset and length values previously written to this file.
 *
 * The return value is the number characters written to buf, or an error
 * code if the I2C read fails.
 *
 * The format of the string returned in buf is:
 * 	"address:<pageaddr> offset:<offset> length:<lenvalue> data:<datavalues>
 * 	where:	<pageaddr>	is the last I2C register page address written
 * 						to this file
 * 			<offset>	is the last register offset written to this file
 * 			<lenvalue>	is the last register transfer length written
 * 						to this file
 * 			<datavalue>	space separated list of <lenvalue> register
 * 						values in OxXX format
 */
ssize_t show_i2c(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	u8						data[MAX_DEBUG_TRANSFER_SIZE];
	u8						idx;
	int						status = -EINVAL;

	MHL_TX_COMM_INFO("called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto no_dev;
	}

	status =  dev_context->drv_info->mhl_device_dbg_i2c_reg_xfer(
											&dev_context->drv_context,
											dev_context->debug_i2c_address,
											dev_context->debug_i2c_offset,
											dev_context->debug_i2c_xfer_length,
											DEBUG_I2C_READ, data);
no_dev:
	up(&dev_context->isr_lock);

	if (status == 0) {
		extern bool i2c_data_only;
		if (i2c_data_only){
			status = 0;
		}else{
			status = scnprintf(buf, PAGE_SIZE, "address:0x%02x offset:0x%02x " \
						   "length:0x%02x data:",
							dev_context->debug_i2c_address,
							dev_context->debug_i2c_offset,
							dev_context->debug_i2c_xfer_length);
		}

		for (idx = 0; idx < dev_context->debug_i2c_xfer_length; idx++) {
			status += scnprintf(&buf[status], PAGE_SIZE, "0x%02x ",
								data[idx]);
		}
	}

	return status;
}

//( begin debug_level API
/*
 * show_debug_level() - Handle read request to the debug_level attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_debug_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int debug_level;
	extern bool debug_reg_dump;


	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}

	status = scnprintf(buf, PAGE_SIZE, "level=%d %s",
			debug_level,
			debug_reg_dump ? "(includes reg dump)" : "" );
	MHL_TX_DBG_INFO("buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	si_dump_important_regs((struct drv_hw_context *)&dev_context->drv_context);

	return status;
}

/*
 * set_debug_level() - Handle write request to the debug_level attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t set_debug_level(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	extern int debug_level;
	long	new_debug_level = 0x100;		/* initialize with invalid values */

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (buf != NULL) {
		/* BUGZILLA 27012 no prefix here, only on module parameter. */
		new_debug_level = simple_strtol(buf, NULL, 0);
		if (new_debug_level > 0xFF) {
			MHL_TX_DBG_ERR("Invalid debug_level specified: 0x%02lX specified\n", new_debug_level);
			goto err_exit;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't find ""debug_level"" parameter\n");
		goto err_exit;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	debug_level = new_debug_level;
	MHL_TX_DBG_ERR("new debug_level=0x%02X\n", debug_level);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

//) end debug_level API
//( begin reg_debug_level API
/*
 * show_reg_debug_level() - Handle read request to the reg_debug_level attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_reg_debug_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int reg_debug_level;
	extern bool debug_reg_dump;


	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}

	status = scnprintf(buf, PAGE_SIZE, "level=%d %s",
			reg_debug_level,
			debug_reg_dump ? "(includes reg dump)" : "" );
	MHL_TX_DBG_INFO("buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * set_reg_debug_level() - Handle write request to the reg_debug_level attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t set_reg_debug_level(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	extern int reg_debug_level;
	extern bool debug_reg_dump;
	unsigned long	new_reg_debug_level = 0x100;		/* initialize with invalid values */
	char			*str;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	str = strstr(buf, "reg_debug_level=");
	if (str != NULL) {
		new_reg_debug_level = simple_strtoul(str + 12, NULL, 0);
		if (new_reg_debug_level > 0xFF) {
			MHL_TX_DBG_ERR("Invalid reg_debug_level specified: 0x%02lX specified\n", new_reg_debug_level);
			goto err_exit;
		}
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't find ""reg_debug_level"" parameter\n");
		goto err_exit;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	reg_debug_level = new_reg_debug_level;
	MHL_TX_DBG_ERR("new reg_debug_level=0x%02X\n", reg_debug_level);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

//) end reg_debug_level API
//( begin gpio_index API
/*
 * show_gpio_index() - Handle read request to the gpio_index attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_gpio_index(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int gpio_index;


	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}

	status = scnprintf(buf, PAGE_SIZE, "%d",gpio_index );
	MHL_TX_DBG_INFO("buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * set_gpio_index() - Handle write request to the gpio_index attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t set_gpio_index(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	extern int gpio_index;
	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	gpio_index = simple_strtol(buf, NULL, 0);
	MHL_TX_DBG_INFO("gpio: %d\n",gpio_index);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

//) end gpio_index API
//( begin gpio_level API
/*
 * show_gpio_level() - Handle read request to the gpio_level attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_gpio_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int gpio_index;
	int gpio_level;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	gpio_level = gpio_get_value(gpio_index);
	status = scnprintf(buf, PAGE_SIZE, "%d",gpio_level );
	MHL_TX_DBG_INFO("buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * set_gpio_level() - Handle write request to the gpio_level attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t set_gpio_level(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	int gpio_level;
	extern int gpio_index;
	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	gpio_level = simple_strtol(buf, NULL, 0);
	MHL_TX_DBG_INFO("gpio: %d<-%d\n",gpio_index,gpio_level);
	gpio_set_value(gpio_index, gpio_level);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

//) end gpio_level API

/*
 * show_aksv()	- Handle read request to the aksv attribute file.
 *
 * Reads from this file return the 5 bytes of the transmitter's
 * Key Selection Vector (KSV). The registers are returned as a string
 * of space separated list of hexadecimal values formated as 0xXX.
 *
 * The return value is the number characters written to buf.
 */
ssize_t show_aksv(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	u8						data[5];
	u8						idx;
	int						status = -EINVAL;

	MHL_TX_DBG_INFO("called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto no_dev;
	}

	status =  dev_context->drv_info->mhl_device_get_aksv(
						(struct drv_hw_context *)&dev_context->drv_context,
						data);
no_dev:
	up(&dev_context->isr_lock);

	if (status == 0) {

		for (idx = 0; idx < 5; idx++) {
			status += scnprintf(&buf[status], PAGE_SIZE, "0x%02x ",
								data[idx]);
		}
	}

	return status;
}

/*
 * show_edid()	- Handle read request to the aksv attribute file.
 *
 * Reads from this file return the edid, as processed by this driver and
 *	presented upon the upstream DDC interface.
 *
 * The return value is the number characters written to buf.
 */
ssize_t show_edid(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	u8						edid_buffer[256];
	int						status = -EINVAL;

	MHL_TX_DBG_INFO("called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else{
		int si_mhl_tx_drv_sample_edid_buffer(struct drv_hw_context *hw_context,uint8_t *edid_buffer);
		status =  si_mhl_tx_drv_sample_edid_buffer(
						(struct drv_hw_context *)&dev_context->drv_context
						, edid_buffer);
	}

	up(&dev_context->isr_lock);

	if (status == 0) {
		int idx,i;
		for (idx = 0,i=0; i < 16; i++) {
			u8 j;
			for (j = 0; j <16;++j,++idx){
				status += scnprintf(&buf[status]
								, PAGE_SIZE, "0x%02x ",
								edid_buffer[idx]);
			}
			status += scnprintf(&buf[status], PAGE_SIZE, "\n");
		}
	}

	return status;
}

/*
 * show_hev_3d()	- Handle read request to the aksv attribute file.
 *
 * Reads from this file return the HEV_VIC,HEV_DTD,3D_VIC,and 3D_DTD WRITE_BURST information
 *		presented in a format showing the respective associations.
 *
 * The return value is the number characters written to buf.
 */
ssize_t show_hev_3d(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = -EINVAL;

	MHL_TX_DBG_INFO("called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else{
		int si_mhl_tx_drv_sample_hev_3d_buffer(struct drv_hw_context *hw_context,uint8_t *edid_buffer);
		edid_3d_data_p p_edid_data = dev_context->edid_parser_context;
		int i;
		status = 0;
		status += scnprintf(&buf[status],PAGE_SIZE,"HEV_DTD list:\n");
		for (i=0; i < p_edid_data->hev_dtd_info.num_items;++i){
			
			status += scnprintf(&buf[status]
				, PAGE_SIZE, "%s %s %s 0x%02x 0x%04x 0x%04x 0x%04x 0x%04x 0x%04x 0x%02x -- 0x%04x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n"
				, p_edid_data->hev_dtd_list[i]._3d_info.vdi_l.top_bottom?"TB":"--"
				, p_edid_data->hev_dtd_list[i]._3d_info.vdi_l.left_right?"LR":"--"
				, p_edid_data->hev_dtd_list[i]._3d_info.vdi_l.frame_sequential?"FS":"--"

				, p_edid_data->hev_dtd_list[i].sequence_index
				, ENDIAN_CONVERT_16(p_edid_data->hev_dtd_list[i].a.pixel_clock_in_MHz)
				, ENDIAN_CONVERT_16(p_edid_data->hev_dtd_list[i].a.h_active_in_pixels)
				, ENDIAN_CONVERT_16(p_edid_data->hev_dtd_list[i].a.h_blank_in_pixels)
				, ENDIAN_CONVERT_16(p_edid_data->hev_dtd_list[i].a.h_front_porch_in_pixels)
				, ENDIAN_CONVERT_16(p_edid_data->hev_dtd_list[i].a.h_sync_width_in_pixels)
				, p_edid_data->hev_dtd_list[i].a.h_flags

				, ENDIAN_CONVERT_16(p_edid_data->hev_dtd_list[i].b.v_total_in_lines)
				, p_edid_data->hev_dtd_list[i].b.v_blank_in_lines
				, p_edid_data->hev_dtd_list[i].b.v_front_porch_in_lines
				, p_edid_data->hev_dtd_list[i].b.v_sync_width_in_lines
				, p_edid_data->hev_dtd_list[i].b.v_refresh_rate_in_fields_per_second
				, p_edid_data->hev_dtd_list[i].b.v_flags

				);
		}

		status += scnprintf(&buf[status],PAGE_SIZE,"HEV_VIC list:\n");
		for (i=0; i < p_edid_data->hev_vic_info.num_items;++i){
			status += scnprintf(&buf[status], PAGE_SIZE
				, "%s %s %s 0x%02x 0x%02x\n"
				, p_edid_data->hev_vic_list[i]._3d_info.vdi_l.top_bottom?"TB":"--"
				, p_edid_data->hev_vic_list[i]._3d_info.vdi_l.left_right?"LR":"--"
				, p_edid_data->hev_vic_list[i]._3d_info.vdi_l.frame_sequential?"FS":"--"

				, p_edid_data->hev_vic_list[i].mhl3_hev_vic_descriptor.vic_cea861f
				, p_edid_data->hev_vic_list[i].mhl3_hev_vic_descriptor.reserved
				);
		} 

		status += scnprintf(&buf[status],PAGE_SIZE,"3D_DTD list:\n");
		for (i=0; i < p_edid_data->_3d_dtd_info.num_items;++i){
			status += scnprintf(&buf[status]
				, PAGE_SIZE, "%s %s %s "
							"0x%02x 0x%02x " /* pixel clock */
							"0x%02x 0x%02x {0x%1x 0x%1x} " /*horizontal active and blanking */
							"0x%02x 0x%02x {0x%1x 0x%1x} " /*vertical active and blanking */
							"0x%02x 0x%02x {0x%1x 0x%1x} {0x%1x 0x%1x 0x%1x 0x%1x} " /* sync pulse width and offset */
							"0x%02x 0x%02x {0x%1x 0x%1x} " /* Image sizes */
							"0x%1x 0x%1x {0x%1x 0x%1x 0x%1x 0x%1x %s}\n" /* borders and flags */
				, p_edid_data->_3d_dtd_list[i]._3d_info.vdi_l.top_bottom?"TB":"--"
				, p_edid_data->_3d_dtd_list[i]._3d_info.vdi_l.left_right?"LR":"--"
				, p_edid_data->_3d_dtd_list[i]._3d_info.vdi_l.frame_sequential?"FS":"--"

				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.pixel_clock_low
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.pixel_clock_high

				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_active_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_blanking_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_active_blanking_high.horz_blanking_11_8
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_active_blanking_high.horz_active_11_8

				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_active_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_blanking_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_active_blanking_high.vert_blanking_11_8
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_active_blanking_high.vert_active_11_8

				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_sync_offset_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_sync_pulse_width7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_sync_offset_width.vert_sync_pulse_width_3_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_sync_offset_width.vert_sync_offset_3_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.hs_offset_hs_pulse_width_vs_offset_vs_pulse_width.vert_sync_pulse_width_5_4
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.hs_offset_hs_pulse_width_vs_offset_vs_pulse_width.vert_sync_offset_5_4
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.hs_offset_hs_pulse_width_vs_offset_vs_pulse_width.horz_sync_pulse_width_9_8
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.hs_offset_hs_pulse_width_vs_offset_vs_pulse_width.horzSyncOffset9_8

				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_image_size_in_mm_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_image_size_in_mm_7_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.image_size_high.vert_image_size_in_mm_11_8
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.image_size_high.horz_image_size_in_mm_11_8

				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.horz_border_in_lines
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.vert_border_in_pixels
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.flags.stereo_bit_0
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.flags.sync_signal_options
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.flags.sync_signal_type
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.flags.stereo_bits_2_1
				, p_edid_data->_3d_dtd_list[i].dtd_cea_861.flags.interlaced?"interlaced":"progressive"
				);
		}

		status += scnprintf(&buf[status],PAGE_SIZE,"3d_VIC list:\n");
		for (i=0; i < p_edid_data->_3d_vic_info.num_items;++i){
			status += scnprintf(&buf[status], PAGE_SIZE
				, "%s %s %s %s 0x%02x\n"
				, p_edid_data->_3d_vic_list[i]._3d_info.vdi_l.top_bottom?"TB":"--"
				, p_edid_data->_3d_vic_list[i]._3d_info.vdi_l.left_right?"LR":"--"
				, p_edid_data->_3d_vic_list[i]._3d_info.vdi_l.frame_sequential?"FS":"--"

				, p_edid_data->_3d_vic_list[i].svd.native?"N":" "
				, p_edid_data->_3d_vic_list[i].svd.VIC
				);
		}
	}

	up(&dev_context->isr_lock);
	return status;
}

#ifdef DEBUG
/*
 * set_tx_power() - Handle write request to the tx_power attribute file.
 *
 * Write the string "on" or "off" to this file to power on or off the
 * MHL transmitter.
 *
 * The return value is the number of characters in buf if successful or an
 * error code if not successful.
 */
ssize_t set_tx_power(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = 0;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else if (strnicmp("on", buf, count - 1) == 0) {
		status = si_8620_power_control(true);
	} else if (strnicmp("off", buf, count - 1) == 0) {
		status = si_8620_power_control(false);
	} else {
		MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
		status = -EINVAL;
	}

	if (status != 0)
		return status;
	else
		return count;
}

/*
 * set_stark_ctl() - Handle write request to the tx_power attribute file.
 *
 * Write the string "on" or "off" to this file to power on or off the
 * MHL transmitter.
 *
 * The return value is the number of characters in buf if successful or an
 * error code if not successful.
 */
ssize_t set_stark_ctl(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = 0;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else{
		if (strnicmp("on", buf, count - 1) == 0) {
			/*
			 * Set MHL/USB switch to USB
			 * NOTE: Switch control is implemented differently on each
			 * version of the starter kit.
			 */
			//set_pin(dev,X01_USB_SW_CTRL,1);
			set_pin(dev,X02_USB_SW_CTRL,1);
		} else if (strnicmp("off", buf, count - 1) == 0) {
			/*
			 * Set MHL/USB switch to USB
			 * NOTE: Switch control is implemented differently on each
			 * version of the starter kit.
			 */
			//set_pin(dev,X01_USB_SW_CTRL,0);
			set_pin(dev,X02_USB_SW_CTRL,0);
		} else {
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	if (status != 0)
		return status;
	else
		return count;
}
#endif

#ifdef BIST_INITIATOR
/*
 * set_bist() - Handle write request to the bist attribute file.
 *
 * The format of the string in buf must be:
 *  cmd={bist_cmd} data={xx}
 *   where:
 *      bist_cmd = BIST_TRIGGER or BIST_STOP or BIST_REQUEST_STAT
 *      xx = the value of the second message byte needed by the BIST_TRIGGER and BIST_REQUEST_STAT commands
 *
 * The return value is the number of characters in buf if successful or an
 * error code if not successful.
 */
ssize_t set_bist(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	char	*str;
	int		status = 0;
	int     dat;

	MHL_TX_DBG_INFO("bist -- received string: %s\n", buf);

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto exit;
	}

	str = strstr(buf, "data=");
	if (str != NULL) {
		dat = simple_strtoul(str + 5, NULL, 0);
		MHL_TX_DBG_INFO("bist data: %d\n", dat);
	} else {
		MHL_TX_DBG_ERR("Invalid string format, can't "
				"find ""data"" value\n");
		status = -EINVAL;
        goto exit;
	}

	str = strstr(buf, "cmd=");
	if (str != NULL) {
        pr_info("cmd=\n");
        if (strnicmp("bist_trigger", str+4, strlen("bist_trigger")) == 0) {
            //pr_info("bist_trigger\n");
            status = si_mhl_tx_bist_cmd(dev_context, MHL_MSC_MSG_BIST_TRIGGER, dat);
        } else if (strnicmp("bist_stop", str+4, strlen("bist_stop")) == 0) {
            //pr_info("bist_stop\n");
            status = si_mhl_tx_bist_cmd(dev_context, MHL_MSC_MSG_BIST_STOP, dat);
        } else if (strnicmp("bist_request_stat", str+4, strlen("bist_request_stat")) == 0) {
            //pr_info("bist_request_stat\n");
            status = si_mhl_tx_bist_cmd(dev_context, MHL_MSC_MSG_BIST_REQUEST_STAT, dat);
        } else {
        	MHL_TX_DBG_ERR("Unrecognized BIST cmd string\n");
        }

        if (status == BIST_STATUS_NO_ERROR)
        	status = count;
        else
        	status = -EINVAL;
    } else {
		MHL_TX_DBG_ERR("Invalid string format, can't "\
				"find ""cmd"" parameter\n");
		status = -EINVAL;
		goto exit;
	}

exit:
	MHL_TX_DBG_INFO("status = %d\n", status);
	return status;
}
#endif /* #ifdef BIST_INITIATOR */


#define MAX_EVENT_STRING_LEN 128
/*
 * Handler for event notifications from the MhlTx layer.
 *
 */
void mhl_event_notify(struct mhl_dev_context *dev_context, u32 event,
					  u32 event_param, void *data)
{
	char	event_string[MAX_EVENT_STRING_LEN];
	char	*envp[] = {event_string, NULL};
	char	*buf;
	u32		length;
	u32		count;
	int		idx;
	extern bool input_dev_rcp;

	MHL_TX_DBG_INFO("called, event: 0x%08x "\
			 "event_param: 0x%08x\n", event, event_param);

	/*
	 * Save the info on the most recent event.  This is done to support the
	 * SII_MHL_GET_MHL_TX_EVENT IOCTL.  If at some point in the future the
	 * driver's IOCTL interface is abandoned in favor of using sysfs attributes
	 * this can be removed.
	 */
	dev_context->pending_event = event;
	dev_context->pending_event_data = event_param;

	switch(event) {

	case MHL_TX_EVENT_CONNECTION:
		dev_context->mhl_flags |= MHL_STATE_FLAG_CONNECTED;

		if (input_dev_rcp){
			init_rcp_input_dev(dev_context);
		}
		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_CONN));

		strncpy(event_string, "MHLEVENT=connected", MAX_EVENT_STRING_LEN);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_DISCONNECTION:
		dev_context->mhl_flags = 0;
		dev_context->rcp_in_key_code = 0;
		dev_context->rcp_out_key_code = 0;
		dev_context->rcp_err_code = 0;
		dev_context->rcp_send_status = 0;
		dev_context->ucp_in_key_code = 0;
		dev_context->ucp_out_key_code = 0;
		dev_context->ucp_err_code = 0;
		dev_context->spad_send_status = 0;

#ifdef MEDIA_DATA_TUNNEL_SUPPORT
		mdt_destroy(dev_context);
#endif
		if (input_dev_rcp){
		destroy_rcp_input_dev(dev_context);
		}
		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_CONN));

		strncpy(event_string, "MHLEVENT=disconnected", MAX_EVENT_STRING_LEN);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_RCP_RECEIVED:

		if (input_dev_rcp){
			int result;
			result =generate_rcp_input_event(dev_context, (uint8_t)event_param);
			if (0 == result)
			si_mhl_tx_rcpk_send(dev_context, (uint8_t)event_param);
		else
			si_mhl_tx_rcpe_send(dev_context,
									MHL_RCPE_STATUS_INEFFECTIVE_KEY_CODE);
		}else{
		dev_context->mhl_flags &= ~MHL_STATE_FLAG_RCP_SENT;
		dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_RECEIVED;
			dev_context->rcp_in_key_code = event_param;

		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_RCP));

		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=received_RCP key code=0x%02x", event_param);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		}
		break;

	case MHL_TX_EVENT_RCPK_RECEIVED:
		if ((dev_context->mhl_flags & MHL_STATE_FLAG_RCP_SENT)
			&& (dev_context->rcp_out_key_code == event_param)) {

			dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_ACK;

			MHL_TX_DBG_INFO("Generating RCPK received event, "
					 "keycode: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_RCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_RCPK key code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR("Ignoring unexpected RCPK received "
					"event, keycode: 0x%02x\n", event_param);
		}
		break;

	case MHL_TX_EVENT_RCPE_RECEIVED:
		if (input_dev_rcp){
			/* do nothing */
		}else{
		if (dev_context->mhl_flags & MHL_STATE_FLAG_RCP_SENT) {

			dev_context->rcp_err_code = event_param;
			dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_NAK;

				MHL_TX_DBG_INFO("Generating RCPE received event, "
					 "error code: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_RCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_RCPE error code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
				MHL_TX_DBG_ERR("Ignoring unexpected RCPE received "
					"event, error code: 0x%02x\n", event_param);
		}
		}
		break;

	case MHL_TX_EVENT_UCP_RECEIVED:
		dev_context->mhl_flags &= ~MHL_STATE_FLAG_UCP_SENT;
		dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_RECEIVED;
		dev_context->ucp_in_key_code = event_param;
		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_UCP));

		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=received_UCP key code=0x%02x", event_param);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_UCPK_RECEIVED:
		if ((dev_context->mhl_flags & MHL_STATE_FLAG_UCP_SENT)
			&& (dev_context->ucp_out_key_code == event_param)) {

			dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_ACK;

			MHL_TX_DBG_INFO("Generating UCPK received event, "
					 "keycode: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_UCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_UCPK key code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR("Ignoring unexpected UCPK received "
					"event, keycode: 0x%02x\n", event_param);
		}
		break;

	case MHL_TX_EVENT_UCPE_RECEIVED:
		if (dev_context->mhl_flags & MHL_STATE_FLAG_UCP_SENT) {

			dev_context->ucp_err_code = event_param;
			dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_NAK;

			MHL_TX_DBG_INFO("Generating UCPE received event, "
					 "error code: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_UCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_UCPE error code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR("Ignoring unexpected UCPE received "
					"event, error code: 0x%02x\n", event_param);
		}
		break;

	case MHL_TX_EVENT_SPAD_RECEIVED:
		length = event_param;
		buf = data;

		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_SPAD));

		idx = snprintf(event_string, MAX_EVENT_STRING_LEN,
					   "MHLEVENT=SPAD_CHG length=0x%02x data=", length);

		count = 0;
		while (idx < MAX_EVENT_STRING_LEN) {
			if (count >= length)
				break;

			idx += snprintf(&event_string[idx], MAX_EVENT_STRING_LEN - idx,
							"0x%02x ", buf[count]);
			count++;
		}

		if (idx < MAX_EVENT_STRING_LEN) {
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR("Buffer too small to contain "
					"scratch pad data!\n");

		}
		break;

	case MHL_TX_EVENT_POW_BIT_CHG:
		MHL_TX_DBG_INFO("Generating VBUS power bit change "\
						"event, POW bit is %s\n", event_param? "ON" : "OFF");
		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=MHL VBUS power %s", event_param? "ON" : "OFF");
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_RAP_RECEIVED:
		MHL_TX_DBG_INFO("Generating RAP received event, "
						"action code: 0x%02x\n", event_param);

		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_RAP_IN));

		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=received_RAP action code=0x%02x", event_param);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	default:
		MHL_TX_DBG_ERR("called with unrecognized event code!\n");
		break;
	}
}

/*
 *  File operations supported by the MHL driver
 */
static const struct file_operations mhl_fops = {
    .owner			= THIS_MODULE
};

/*
 * Sysfs attribute files supported by this driver.
 */
struct device_attribute driver_attribs[] = {
	__ATTR(SYS_ATTR_NAME_CONN		,0444,show_connection_state, NULL),
	__ATTR(SYS_ATTR_NAME_SPAD		,0666,show_scratch_pad	, send_scratch_pad),
	__ATTR(SYS_ATTR_NAME_I2C_REGS	,0666,show_i2c			, set_i2c),
	__ATTR(SYS_ATTR_NAME_DEBUG_LEVEL,0666,get_debug_level	, set_debug_level),
	__ATTR(SYS_ATTR_NAME_REG_DEBUG_LEVEL,0666,get_reg_debug_level	, set_reg_debug_level),
	__ATTR(SYS_ATTR_NAME_GPIO_INDEX ,0666,get_gpio_index	, set_gpio_index),
	__ATTR(SYS_ATTR_NAME_GPIO_VALUE ,0666,get_gpio_level	, set_gpio_level),
	__ATTR(SYS_ATTR_NAME_AKSV		,0444,show_aksv			, NULL),
	__ATTR(SYS_ATTR_NAME_EDID		,0444,show_edid			, NULL),
	__ATTR(SYS_ATTR_NAME_HEV_3D_DATA,0444,show_hev_3d		, NULL),
#ifdef DEBUG
	__ATTR(SYS_ATTR_NAME_TX_POWER	,0222,NULL				, set_tx_power),
	__ATTR(SYS_ATTR_NAME_STARK_CTL	,0222,NULL				, set_stark_ctl),
#endif
#ifdef BIST_INITIATOR
	__ATTR(SYS_ATTR_NAME_BIST	,0222,NULL				, set_bist),
#endif
	__ATTR_NULL
};

ssize_t show_rap_in( struct device *dev
					,struct device_attribute *attr
					, char *buf
					)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("could not acquire mutex!!!\n");
		return -ERESTARTSYS;
	}
	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else {
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rap_in_sub_command);
	}
	up(&dev_context->isr_lock);

	return status;
}

/*
 * set_rap_in_status() - Handle write request to the rap attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t set_rap_in_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else{
		unsigned long param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0x00:
			dev_context->mhl_flags &= ~MHL_STATE_APPLICATION_RAP_BUSY;
			break;
		case 0x03:
			dev_context->mhl_flags |= MHL_STATE_APPLICATION_RAP_BUSY;
			break;
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rap_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rap_out_sub_command);
	}

	up(&dev_context->isr_lock);

	return status;
}

/*
 * send_rap_out() - Handle write request to the rap attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t send_rap_out(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned long param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case MHL_RAP_POLL:
		case MHL_RAP_CONTENT_ON:
		case MHL_RAP_CONTENT_OFF:
		case MHL_RAP_CBUS_MODE_DOWN:
		case MHL_RAP_CBUS_MODE_UP:
			if (!si_mhl_tx_rap_send(dev_context, param)){
				MHL_TX_DBG_ERR("-EPERM\n");
				status = -EPERM;
			}else{
				dev_context->rap_out_sub_command = (u8)param;
			}
			break;													//TODO: Lee - I added this - is it supposed to not be here?
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rap_out_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		// todo: check for un-ack'ed RAP sub command and block until ack received.
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rap_out_sub_command);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rap_input_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rap;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"%d\n",input_dev_rap);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_rap_input_dev(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_rap;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			input_dev_rap = param;
			break;
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}
static struct	device_attribute rap_in_attr 			=__ATTR(SYS_ATTR_NAME_RAP_IN		,0444,show_rap_in			,NULL);
static struct	device_attribute rap_in_status_attr 	=__ATTR(SYS_ATTR_NAME_RAP_IN_STATUS	,0222,NULL					,set_rap_in_status);
static struct	device_attribute rap_out_attr 			=__ATTR(SYS_ATTR_NAME_RAP_OUT		,0666,show_rap_out			,send_rap_out);
static struct	device_attribute rap_out_status_attr	=__ATTR(SYS_ATTR_NAME_RAP_OUT_STATUS,0444,show_rap_out_status	,NULL);
static struct	device_attribute rap_input_dev			=__ATTR(SYS_ATTR_NAME_RAP_INPUT_DEV	,0666,show_rap_input_dev	,set_rap_input_dev);

static struct attribute *rap_attrs[]={
	 &rap_in_attr.attr
	,&rap_in_status_attr.attr
	,&rap_out_attr.attr
	,&rap_out_status_attr.attr
	,&rap_input_dev.attr
	,NULL
};
static struct attribute_group rap_attribute_group ={
	 .name = __stringify(rap)
	,.attrs = rap_attrs
};

ssize_t show_rcp_in( struct device *dev
					,struct device_attribute *attr
					, char *buf
					)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("could not acquire mutex!!!\n");
		return -ERESTARTSYS;
	}
	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else if (input_dev_rcp){
		MHL_TX_DBG_INFO("\n");
		status = scnprintf(buf,PAGE_SIZE,"rcp_input_dev\n");
	}else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rcp_in_key_code);
	}
	up(&dev_context->isr_lock);

	return status;
}

/*
 * send_rcp_in_status() - Handle write request to the rcp attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t send_rcp_in_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else{
		unsigned long err_code;
		err_code = simple_strtoul(buf, NULL, 0);
		status = count;
		if (err_code == 0) {
			if (!si_mhl_tx_rcpk_send(dev_context, dev_context->rcp_in_key_code)) {
				status = -ENOMEM;
			}
		} else if (!si_mhl_tx_rcpe_send(dev_context, (u8)err_code)){
				status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rcp_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rcp_out_key_code);
	}

	up(&dev_context->isr_lock);

	return status;
}

/*
 * send_rcp_out() - Handle write request to the rcp attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t send_rcp_out(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned long param;
		param = simple_strtoul(buf, NULL, 0);
		dev_context->mhl_flags &= ~(MHL_STATE_FLAG_RCP_RECEIVED |
									MHL_STATE_FLAG_RCP_ACK |
									MHL_STATE_FLAG_RCP_NAK);
		dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_SENT;
		dev_context->rcp_send_status = 0;
		if (!si_mhl_tx_rcp_send(dev_context, (u8)param)){
			MHL_TX_DBG_ERR("-EPERM\n");
			status = -EPERM;
		}else{
			dev_context->rcp_out_key_code = param;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rcp_out_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		// todo: check for un-ack'ed RCP sub command and block until ack received.
		if (dev_context->mhl_flags & (MHL_STATE_FLAG_RCP_ACK | MHL_STATE_FLAG_RCP_NAK)) {
			status = scnprintf(buf,PAGE_SIZE,"0x%02x\n" ,dev_context->rcp_err_code);
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rcp_input_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",input_dev_rcp);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_rcp_input_dev(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			input_dev_rcp = param;
			break;
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

static struct	device_attribute rcp_in_attr 			=__ATTR(SYS_ATTR_NAME_RCP_IN		,0444,show_rcp_in		,NULL)             ;
static struct	device_attribute rcp_in_status_attr 	=__ATTR(SYS_ATTR_NAME_RCP_IN_STATUS ,0222,NULL				,send_rcp_in_status);
static struct	device_attribute rcp_out_attr 			=__ATTR(SYS_ATTR_NAME_RCP_OUT		,0666,show_rcp_out		,send_rcp_out)   ;
static struct	device_attribute rcp_out_status_attr	=__ATTR(SYS_ATTR_NAME_RCP_OUT_STATUS,0444,show_rcp_out_status,NULL)             ;
static struct	device_attribute rcp_input_dev			=__ATTR(SYS_ATTR_NAME_RCP_INPUT_DEV ,0666,show_rcp_input_dev,set_rcp_input_dev) ;

static struct attribute *rcp_attrs[]={
	 &rcp_in_attr.attr
	,&rcp_in_status_attr.attr
	,&rcp_out_attr.attr
	,&rcp_out_status_attr.attr
	,&rcp_input_dev.attr
	,NULL
};
static struct attribute_group rcp_attribute_group ={
	 .name = __stringify(rcp)
	,.attrs = rcp_attrs
};


ssize_t show_ucp_in( struct device *dev
					,struct device_attribute *attr
					, char *buf
					)
{
	//extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("could not acquire mutex!!!\n");
		return -ERESTARTSYS;
	}
	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else {
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->ucp_in_key_code);
	}
	up(&dev_context->isr_lock);

	return status;
}

/*
 * send_ucp_in_status() - Handle write request to the ucp attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t send_ucp_in_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	//extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else{
		unsigned long err_code;
		err_code = simple_strtoul(buf, NULL, 0);
		status = count;
		if (err_code == 0) {
			if (!si_mhl_tx_ucpk_send(dev_context, dev_context->ucp_in_key_code)) {
				status = -ENOMEM;
			}
		} else if (!si_mhl_tx_ucpe_send(dev_context, (u8)err_code)){
				status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	//extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->ucp_out_key_code);
	}

	up(&dev_context->isr_lock);

	return status;
}

/*
 * send_ucp_out() - Handle write request to the ucp attribute file.
 *
 * Writes to this file cause a RAP message with the specified action code
 * to be sent to the downstream device.
 */
ssize_t send_ucp_out(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		dev_context->mhl_flags &= ~(MHL_STATE_FLAG_UCP_RECEIVED |
									MHL_STATE_FLAG_UCP_ACK |
									MHL_STATE_FLAG_UCP_NAK);
		dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_SENT;
		if (!si_mhl_tx_ucp_send(dev_context, param)){
			MHL_TX_DBG_ERR("-EPERM\n");
			status = -EPERM;
		}else{
			dev_context->ucp_out_key_code = param;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp_out_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	//extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		// todo: check for un-ack'ed RAP sub command and block until ack received.
		if (dev_context->mhl_flags & (MHL_STATE_FLAG_UCP_ACK | MHL_STATE_FLAG_UCP_NAK)) {
			status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->ucp_err_code);
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp_input_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",input_dev_ucp);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_ucp_input_dev(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			input_dev_ucp = param;
			break;
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

static struct	device_attribute ucp_in_attr 			=__ATTR(SYS_ATTR_NAME_UCP_IN		,0444,show_ucp_in		,NULL)             ;
static struct	device_attribute ucp_in_status_attr 	=__ATTR(SYS_ATTR_NAME_UCP_IN_STATUS ,0222,NULL				,send_ucp_in_status);
static struct	device_attribute ucp_out_attr 			=__ATTR(SYS_ATTR_NAME_UCP_OUT		,0666,show_ucp_out		,send_ucp_out)   ;
static struct	device_attribute ucp_out_status_attr	=__ATTR(SYS_ATTR_NAME_UCP_OUT_STATUS,0444,show_ucp_out_status,NULL)             ;
static struct	device_attribute ucp_input_dev			=__ATTR(SYS_ATTR_NAME_UCP_INPUT_DEV ,0666,show_ucp_input_dev,set_ucp_input_dev) ;

static struct attribute *ucp_attrs[]={
	 &ucp_in_attr.attr
	,&ucp_in_status_attr.attr
	,&ucp_out_attr.attr
	,&ucp_out_status_attr.attr
	,&ucp_input_dev.attr
	,NULL
};
static struct attribute_group ucp_attribute_group ={
	 .name = __stringify(ucp)
	,.attrs = ucp_attrs
};


ssize_t show_devcap_local_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->dev_cap_local_offset);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_devcap_local_offset(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		if (param >= 16){
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
		}else{
			dev_context->dev_cap_local_offset= param;
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_devcap_local(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		extern uint8_t dev_cap_values[];
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_cap_values[dev_context->dev_cap_local_offset]);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_devcap_local(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		extern uint8_t dev_cap_values[];
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			dev_cap_values[dev_context->dev_cap_local_offset]= param;
			break;
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_devcap_remote_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->dev_cap_remote_offset);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_devcap_remote_offset(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		extern uint8_t dev_cap_values[];
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			dev_cap_values[dev_context->dev_cap_remote_offset]= param;
			break;
		default:
			MHL_TX_DBG_ERR("Invalid parameter %s received\n", buf);
			status = -EINVAL;
			break;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_devcap_remote(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		uint8_t	regValue;
		status = si_mhl_tx_get_peer_dev_cap_entry(dev_context,
											dev_context->dev_cap_remote_offset,
											&regValue);
		if (status != 0) {
			/*
			 * Driver is busy and cannot provide the requested DEVCAP
			 * register value right now so inform caller they need to
			 * try again later.
			 */
			status = -EAGAIN;
		}else{
			status = scnprintf(buf, PAGE_SIZE, "0x%02x",regValue);
		}
	}

	up(&dev_context->isr_lock);

	return status;
}
static struct	device_attribute attr_devcap_local_offset	=__ATTR(SYS_ATTR_NAME_DEVCAP_LOCAL_OFFSET	,0666,show_devcap_local_offset	,set_devcap_local_offset	);
static struct	device_attribute attr_devcap_local			=__ATTR(SYS_ATTR_NAME_DEVCAP_LOCAL			,0666,show_devcap_local			,set_devcap_local			);
static struct	device_attribute attr_devcap_remote_offset	=__ATTR(SYS_ATTR_NAME_DEVCAP_REMOTE_OFFSET	,0666,show_devcap_remote_offset	,set_devcap_remote_offset	);
static struct	device_attribute attr_devcap_remote			=__ATTR(SYS_ATTR_NAME_DEVCAP_REMOTE			,0444,show_devcap_remote		,NULL						);

static struct attribute *devcap_attrs[]={
	 &attr_devcap_local_offset.attr
	,&attr_devcap_local.attr
	,&attr_devcap_remote_offset.attr
	,&attr_devcap_remote.attr
	,NULL
};
static struct attribute_group devcap_attribute_group ={
	 .name = __stringify(SYS_OBJECT_NAME_DEVCAP)
	,.attrs = devcap_attrs
};

ssize_t set_hdcp_force_content_type(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	/* Assume success */
	status = count;

	MHL_TX_DBG_INFO("received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	}else{
		extern	int	hdcp_content_type;
		hdcp_content_type = simple_strtol(buf, NULL, 0);
		/* Wait until the next mode set... */
		/* if (-1 != hdcp_content_type){ force hdcp reauthentication } */
		status = count;
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_hdcp_force_content_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR("-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR("-ENODEV\n");
		status = -ENODEV;
	} else{
		extern	int	hdcp_content_type;
		status = scnprintf(buf, PAGE_SIZE, "%d",hdcp_content_type);
	}

	up(&dev_context->isr_lock);

	return status;
}

static struct	device_attribute attr_hdcp_force_content_type	=__ATTR(SYS_ATTR_NAME_HDCP_CONTENT_TYPE,0666,show_hdcp_force_content_type	,set_hdcp_force_content_type);
static struct attribute *hdcp_attrs[]={
	 &attr_hdcp_force_content_type.attr
	,NULL
};

static struct attribute_group hdcp_attribute_group ={
	 .name = __stringify(SYS_OBJECT_NAME_HDCP)
	,.attrs = hdcp_attrs
};

/*
 * Interrupt handler for MHL transmitter interrupts.
 *
 * @irq:	The number of the asserted IRQ line that caused
 *  		this handler to be called.
 * @data:	Data pointer passed when the interrupt was enabled,
 *  		which in this case is a pointer to an mhl_dev_context struct.
 *
 * Always returns IRQ_HANDLED.
 */
static irqreturn_t mhl_irq_handler(int irq, void *data)
{
	struct mhl_dev_context	*dev_context = (struct mhl_dev_context *)data;
	//int i=0;
	MHL_TX_DBG_INFO("[sii8620] called mhl_irq_handler\n");
	printk(KERN_ERR"[sii8620]start\n");
	//msleep(30);
//if(1)
	//return IRQ_HANDLED;
	if (!down_interruptible(&dev_context->isr_lock)) 
		{
		int loop_limit=0;
		if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN)
			goto irq_done;
		if (dev_context->dev_flags & DEV_FLAG_COMM_MODE)
			goto irq_done;
		/* do bottom tested loop to ensure that it gets executed at least once */
		do{
			//for(i=0;i<300;i++){
			printk(KERN_ERR "[sii8620]start%s, loop_limit=%d\n",__FUNCTION__,loop_limit++);


			memset(&dev_context->intr_info, 0, sizeof(*(&dev_context->intr_info)));

			dev_context->intr_info.edid_parser_context =
					dev_context->edid_parser_context;
			
			dev_context->drv_info->mhl_device_isr((struct drv_hw_context *)
												  (&dev_context->drv_context),
												  &dev_context->intr_info);
				//msleep(30);
			
//#if 0
			/* Now post process events detected by the interrupt handler */
			if(dev_context->intr_info.flags & DRV_INTR_FLAG_DISCONNECT) {
				dev_context->misc_flags.flags.rap_content_on = false;
				dev_context->misc_flags.flags.mhl_rsen = false;
				dev_context->mhl_connection_event = true;
				dev_context->mhl_connected = MHL_TX_EVENT_DISCONNECTION;
				si_mhl_tx_process_events(dev_context);
			} else {
				if (dev_context->intr_info.flags & DRV_INTR_FLAG_CONNECT) {
					dev_context->misc_flags.flags.rap_content_on = true;
					dev_context->rap_in_sub_command = MHL_RAP_CONTENT_ON;
					dev_context->misc_flags.flags.mhl_rsen = true;
					dev_context->mhl_connection_event = true;
					dev_context->mhl_connected = MHL_TX_EVENT_CONNECTION;
					si_mhl_tx_process_events(dev_context);
					// Start timer here to circumvent issue of not getting DCAP_RDY
					mhl_tx_start_timer(dev_context,dev_context->dcap_rdy_dcap_chg_timer,2000);
				}

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_CBUS_ABORT)
					process_cbus_abort(dev_context);

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_WRITE_BURST)
					si_mhl_tx_process_write_burst_data(dev_context);

				/* some times SET_INT and WRITE_STAT come at the same time...
					Always process WRITE_STAT first.
				*/
				if (dev_context->intr_info.flags & DRV_INTR_FLAG_WRITE_STAT)
					si_mhl_tx_got_mhl_status(dev_context,
							&dev_context->intr_info.dev_status);

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_SET_INT)
					si_mhl_tx_got_mhl_intr(dev_context,
										   dev_context->intr_info.int_msg[0],
										   dev_context->intr_info.int_msg[1]);

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_MSC_DONE)
					si_mhl_tx_msc_command_done(dev_context,
							dev_context->intr_info.msc_done_data);

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_EMSC_INCOMING)
					si_mhl_tx_emsc_received(dev_context);

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_HPD_CHANGE)
					si_mhl_tx_notify_downstream_hpd_change(dev_context,
							dev_context->intr_info.hpd_status);

				if (dev_context->intr_info.flags & DRV_INTR_FLAG_MSC_RECVD) {
					dev_context->msc_msg_arrived		= true;
					dev_context->msc_msg_sub_command	=
							dev_context->intr_info.msc_msg[0];
					dev_context->msc_msg_data			=
							dev_context->intr_info.msc_msg[1];
					si_mhl_tx_process_events(dev_context);
				}
				if (DRV_INTR_FLAG_COC_CAL & dev_context->intr_info.flags) {
					if (dev_context->bist_setup.bist_trigger_cmd) {
						MHL_TX_DBG_INFO("would have called initiate_bist_test\n");
//						initiate_bist_test(dev_context);
#ifdef BIST_INITIATOR
					} else if (
						dev_context->initiator_bist_setup.bist_trigger_cmd) {
						start_bist_initiator_test(dev_context);
#endif
					}
				}
				/*
					MHL3 spec requires the process of reading
						the remainder of XDEVCAP and all of DEVCAP
						to be deferred until after the first transition
						to eCBUS mode.
					So: check DCAP_RDY and initiate XDEVCAP reading here.

				*/

				if (DRV_INTR_FLAG_TDM_SYNC & dev_context->intr_info.flags) {
					if (MHL_STATUS_DCAP_RDY & dev_context->status_0) {
						si_mhl_tx_ecbus_started(dev_context);
					}
					si_mhl_tx_initialize_block_transport(dev_context);
					if (dev_context->misc_flags.flags.mhl_hpd) {
						if (!dev_context->edid_valid){
							if (dev_context->misc_flags.flags.have_complete_devcap) {
								si_mhl_tx_initiate_edid_sequence(
									dev_context->edid_parser_context);
							}
						}
					}
				}
			}
			/*
			 *  Check to see if we can send any messages that may have
			 * been queued up as the result of interrupt processing.
			 */
			si_mhl_tx_drive_states(dev_context);
			if (si_mhl_tx_drv_get_cbus_mode(dev_context)>=CM_eCBUS_S){
				si_mhl_tx_push_block_transactions(dev_context);
			}
				//}
			printk("[sii8620]end%s\n",__FUNCTION__);
		}while (is_interrupt_asserted());
irq_done:
		up(&dev_context->isr_lock);
		;
	}

	return IRQ_HANDLED;
}

/* APIs provided by the MHL layer to the lower level driver */

int mhl_handle_power_change_request(struct device *parent_dev, bool power_up)
{
	struct mhl_dev_context *dev_context;
	int						status;

	dev_context = dev_get_drvdata(parent_dev);

	MHL_TX_DBG_INFO("\n");

    /* Power down the MHL transmitter hardware. */
	status = down_interruptible(&dev_context->isr_lock);
	if (status) {
		MHL_TX_DBG_ERR("failed to acquire ISR semaphore,"\
						"status: %d\n", status);
		goto done;
	}

	if (power_up)
		status = si_mhl_tx_initialize(dev_context);
	else
		status = si_mhl_tx_shutdown(dev_context);

	up(&dev_context->isr_lock);
done:
	return status;

}
//#define SII8620_POLLING 1
#ifdef SII8620_POLLING
/*****************************************************************************/
/**
 *  @brief Thread function that periodically polls for MHLTx events.
 *
 *  @param[in]	data	Pointer to driver context structure
 *
 *  @return		Always returns zero when the thread exits.
 *
 *****************************************************************************/
//static struct mhl_drv_info *drv_info_bk;
//static struct mhl_dev_context *dev_context_bk;
static struct mhl_drv_info drv_info_bk;
static struct mhl_dev_context dev_context_bk;

static int EventThread(void *data)
{
	printk("%s EventThread starting up\n", MHL_DRIVER_NAME);

	while (true)
	{
		if (0)
		{
			printk("%s EventThread exiting\n", MHL_DRIVER_NAME);
			break;
		}

		msleep(30);
		//SiiMhlTxDeviceIsr();
		if(1)mhl_irq_handler(gpio_to_irq(GPIO_MHL_INT),(void *)(&dev_context_bk));
	}
	return 0;
}


/***** public functions ******************************************************/


/*****************************************************************************/
/**
 * @brief Start drivers event monitoring thread.
 *
 *****************************************************************************/
void StartEventThread(void)
{
	pr_err("%s!!!!!\n",__FUNCTION__);
	kthread_run(EventThread,&dev_context_bk, MHL_DRIVER_NAME);
}

#endif
//added by zhangyue on 2014-05-22 for qualcomm platform start

#define GPIO_BB_ID_SEL                        26

static int mhl_sii_wait_for_rgnd(struct mhl_dev_context *dev_context)
{
	int timeout;

	pr_warn("%s:%u\n", __func__, __LINE__);

	if (dev_context->mhl_mode) {
		pr_debug("%s: already in mhl mode\n", __func__);
		return 0;
	}
	
	INIT_COMPLETION(dev_context->rgnd_done);
	/*
	 * after toggling reset line and enabling disc
	 * tx can take a while to generate intr
	 */

	timeout = wait_for_completion_timeout
		(&dev_context->rgnd_done, HZ * 3);
	if (!timeout) {
		/*
		 * most likely nothing plugged in USB
		 * USB HOST connected or already in USB mode
		 */
		pr_warn("%s:%u timedout\n", __func__, __LINE__);
		return -ENODEV;
	}
	
	return 0;
}


static int mhl_sii_device_discovery(void *data, int id,
			     void (*usb_notify_cb)(void *, int), void *ctx)
{
	int rc;
	struct mhl_dev_context *dev_context= data;
	//unsigned long flags;
	pr_warn("%s:%u\n", __func__, __LINE__);

	if (id) {
		/* When MHL cable is disconnected we get a sii8334
		 * mhl_disconnect interrupt which is handled separately.
		 */
		pr_warn("%s: USB ID pin high\n", __func__);
		return id;
	}

	if (!dev_context || !usb_notify_cb) {
		pr_warn("%s: cb || dev_context is NULL\n", __func__);
		/* return "USB" so caller can proceed */
		return -EINVAL;
	}

	if(!(dev_context->qualcomm_p_data->notify_usb_online)){
		dev_context->qualcomm_p_data->notify_usb_online = usb_notify_cb;
		dev_context->qualcomm_p_data->notify_ctx = ctx;
	}

	
	//change switch to MHL 
	//set_pin(X02_USB_SW_CTRL, 1);
	gpio_direction_output(GPIO_BB_ID_SEL, 1);
	
	if(mhl_sii_wait_for_rgnd(dev_context)) {
		pr_err("%s: discovery timeout\n", __func__);

		//mhl_sii_config(mhl_ctrl, false);
		//change switch to usb
		//set_pin(X02_USB_SW_CTRL, 0);
		gpio_direction_output(GPIO_BB_ID_SEL, 0);

		return -EAGAIN;
	}


	rc = dev_context->mhl_mode ? 0 : 1;
	if(rc){
		//mhl_sii_config(mhl_ctrl, false);
		//change switch to usb
		//set_pin(X02_USB_SW_CTRL, 0); //switch to usb
		gpio_direction_output(GPIO_BB_ID_SEL, 0);
	}

	pr_err("%s: ret result: %s\n", __func__, rc ? "usb" : " mhl");
	return rc;
}

#define MAX_CURRENT 700000

static char *mhl_pm_power_supplied_to[] = {
	"usb",
};

static enum power_supply_property mhl_pm_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};
static int mhl_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct qualcomm_platform_data *qualcomm_p_data = NULL;
	struct mhl_dev_context *dev_context = NULL;

	if(psy){
		qualcomm_p_data = container_of(psy, struct qualcomm_platform_data, mhl_psy);
		dev_context = qualcomm_p_data->dev_context;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = qualcomm_p_data->current_val;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = qualcomm_p_data->vbus_active;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = qualcomm_p_data->vbus_active && dev_context->mhl_mode;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mhl_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct qualcomm_platform_data *qualcomm_p_data = NULL;
	struct mhl_dev_context *dev_context = NULL;

	qualcomm_p_data = container_of(psy, struct qualcomm_platform_data, mhl_psy);
	dev_context = qualcomm_p_data->dev_context;


	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		qualcomm_p_data->vbus_active = val->intval;
		if (qualcomm_p_data->vbus_active)
			qualcomm_p_data->current_val = MAX_CURRENT;
		else
			qualcomm_p_data->current_val = 0;
		power_supply_changed(psy);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int mhl_tx_get_platform_data(struct device *dev,
	struct mhl_dev_context *dev_context,
	struct qualcomm_platform_data *pdata)
{
	int  rc = 0;
	struct device_node *of_node = NULL;
	struct platform_device *hdmi_pdev = NULL;
	struct device_node *hdmi_tx_node = NULL;
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops = NULL;
	struct usb_ext_notification *mhl_info = NULL;
	//struct power_supply *mhl_psy;

	if (!dev || !pdata ||!dev_context) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}
	
	hdmi_mhl_ops = kzalloc(sizeof(struct msm_hdmi_mhl_ops),GFP_KERNEL);
	if(!hdmi_mhl_ops){
		return -ENOMEM;
	}

	mhl_info = kzalloc(sizeof(struct usb_ext_notification),GFP_KERNEL);
	if(!mhl_info){
		return -ENOMEM;
	}


	dev_context->qualcomm_p_data = pdata;
	pdata->dev_context = dev_context;

	of_node = dev->of_node;
	if (!of_node) {
		pr_err("%s: invalid of_node\n", __func__);
		goto error;
	}

	pr_warn("%s: id=%d\n", __func__, dev->id);
	
	/* parse phandle for hdmi tx */
	hdmi_tx_node = of_parse_phandle(of_node, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		goto error;
	}
	
	hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		goto error;
	}
	pr_debug("%s: hdmi_pdev [0X%x] to pdata->pdev\n",
	       __func__, (unsigned int)hdmi_pdev);

	pdata->hdmi_pdev = hdmi_pdev;

	if(pdata->hdmi_pdev){
		rc = msm_hdmi_register_mhl(pdata->hdmi_pdev,hdmi_mhl_ops, dev_context);
		if(rc){
			pr_err("%s: register with hdmi failed\n", __func__);
		}
	}
	//if(hdmi_mhl_ops->set_mhl_max_pclk)
		//hdmi_mhl_ops->set_mhl_max_pclk(pdata->hdmi_pdev,75000); // Max pixel clock 75MHz
	pdata->hdmi_mhl_ops = hdmi_mhl_ops;
	pdata->mhl_psy.name = "ext-vbus";
	pdata->mhl_psy.type = POWER_SUPPLY_TYPE_USB_DCP;
	pdata->mhl_psy.supplied_to = mhl_pm_power_supplied_to;
	pdata->mhl_psy.num_supplicants = ARRAY_SIZE(
					mhl_pm_power_supplied_to);
	pdata->mhl_psy.properties = mhl_pm_power_props;
	pdata->mhl_psy.num_properties = ARRAY_SIZE(mhl_pm_power_props);
	pdata->mhl_psy.get_property = mhl_power_get_property;
	pdata->mhl_psy.set_property = mhl_power_set_property;

	rc = power_supply_register(dev, &pdata->mhl_psy);
	if (rc < 0) {
		dev_err(dev, "%s:power_supply_register ext_vbus_psy failed\n",
			__func__);
	}

	//pdata->mhl_psy = *mhl_psy;
	
	mhl_info->ctxt = dev_context;
	mhl_info->notify = mhl_sii_device_discovery;
	if (msm_register_usb_ext_notification(mhl_info)) {
		pr_err("%s: register for usb notifcn failed\n", __func__);
		rc = -EPROBE_DEFER;
	}
	pdata->mhl_info = mhl_info;

	pdata->notify_usb_online = NULL;
	pdata->notify_ctx = NULL;
	
	

	return 0;
error:
	pr_err("%s: ret due to err\n", __func__);
	return rc;
}

//added by zhangyue on 2014-05-22 for qualcomm platform end
int mhl_tx_init(struct mhl_drv_info const *drv_info,
				struct device *parent_dev)
{
	struct mhl_dev_context *dev_context;
	int		ret,dummy;
	//added by zhang yue for qualcomm platform data start
	struct qualcomm_platform_data * qualcomm_p_data;
	//added by zhang yue for qualcomm platform data end

	if (drv_info == NULL || parent_dev == NULL) {
		pr_err("Null parameter passed to %s\n",__FUNCTION__);
		return -EINVAL;
	}
#if 0
	if (drv_info->mhl_device_isr == NULL || drv_info->irq == 0) {
		dev_err(parent_dev, "No IRQ specified!\n");
		return -EINVAL;
	}
#endif
	dev_context = kzalloc(sizeof(*dev_context) + drv_info->drv_context_size,
								 GFP_KERNEL);
	if (!dev_context) {
		dev_err(parent_dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	dev_context->signature = MHL_DEV_CONTEXT_SIGNATURE;
	dev_context->drv_info = drv_info;
	
	sema_init(&dev_context->mhl_emsc.data_wait_lock, 1);

	sema_init(&dev_context->isr_lock, 1);
	INIT_LIST_HEAD(&dev_context->timer_list);
	dev_context->timer_work_queue = create_workqueue(MHL_DRIVER_NAME);
	if (dev_context->timer_work_queue == NULL) {
		ret = -ENOMEM;
		goto free_mem;
	}

	if (mhl_class == NULL) {
		mhl_class = class_create(THIS_MODULE, "mhl");
		if(IS_ERR(mhl_class)) {
			ret = PTR_ERR(mhl_class);
			pr_info("class_create failed %d\n", ret);
			goto err_exit;
		}

		mhl_class->dev_attrs = driver_attribs;

        ret = alloc_chrdev_region(&dev_num,
                        0, MHL_DRIVER_MINOR_MAX,
                        MHL_DRIVER_NAME);

        if (ret) {
        	pr_info("register_chrdev %s failed, error code: %d\n",
        			MHL_DRIVER_NAME, ret);
        	goto free_class;
        }

        cdev_init(&dev_context->mhl_cdev, &mhl_fops);
        dev_context->mhl_cdev.owner = THIS_MODULE;
        ret = cdev_add(&dev_context->mhl_cdev, MINOR(dev_num), MHL_DRIVER_MINOR_MAX);
        if (ret) {
        	pr_info("cdev_add %s failed %d\n", MHL_DRIVER_NAME, ret);
        	goto free_chrdev;
        }
	}

	dev_context->mhl_dev = device_create(mhl_class, parent_dev,
	    									 dev_num, dev_context,
	    									 "%s", MHL_DEVICE_NAME);
    if (IS_ERR(dev_context->mhl_dev)) {
        ret = PTR_ERR(dev_context->mhl_dev);
    	pr_info("device_create failed %s %d\n", MHL_DEVICE_NAME, ret);
        goto free_cdev;
    }
	{
		int status = 0;
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&rap_attribute_group);
		if(status){
			MHL_TX_DBG_ERR("sysfs_create_group failed:%d\n",status);
		}
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&rcp_attribute_group);
		if(status){
			MHL_TX_DBG_ERR("sysfs_create_group failed:%d\n",status);
		}
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&ucp_attribute_group);
		if(status){
			MHL_TX_DBG_ERR("sysfs_create_group failed:%d\n",status);
		}
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&devcap_attribute_group);
		if(status){
			MHL_TX_DBG_ERR("sysfs_create_group failed:%d\n",status);
		}

		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&hdcp_attribute_group);
		if(status){
			MHL_TX_DBG_ERR("sysfs_create_group failed:%d\n",status);
		}
	}
	
if(1)
{
	ret = request_threaded_irq(drv_info->irq, NULL,
								mhl_irq_handler,
								IRQF_TRIGGER_LOW | IRQF_ONESHOT, //IRQF_TRIGGER_FALLING //IRQF_TRIGGER_LOW | IRQF_ONESHOT,
								MHL_DEVICE_NAME,
								dev_context);
	if (ret < 0) {
		dev_err(parent_dev, "request_threaded_irq failed, status: %d\n", ret);
		//goto free_device;
	}
}
	//return 0;

    /* Initialize the MHL transmitter hardware. */
	ret = down_interruptible(&dev_context->isr_lock);
	if (ret) {
		dev_err(parent_dev, "failed to acquire ISR semaphore, status: %d\n", ret);
		goto free_irq_handler;
	}

	/* Initialize EDID parser module */
	dev_context->edid_parser_context = si_edid_create_context(dev_context,(struct drv_hw_context *)&dev_context->drv_context);
	rcp_input_dev_one_time_init(dev_context);

	ret = si_mhl_tx_initialize(dev_context);
	up(&dev_context->isr_lock);

	//if (ret)
	//	goto free_irq_handler;
if(1){
	MHL_TX_DBG_INFO("MHL transmitter successfully initialized\n");
	#ifdef SII8620_POLLING
	memcpy(&drv_info_bk,dev_context->drv_info,sizeof(struct mhl_drv_info));
	memcpy(&dev_context_bk,dev_context,sizeof(struct mhl_dev_context));
	//StartEventThread();
	#endif
	dev_set_drvdata(parent_dev, dev_context);
	//added by zhangyue for qualcomm platform start 
	init_completion(&dev_context->rgnd_done);
	qualcomm_p_data = kzalloc(sizeof(struct qualcomm_platform_data), GFP_KERNEL);
	
	if(!qualcomm_p_data){
		dev_err(parent_dev, "failed to allocate driver data\n");
		ret = -ENOMEM;
		//goto free_edid_context;
		goto free_irq_handler;
	}

	ret = mhl_tx_get_platform_data(parent_dev ,dev_context,qualcomm_p_data);
	if(ret){
		dev_err(parent_dev, "failed to Get qualcomm platform datat\n");
		goto free_platform_data;
		
	}
	return ret;
	//added by zhangyue for qualcomm platform end 
	//added by zhangyue for qualcomm platform start	
free_platform_data:
	MHL_TX_DBG_INFO("%sMHL transmitter initialization failed%s\n",
			ANSI_ESC_RED_TEXT, ANSI_ESC_RESET_TEXT);
	if(qualcomm_p_data){
		if(qualcomm_p_data->mhl_info)
			kfree(qualcomm_p_data->mhl_info);
		if(qualcomm_p_data->hdmi_mhl_ops)
			kfree(qualcomm_p_data->hdmi_mhl_ops);
		//if(qualcomm_p_data->mhl_psy)
			//kfree(qualcomm_p_data->mhl_psy);
		qualcomm_p_data->mhl_info = NULL;
		qualcomm_p_data->hdmi_mhl_ops = NULL;
		//qualcomm_p_data->mhl_psy = NULL;

		kfree(qualcomm_p_data->mhl_info);
	}
//added by zhangyue for qualcomm platform end 
free_irq_handler:
	MHL_TX_DBG_INFO("%sMHL transmitter initialization failed%s\n"
				,ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
	dummy = down_interruptible(&dev_context->isr_lock);
	if(dev_context->edid_parser_context)
		si_edid_destroy_context(dev_context->edid_parser_context);

	if ( dev_context->client )
		free_irq(dev_context->client->irq, dev_context);
//free_device:
	device_destroy(mhl_class, dev_num);

free_cdev:
	cdev_del(&dev_context->mhl_cdev);

free_chrdev:
	unregister_chrdev_region(dev_num, MHL_DRIVER_MINOR_MAX);
	dev_num = 0;

free_class:
	class_destroy(mhl_class);

err_exit:
	destroy_workqueue(dev_context->timer_work_queue);

free_mem:
	kfree(dev_context);

	return ret;
}
}

int mhl_tx_remove(struct device *parent_dev)
{
	struct mhl_dev_context	*dev_context;
	int						ret = 0;

	dev_context = dev_get_drvdata(parent_dev);

	if (dev_context != NULL){
		extern bool input_dev_rcp;
		MHL_TX_DBG_INFO("%x\n",dev_context);
		ret = down_interruptible(&dev_context->isr_lock);

		dev_context->dev_flags |= DEV_FLAG_SHUTDOWN;

		mhl3_hid_remove_all(dev_context);

		ret = si_mhl_tx_shutdown(dev_context);

		mhl_tx_destroy_timer_support(dev_context);

		up(&dev_context->isr_lock);

		free_irq(dev_context->drv_info->irq, dev_context);

		sysfs_remove_group(&dev_context->mhl_dev->kobj,&rap_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&rcp_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&ucp_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&devcap_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&hdcp_attribute_group);

		device_destroy(mhl_class, dev_num);

		cdev_del(&dev_context->mhl_cdev);

		unregister_chrdev_region(dev_num, MHL_DRIVER_MINOR_MAX);
		dev_num = 0;

		class_destroy(mhl_class);
		mhl_class = NULL;

	#ifdef MEDIA_DATA_TUNNEL_SUPPORT
		mdt_destroy(dev_context);
	#endif
		if (input_dev_rcp)
			destroy_rcp_input_dev(dev_context);

		si_edid_destroy_context(dev_context->edid_parser_context);

		kfree(dev_context);

		MHL_TX_DBG_INFO("%x\n",dev_context);
	}
	return ret;
}

static void mhl_tx_destroy_timer_support(struct  mhl_dev_context *dev_context)
{
	struct timer_obj	*mhl_timer;

	/*
	 * Make sure all outstanding timer objects are canceled and the
	 * memory allocated for them is freed.
	 */
	while(!list_empty(&dev_context->timer_list)) {
		mhl_timer = list_first_entry(&dev_context->timer_list,
									 struct timer_obj, list_link);
		hrtimer_cancel(&mhl_timer->hr_timer);
		list_del(&mhl_timer->list_link);
		kfree(mhl_timer);
	}

	destroy_workqueue(dev_context->timer_work_queue);
	dev_context->timer_work_queue = NULL;
}

static void mhl_tx_timer_work_handler(struct work_struct *work)
{
	struct timer_obj	*mhl_timer;

	mhl_timer = container_of(work, struct timer_obj, work_item);

	mhl_timer->flags |= TIMER_OBJ_FLAG_WORK_IP;
	if (!down_interruptible(&mhl_timer->dev_context->isr_lock)) {

		mhl_timer->timer_callback_handler(mhl_timer->callback_param);

		up(&mhl_timer->dev_context->isr_lock);
	}
	mhl_timer->flags &= ~TIMER_OBJ_FLAG_WORK_IP;

	if(mhl_timer->flags & TIMER_OBJ_FLAG_DEL_REQ) {
		/*
		 * Deletion of this timer was requested during the execution of
		 * the callback handler so go ahead and delete it now.
		 */
		kfree(mhl_timer);
	}
}

static enum hrtimer_restart mhl_tx_timer_handler(struct hrtimer *timer)
{
	struct timer_obj	*mhl_timer;

	mhl_timer = container_of(timer, struct timer_obj, hr_timer);

	queue_work(mhl_timer->dev_context->timer_work_queue,
			   &mhl_timer->work_item);

	return HRTIMER_NORESTART;
}

static int is_timer_handle_valid(struct mhl_dev_context *dev_context,
								 void *timer_handle)
{
	struct timer_obj	*timer;

	list_for_each_entry(timer, &dev_context->timer_list, list_link) {
		if (timer == timer_handle)
			break;
	}

	if(timer != timer_handle) {
    	MHL_TX_DBG_WARN("Invalid timer handle %p received\n",
    				   timer_handle);
		return -EINVAL;
	}
	return 0;
}

int mhl_tx_create_timer(void *context,
						void (*callback_handler)(void *callback_param),
						void *callback_param,
						void **timer_handle)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*new_timer;

	dev_context = get_mhl_device_context(context);

	if (callback_handler == NULL)
		return -EINVAL;

	if (dev_context->timer_work_queue == NULL)
		return -ENOMEM;

	new_timer = kmalloc(sizeof(*new_timer), GFP_KERNEL);
	if (new_timer == NULL)
		return -ENOMEM;

	new_timer->timer_callback_handler = callback_handler;
	new_timer->callback_param = callback_param;
	new_timer->flags = 0;

	new_timer->dev_context = dev_context;
	INIT_WORK(&new_timer->work_item, mhl_tx_timer_work_handler);

	list_add(&new_timer->list_link, &dev_context->timer_list);

	hrtimer_init(&new_timer->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	new_timer->hr_timer.function = mhl_tx_timer_handler;
	*timer_handle = new_timer;
	return 0;
}

int mhl_tx_delete_timer(void *context, void **timer_handle)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*timer;
	int						status;

	dev_context = get_mhl_device_context(context);

	status = is_timer_handle_valid(dev_context, *timer_handle);
	if (status == 0) {
		timer = *timer_handle;

		list_del(&timer->list_link);

		hrtimer_cancel(&timer->hr_timer);

		if(timer->flags & TIMER_OBJ_FLAG_WORK_IP) {
			/*
			 * Request to delete timer object came from within the timer's
			 * callback handler.  If we were to proceed with the timer deletion
			 * we would deadlock at cancel_work_sync().  So instead just flag
			 * that the user wants the timer deleted.  Later when the timer
			 * callback completes the timer's work handler will complete the
			 * process of deleting this timer.
			 */
			timer->flags |= TIMER_OBJ_FLAG_DEL_REQ;
		} else {
			cancel_work_sync(&timer->work_item);
			*timer_handle = NULL;
			kfree(timer);
		}
	}

	return status;
}

int mhl_tx_start_timer(void *context, void *timer_handle,
					   uint32_t time_msec)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*timer;
	ktime_t					timer_period;
	int						status;

	dev_context = get_mhl_device_context(context);

	status = is_timer_handle_valid(dev_context, timer_handle);
	if (status == 0) {
		long secs=0;
		timer = timer_handle;

		secs=time_msec/1000;
		time_msec %= 1000;
		timer_period = ktime_set(secs, MSEC_TO_NSEC(time_msec));
		hrtimer_start(&timer->hr_timer, timer_period, HRTIMER_MODE_REL);
	}

	return status;
}

int mhl_tx_stop_timer(void *context, void *timer_handle)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*timer;
	int						status;

	dev_context = get_mhl_device_context(context);

	status = is_timer_handle_valid(dev_context, timer_handle);
	if (status == 0) {
		timer = timer_handle;

		hrtimer_cancel(&timer->hr_timer);
	}
	return status;
}
