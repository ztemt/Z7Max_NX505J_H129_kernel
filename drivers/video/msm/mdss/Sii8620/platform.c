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
#include <linux/moduleparam.h>
#include <linux/kernel.h>


#include <linux/types.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>

#include "si_fw_macros.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl_defs.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include <linux/input.h>
#include "si_mdt_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "platform.h"
#include "si_mhl_callback_api.h"
#include "si_8620_drv.h"
#include "si_8620_regs.h"

#include <linux/of_gpio.h>
/*#include "mhl_msc.h"
#include "mdss_hdmi_mhl.h"
#include <linux/vmalloc.h>*/
#ifndef CONFIG_OF
#define CONFIG_OF
#endif
int si_mhl_tx_get_num_block_reqs(void); /*from mhl_supp.c */

/* System GPIOs assigned to control various starter kit signals */
#define GPIO_MHL_INT				102//138	/* W_INT */
#define GPIO_BB_RESET				78//140	/* P_BB_RST# */
#define GPIO_BB_FW_WAKE                   27
#define GPIO_BB_POWER_1V0                14

#define GPIO_BB_ID_SEL                        26
//#define I2C_CLK_TEST
#ifdef I2C_CLK_TEST
#define GPIO_I2C_CLK                             3
#endif

#define GPIO_EXP_ADDR				0x40
/*
 * NOTE: The following GPIO expander register type address
 * offsets are all defined with the address auto-increment
 * bit set (0x80)
 */
#define GPIO_EXP_INPUT_REGS_OFFSET		0x80
#define GPIO_EXP_OUTPUT_REGS_OFFSET		0x88
#define GPIO_EXP_POL_INVERT_REGS_OFFSET		0x90
#define GPIO_EXP_IO_CONFIG_REGS_OFFSET		0x98
#define GPIO_EXP_INTR_MASK_REGS_OFFSET		0xA0

#define GPIO_EXP_BANK_2_OUTPUT_DEFAULT		0xFF
#define GPIO_EXP_BANK_2_3D			(0x01 << 0)
#define GPIO_EXP_BANK_2_PKD_PXL			(0x01 << 1)
#define GPIO_EXP_BANK_2_HDCP_ON			(0x01 << 2)
#define GPIO_EXP_BANK_2_N_TCODE			(0x01 << 3)
#define GPIO_EXP_BANK_2_SPR_LED1		(0x01 << 4)
#define GPIO_EXP_BANK_2_SPR_LED2		(0x01 << 5)
#define GPIO_EXP_BANK_2_SPR_LED3		(0x01 << 6)
#define GPIO_EXP_BANK_2_SPR_LED4		(0x01 << 7)

#define GPIO_EXP_BANK_3_OUTPUT_DEFAULT		0x2F

#define GPIO_EXP_BANK_3_MHL_TX_RST_B		(0x01 << 0)
#define GPIO_EXP_BANK_3_FW_WAKE_A		(0x01 << 1)
#define GPIO_EXP_BANK_3_CHG_DET			(0x01 << 2)
#define GPIO_EXP_BANK_3_USB_SW_CTRL		(0x01 << 3)
#define GPIO_EXP_BANK_3_12V_PS_SENSE		(0x01 << 4)
#define GPIO_EXP_BANK_3_EEPROM_WR_EN		(0x01 << 5)
#define GPIO_EXP_BANK_3_TX2MHLRX_PWR_A		(0x01 << 6)
#define GPIO_EXP_BANK_3_M2U_VBUS_CTRL_A		(0x01 << 7)

#define GPIO_EXP_BANK_4_OUTPUT_DEFAULT		0xF0
#define GPIO_EXP_BANK_4_DSW9			(0x01 << 0)
#define GPIO_EXP_BANK_4_DSW10			(0x01 << 1)
#define GPIO_EXP_BANK_4_DSW11			(0x01 << 2)
#define GPIO_EXP_BANK_4_DSW12			(0x01 << 3)
#define GPIO_EXP_BANK_4_USB_SW_CTRL0	(0x01 << 4)
#define GPIO_EXP_BANK_4_USB_SW_CTRL1	(0x01 << 5)
#define GPIO_EXP_BANK_4_TP15			(0x01 << 6)
#define GPIO_EXP_BANK_4_TP16			(0x01 << 7)

#define GET_FROM_MODULE_PARAM			-1
#define GPIO_ON_EXPANDER			-2

#define REG_PCA_950x_PORT_0_INPUT		GPIO_EXP_ADDR,0x00
#define REG_PCA_950x_PORT_1_INPUT		GPIO_EXP_ADDR,0x01
#define REG_PCA_950x_PORT_2_INPUT		GPIO_EXP_ADDR,0x02
#define REG_PCA_950x_PORT_3_INPUT		GPIO_EXP_ADDR,0x03
#define REG_PCA_950x_PORT_4_INPUT		GPIO_EXP_ADDR,0x04

#define REG_PCA_950x_PORT_0_OUTPUT		GPIO_EXP_ADDR,0x08
#define REG_PCA_950x_PORT_1_OUTPUT		GPIO_EXP_ADDR,0x09
#define REG_PCA_950x_PORT_2_OUTPUT		GPIO_EXP_ADDR,0x0A
#define REG_PCA_950x_PORT_3_OUTPUT		GPIO_EXP_ADDR,0x0B
#define REG_PCA_950x_PORT_4_OUTPUT		GPIO_EXP_ADDR,0x0C

u8		gpio_exp_bank2_output;
u8		gpio_exp_bank3_output;
u8		gpio_exp_bank4_output;

static char *buildTime = "Built " __DATE__"-" __TIME__;
static char *buildVersion = "1.00."BUILD_NUM_STRING;

struct semaphore	platform_lock;
static uint32_t		platform_flags;
bool				probe_fail = false;

static struct spi_device *spi_dev = NULL;
#define SPI_BUS_NUM			1
#define	SPI_CHIP_SEL		0
#define SPI_TRANSFER_MODE	SPI_MODE_0
#define SPI_BUS_SPEED		1000000

enum si_spi_opcodes {
	spi_op_disable			= 0x04,
	spi_op_enable			= 0x06,
	spi_op_reg_read			= 0x60,
	spi_op_reg_write		= 0x61,
	spi_op_emsc_read		= 0x80,
	spi_op_emsc_write		= 0x81,
	spi_op_slow_cbus_read	= 0x90,
	spi_op_slow_cbus_write	= 0x91
};

#define MAX_SPI_PAYLOAD_SIZE	256
#define MAX_SPI_CMD_SIZE		3
#define EMSC_WRITE_SPI_CMD_SIZE 1
#define EMSC_READ_SPI_CMD_SIZE 1
#define MAX_SPI_DUMMY_XFER_BYTES	20
#define MAX_SPI_XFER_BUFFER_SIZE	MAX_SPI_CMD_SIZE \
									+ MAX_SPI_DUMMY_XFER_BYTES \
									+ MAX_SPI_PAYLOAD_SIZE
#define MAX_SPI_EMSC_BLOCK_SIZE (MAX_SPI_CMD_SIZE + MAX_SPI_PAYLOAD_SIZE)



#define MAX_I2C_PAYLOAD_SIZE	256
#define MAX_I2C_CMD_SIZE		0

#define MAX_I2C_EMSC_BLOCK_SIZE (MAX_I2C_CMD_SIZE + MAX_I2C_PAYLOAD_SIZE)


struct spi_xfer_mem {
	u8	*tx_buf;
	u8	*rx_buf;
	/* block commands are asynchronous to normal cbus traffic
		and CANNOT share a buffer.
	*/
	uint8_t *block_tx_buffers;
	struct spi_transfer	spi_xfer[2];
	struct spi_message spi_cmd;
} spi_mem;

struct i2c_xfer_mem{
	uint8_t *block_tx_buffers;
}i2c_mem;

static int gpio_expander_transfer(u8 offset, u16 count,
						   u8 *values, bool write);

static struct i2c_adapter	*i2c_bus_adapter = NULL;

struct i2c_dev_info {
	uint8_t				dev_addr;
	struct i2c_client	*client;
};

#define I2C_DEV_INFO(addr) \
	{.dev_addr = addr >> 1, .client = NULL}

static struct i2c_dev_info device_addresses[] = {
	I2C_DEV_INFO(TX_PAGE_0),
	I2C_DEV_INFO(TX_PAGE_1),
	I2C_DEV_INFO(TX_PAGE_2),
	I2C_DEV_INFO(TX_PAGE_3),
	I2C_DEV_INFO(TX_PAGE_6),
	I2C_DEV_INFO(TX_PAGE_5),
	//I2C_DEV_INFO(TX_PAGE_DDC_SEGM),
	//I2C_DEV_INFO(TX_PAGE_DDC_EDID),
	I2C_DEV_INFO(GPIO_EXP_ADDR),
};

int		debug_level		= 3;
int		reg_debug_level	= 0;
bool	debug_reg_dump	= 0;
bool	input_dev_rap	= 1;
bool	input_dev_rcp	= 0;	// TODO: Keep default enabled when Feature complete
bool	input_dev_ucp	= 0;	// TODO: Keep default enabled when Feature complete
int		gpio_index		= 138;
int		vic_override	= -1;
int		hdcp_content_type = 0;
bool	i2c_data_only	= 1;
bool	use_spi			= 0;
int		crystal_khz		= 24000;//19200;	// Titan SK has 19.2MHz crystal so the default
bool	bpp_on_wb		= 0;
bool	simulate_scdt	= 1;
bool	use_heartbeat	= 0;	// Changed default to disable disconnection on hearbeat failure.
bool	wait_for_user_intr= 0;
int		tmds_link_speed	= 0;
#ifdef	FORCE_OCBUS_FOR_ECTS
bool	force_ocbus_for_ects = 0; // only for compile time flag FORCE_OCBUS_FOR_ECTS
#endif // FORCE_OCBUS_FOR_ECTS

module_param(debug_reg_dump		,bool	,S_IRUGO);
module_param(debug_level		,int	,S_IRUGO);
module_param(reg_debug_level	,int	,S_IRUGO);
module_param(input_dev_rap		,bool	,S_IRUGO);
module_param(input_dev_rcp		,bool	,S_IRUGO);
module_param(input_dev_ucp		,bool	,S_IRUGO);
module_param(vic_override		,int	,S_IRUGO);
module_param(hdcp_content_type 	,int	,S_IRUGO);
module_param(i2c_data_only		,bool	,S_IRUGO);
module_param(use_spi			,bool	,S_IRUGO);
module_param(crystal_khz		,int	,S_IRUGO);
module_param(bpp_on_wb			,bool	,S_IRUGO);
module_param(simulate_scdt		,bool	,S_IRUGO);
module_param(use_heartbeat		,bool	,S_IRUGO);
module_param(wait_for_user_intr,bool	,S_IRUGO);
module_param(tmds_link_speed	,int	,S_IRUGO);
#ifdef	FORCE_OCBUS_FOR_ECTS
module_param(force_ocbus_for_ects,bool	,S_IRUGO);
#endif // FORCE_OCBUS_FOR_ECTS

module_param_named(debug_msgs,debug_level,      int, S_IRUGO);

struct platform_signals_list platform_signals[] = {
	{	.name			= "TX_HW_RESET",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_MHL_TX_RST_B,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "TX_FW_WAKE",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_FW_WAKE_A,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "CHG_DET",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_CHG_DET,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "USB_SW_CTRL",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_USB_SW_CTRL,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "TWELVE_VOLT_PS_SENSE",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_12V_PS_SENSE,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "EEPROM_WR_EN",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_EEPROM_WR_EN,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "TX2MHLRX_PWR",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_TX2MHLRX_PWR_A,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "M2U_VBUS_CTRL",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_M2U_VBUS_CTRL_A,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param				= NULL
	},
	{	.name			= "LED_3D",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_3D,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "LED_PACKED_PIXEL",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_PKD_PXL,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "LED_HDCP",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_HDCP_ON,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "LED_SPARE_1",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_SPR_LED1,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "LED_SPARE_2",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_SPR_LED2,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "LED_SPARE_3",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_SPR_LED3,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "LED_SPARE_4",
		.gpio_number		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_2_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_2_SPR_LED4,
		.gpio_bank_value	= &gpio_exp_bank2_output,
		.param			= NULL
	},
	{	.name			= "X01_USB_SW_CTRL",
		.gpio_number 		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_3_OUTPUT},
		.gpio_mask_PCA950x	= GPIO_EXP_BANK_3_USB_SW_CTRL,
		.gpio_bank_value	= &gpio_exp_bank3_output,
		.param			= NULL
	},
	{	.name			= "X02_USB_SW_CTRL",
		.gpio_number 		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_4_OUTPUT},
		.gpio_mask_PCA950x	= (GPIO_EXP_BANK_4_USB_SW_CTRL0 |
								GPIO_EXP_BANK_4_USB_SW_CTRL1),
		.gpio_bank_value	= &gpio_exp_bank4_output,
		.param			= NULL
	},
	{	.name			= "X02_USB_SW_CTRL0",
		.gpio_number 		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_4_OUTPUT},
		.gpio_mask_PCA950x	= (GPIO_EXP_BANK_4_USB_SW_CTRL0),
		.gpio_bank_value	= &gpio_exp_bank4_output,
		.param			= NULL
	},
	{	.name			= "X02_USB_SW_CTRL1",
		.gpio_number 		= GPIO_ON_EXPANDER,
		.gpio_reg_PCA950x	= {REG_PCA_950x_PORT_4_OUTPUT},
		.gpio_mask_PCA950x	= (GPIO_EXP_BANK_4_USB_SW_CTRL1),
		.gpio_bank_value	= &gpio_exp_bank4_output,
		.param			= NULL
	}
};

static inline int platform_read_i2c_block(struct i2c_adapter *i2c_bus
								, u8 page
								, u8 offset
								, u16 count
								, u8 *values
								)
{
    struct i2c_msg			msg[2];

	msg[0].flags = 0;
	msg[0].addr = page >> 1;
	msg[0].buf = &offset;
	msg[0].len = 1;

	msg[1].flags = I2C_M_RD;
	msg[1].addr = page >> 1;
	msg[1].buf = values;
	msg[1].len = count;

	return i2c_transfer(i2c_bus_adapter, msg, 2);
}

static inline int platform_write_i2c_block(struct i2c_adapter *i2c_bus
										, u8 page
										, u8 offset
										, u16 count
										, u8 *values
										)
{
    struct i2c_msg			msg;
    u8						*buffer;
	int						ret;

	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		printk("%s:%d buffer allocation failed\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	msg.flags = 0;
	msg.addr = page >> 1;
	msg.buf = buffer;
	msg.len = count + 1;

	ret = i2c_transfer(i2c_bus, &msg, 1);

	kfree(buffer);

	if (ret != 1) {
		printk("%s:%d I2c write failed 0x%02x:0x%02x\n"
				,__FUNCTION__,__LINE__, page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

static int platform_read_i2c_reg(struct i2c_adapter *bus_adapter_i2c,u8 page, u8 offset)
{
	int						ret;
	u8	byte_read;
	ret = platform_read_i2c_block(bus_adapter_i2c
								, page
								, offset
								, 1
								, &byte_read
								);
	MHL_TX_DBG_INFO("\tGI2C_R %2x:%2x = %2x\n", page, offset, ret);
	if (ret != 2) {
		printk("%s:%d I2c read failed, 0x%02x:0x%02x\n",__FUNCTION__,__LINE__, page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}
	return ret ? ret : byte_read;
}

static int platform_write_i2c_reg(struct i2c_adapter *bus_adapter_i2c
								, u8 page
								, u8 offset
								, u8 value
								)
{
	MHL_TX_DBG_INFO("\tGI2C_W %2x:%2x <- %2x\n", page, offset, value);
	return platform_write_i2c_block(bus_adapter_i2c, page, offset, 1, &value);
}

uint32_t platform_get_flags(void)
{
	return platform_flags;
}

static void toggle_reset_n(void)
{
	MHL_TX_DBG_INFO("Toggle RESET_N pin\n");
	//gpio_set_value(GPIO_BB_RESET, 0);
	msleep(1);		// Without this, we see a 500ns reset pulse. Enforce 1ms.
	//gpio_set_value(GPIO_BB_RESET, 1);
}

static void toggle_mhl_tx_reset(int reset_period)
{
	MHL_TX_DBG_INFO("Toggle MHL_RST_B pin. Resets GPIO expander\n");
	gpio_exp_bank3_output &= ~GPIO_EXP_BANK_3_MHL_TX_RST_B;
	gpio_expander_transfer(GPIO_EXP_OUTPUT_REGS_OFFSET + 3,
						   1, &gpio_exp_bank3_output, true);

	msleep(reset_period);

	gpio_exp_bank3_output |= GPIO_EXP_BANK_3_MHL_TX_RST_B;
	gpio_expander_transfer(GPIO_EXP_OUTPUT_REGS_OFFSET + 3,
						   1, &gpio_exp_bank3_output, true);

}

/*
 * since we've agreed that the interrupt pin will never move
 *  we've special cased it for performance reasons.
 * This is why there is no set_pin() index for it.
 */
int is_interrupt_asserted(void)
{
	return (gpio_get_value(GPIO_MHL_INT) ? 0 : 1);
}

int get_config(void *dev_context, int config_idx)
{
	int	pin_state = 0;

	if (config_idx < ARRAY_SIZE(platform_signals)) {
		switch (platform_signals[config_idx].gpio_number) {
		case GET_FROM_MODULE_PARAM:
			pin_state =  *(platform_signals[config_idx].param);
			break;
		case GPIO_ON_EXPANDER:
			pin_state = (platform_read_i2c_reg(i2c_bus_adapter
						,platform_signals[config_idx].gpio_reg_PCA950x.slave_addr
						,platform_signals[config_idx].gpio_reg_PCA950x.offset)
					& platform_signals[config_idx].gpio_mask_PCA950x
					)?1:0;
			break;
		default:
			pin_state =  gpio_get_value(platform_signals[config_idx].gpio_number);
			break;
		}
	}
	return pin_state;
}

void set_pin_impl(void *dev_context, int pin_idx, int value
				,const char *function_name,int line_num)
{
	uint8_t	bank_value;

	if (pin_idx < ARRAY_SIZE(platform_signals)) {

		MHL_TX_DBG_INFO("set_pin(%s,%d)\n",
						platform_signals[pin_idx].name, value);
		switch (platform_signals[pin_idx].gpio_number) {
		case GET_FROM_MODULE_PARAM:
			break;
		case GPIO_ON_EXPANDER:
			bank_value = *(platform_signals[pin_idx].gpio_bank_value);
			if (value)
				bank_value |= platform_signals[pin_idx].gpio_mask_PCA950x;
			else
				bank_value &= ~platform_signals[pin_idx].gpio_mask_PCA950x;

			*(platform_signals[pin_idx].gpio_bank_value) = bank_value;
			platform_write_i2c_reg(i2c_bus_adapter,
					platform_signals[pin_idx].gpio_reg_PCA950x.slave_addr,
					platform_signals[pin_idx].gpio_reg_PCA950x.offset,
					bank_value);
			break;
		default:
			gpio_set_value(platform_signals[pin_idx].gpio_number, value);
			break;
		}
	}
}

void platform_mhl_tx_hw_reset(uint32_t reset_period,
							  uint32_t reset_delay)
{
	/* then reset the chip */
	toggle_reset_n();

	if (reset_delay)
		msleep(reset_delay);

	if (use_spi) {
		u8	cmd = spi_op_enable;
		spi_write(spi_dev, &cmd, 1);
	}
}

void mhl_tx_vbus_control(enum vbus_power_state power_state)
{
	struct device *parent_dev;
	struct mhl_dev_context *dev_context;

	if (use_spi)
		parent_dev = &spi_dev->dev;
	else
		parent_dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(parent_dev);

	switch (power_state) {
	case VBUS_OFF:
		//set_pin(dev_context,TX2MHLRX_PWR,1);
		break;

	case VBUS_ON:
		//set_pin(dev_context,TX2MHLRX_PWR,0);
		break;

	default:
		dev_err(dev_context->mhl_dev,
				"%s: Invalid power state %d received!\n",
				__func__, power_state);
		break;
	}
}

int si_device_dbg_i2c_reg_xfer(void *dev_context, u8 page, u8 offset,
									u16 count, bool rw_flag, u8 *buffer)
{
	if (rw_flag == DEBUG_I2C_WRITE)
		return mhl_tx_write_reg_block(dev_context, page, offset, count, buffer);
	else
		return mhl_tx_read_reg_block(dev_context, page, offset, count, buffer);
}


#define MAX_DEBUG_MSG_SIZE	1024

#if defined(DEBUG)

/*
 * Return a pointer to the file name part of the
 * passed path spec string.
 */
char *find_file_name(const char *path_spec)
{
	char *pc;

	for (pc = (char *)&path_spec[strlen(path_spec)]; pc != path_spec; --pc) {
		if ('\\' == *pc) {
			++pc;
			break;
		}
		if ('/' == *pc) {
			++pc;
			break;
		}
	}
	return pc;
}

void print_formatted_debug_msg(char *file_spec, const char *func_name,
							   int line_num,
							   char *fmt, ...)
{
	uint8_t		*msg = NULL;
	uint8_t		*msg_offset;
	char		*file_spec_sep = NULL;
	int			remaining_msg_len = MAX_DEBUG_MSG_SIZE;
	int			len;
	va_list		ap;

	if (fmt == NULL)
		return;

	msg = kmalloc(remaining_msg_len, GFP_KERNEL);
	if(msg == NULL)
		return;

	msg_offset = msg;

	len = scnprintf(msg_offset, remaining_msg_len, "mhl: ");
	msg_offset += len;
	remaining_msg_len -= len;

	/* Only print the file name, not the path */
	if (file_spec != NULL) {
		file_spec = find_file_name(file_spec);
	}

	if (file_spec != NULL) {
		if (func_name != NULL)
			file_spec_sep = "->";
		else if (line_num != -1)
			file_spec_sep = ":";
	}

	if (file_spec) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", file_spec);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (file_spec_sep) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", file_spec_sep);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (func_name) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", func_name);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (line_num != -1) {
		if ((file_spec != NULL) || (func_name != NULL))
			len = scnprintf(msg_offset, remaining_msg_len, ":%d ", line_num);
		else
			len = scnprintf(msg_offset, remaining_msg_len, "%d ", line_num);

		msg_offset += len;
		remaining_msg_len -= len;
	}

	va_start(ap, fmt);
	len = vscnprintf(msg_offset, remaining_msg_len, fmt, ap);
	va_end(ap);

	printk(KERN_ERR "%s",msg);

	kfree(msg);
}

void dump_transfer(enum tx_interface_types if_type,
					u8 page, u8 offset, u16 count, u8 *values, bool write)
{
	if (debug_reg_dump != 0)
	{
		int		buf_size = 64;
		u16		idx;
		int		buf_offset;
		char	*buf;
		char	*if_type_msg;

		switch (if_type) {
		case TX_INTERFACE_TYPE_I2C:
			if_type_msg = "I2C";
			break;
		case TX_INTERFACE_TYPE_SPI:
			if_type_msg = "SPI";
			break;
		default:
			return;
		};

		if (count > 1) {
			buf_size += count * 3;				/* 3 chars per byte displayed */
			buf_size += ((count / 16) + 1) * 8;	/* plus per display row overhead */
		}

		buf = kmalloc(buf_size, GFP_KERNEL);
		if (!buf)
			return;

		if (count == 1) {

			scnprintf(buf, buf_size, "   %s %02X.%02X %s %02X\n",
						if_type_msg,
						page, offset,
						write ? "W" : "R",
						values[0]);
		} else {
			idx = 0;
			buf_offset = scnprintf(buf, buf_size, "%s %02X.%02X %s(%d)",
								   if_type_msg, page, offset,
								   write ? "W" : "R", count);

			for (idx = 0; idx < count; idx++) {
				if (0 == (idx & 0x0F))
					buf_offset += scnprintf(&buf[buf_offset],
											buf_size - buf_offset,
											"\n%04X: ", idx);

				buf_offset += scnprintf(&buf[buf_offset],
											buf_size - buf_offset,
											"%02X ", values[idx]);
			}
			buf_offset += scnprintf(&buf[buf_offset], buf_size - buf_offset, "\n");
		}

		print_formatted_debug_msg(NULL, NULL, -1, buf);
		kfree(buf);
	}
}
#endif /* #if defined(DEBUG) */


static struct mhl_drv_info drv_info = {
	.drv_context_size = sizeof(struct drv_hw_context),
	.mhl_device_initialize = si_mhl_tx_chip_initialize,
	.mhl_device_isr = si_mhl_tx_drv_device_isr,
	.mhl_device_dbg_i2c_reg_xfer = si_device_dbg_i2c_reg_xfer,
	.mhl_device_get_aksv = si_mhl_tx_drv_get_aksv
};


static struct gpio starter_kit_control_gpios[] =
{
	/*
	 * GPIO signals needed for the starter kit board.
	 */
	{GPIO_MHL_INT,		GPIOF_IN,				"MHL_intr"},
	{GPIO_BB_RESET,		GPIOF_OUT_INIT_HIGH,	"MHL_reset"},
	{GPIO_BB_FW_WAKE,   GPIOF_OUT_INIT_HIGH,       "MHL_fwwake"},
	{GPIO_BB_POWER_1V0,   GPIOF_OUT_INIT_HIGH,       "MHL_power1v0"},
#ifdef I2C_CLK_TEST
	{GPIO_I2C_CLK,GPIOF_OUT_INIT_HIGH,"MHL_test_i2c_clk"},
#endif
	{GPIO_BB_ID_SEL,GPIOF_OUT_INIT_HIGH,"MHL_select"},
};

int mhl_tx_write_reg_block_i2c(void *drv_context, u8 page, u8 offset,
								u16 count, u8 *values)
{
	DUMP_I2C_TRANSFER(page, offset, count, values, true);

	return platform_write_i2c_block(i2c_bus_adapter,page, offset, count,values);
}


int mhl_tx_write_reg_i2c(void *drv_context, u8 page, u8 offset, u8 value)
{
	return mhl_tx_write_reg_block_i2c(drv_context, page, offset, 1, &value);
}


int mhl_tx_read_reg_block_i2c(void *drv_context, u8 page, u8 offset,
							  u16 count, u8 *values)
{
	int						ret;

	if ( count == 0 ) {
		MHL_TX_DBG_ERR("Tried to read 0 bytes\n");
		return(-EINVAL);
	}

	ret = platform_read_i2c_block(i2c_bus_adapter
								, page
								, offset
								, count
								, values
								);
	if (ret != 2) {
		MHL_TX_DBG_ERR(drv_context, "I2c read failed, 0x%02x:0x%02x\n", page, offset);
		ret = -EIO;
	} else {
		ret = 0;
		DUMP_I2C_TRANSFER(page, offset, count, values, false);
	}

	return ret;
}

int mhl_tx_read_reg_i2c(void *drv_context, u8 page, u8 offset)
{
	u8		byte_read;
	int		status;

	status = mhl_tx_read_reg_block_i2c(drv_context, page, offset,
										1, &byte_read);

	return status ? status : byte_read;
}

static int i2c_addr_to_spi_cmd(void *drv_context, bool write, u8 *page,
								u8 *opcode, u8 *dummy_bytes)
{
	if (write) {
		*opcode = spi_op_reg_write;
		*dummy_bytes = 0;
	} else {
		*opcode = spi_op_reg_read;
		*dummy_bytes = 5;
	}

	switch (*page) {
	case TX_PAGE_0:
		*page = 0;
		break;
	case TX_PAGE_1:
		*page = 1;
		break;
	case TX_PAGE_2:
		*page = 2;
		break;
	case TX_PAGE_3:
		*page = 3;
		break;
	case TX_PAGE_4:
		*page = 4;
		break;
	case TX_CBUS:
		*page = 5;
		break;
	case TX_PAGE_6:
		*page = 6;
		break;
	case TX_PAGE_7:
		*page = 7;
		break;
	case TX_PAGE_8:
		*page = 8;
		break;
	default:
		MHL_TX_DBG_ERR(drv_context, "Called with unknown page 0x%02x\n",
						*page);
		return -EINVAL;
	}
	return 0;
}

static int mhl_tx_write_reg_block_spi(void *drv_context, u8 page, u8 offset,
									  u16 count, u8 *values)
{
	u8		opcode;
	u8		dummy_bytes;
	u16		length = count + 3;
	int		ret;

	DUMP_SPI_TRANSFER(page, offset, count, values, true);

	ret = i2c_addr_to_spi_cmd(drv_context, true, &page, &opcode,
								&dummy_bytes);
	if (ret != 0)
		return ret;

	length = 3 + count + dummy_bytes;

	if (length > MAX_SPI_XFER_BUFFER_SIZE) {
		MHL_TX_DBG_ERR(drv_context, "Transfer count (%d) is too large!\n",
				count);
		return -EINVAL;
	}

	spi_mem.tx_buf[0] = opcode;
	spi_mem.tx_buf[1] = page;
	spi_mem.tx_buf[2] = offset;
	if (dummy_bytes)
		memset(&spi_mem.tx_buf[3], 0, dummy_bytes);

	memmove(&spi_mem.tx_buf[dummy_bytes + 3], values, count);

	ret = spi_write(spi_dev, spi_mem.tx_buf, length);

	if (ret != 0) {
		MHL_TX_DBG_ERR(drv_context, "SPI write block failed, " \
				"page: 0x%02x, register: 0x%02x\n",
				page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}


static int mhl_tx_write_reg_spi(void *drv_context, u8 page, u8 offset,
								u8 value)
{
	return mhl_tx_write_reg_block_spi(drv_context, page, offset, 1, &value);
}


static int mhl_tx_read_reg_block_spi(void *drv_context, u8 page, u8 offset,
									u16 count, u8 *values)
{
	u8		page_num = page;
	u8		opcode;
	u8		dummy_bytes;
	u16		length;
	int		ret;

	if (count > MAX_SPI_PAYLOAD_SIZE) {
		MHL_TX_DBG_ERR(drv_context, "Requested transfer size is too large\n");
		return -EINVAL;
	}

	ret = i2c_addr_to_spi_cmd(drv_context, false, &page_num, &opcode,
								&dummy_bytes);
	if (ret != 0)
		return ret;

	length = 3 + count + dummy_bytes;
	if (length > MAX_SPI_XFER_BUFFER_SIZE) {
		MHL_TX_DBG_ERR(drv_context, "Requested transfer size is too large\n");
		return -EINVAL;
	}

	spi_message_init(&spi_mem.spi_cmd);
	memset(&spi_mem.spi_xfer, 0, sizeof(spi_mem.spi_xfer));
	spi_mem.tx_buf[0] = opcode;
	spi_mem.tx_buf[1] = page_num;
	spi_mem.tx_buf[2] = offset;

	spi_mem.spi_xfer[0].tx_buf = spi_mem.tx_buf;
	spi_mem.spi_xfer[0].len = 3 + dummy_bytes;
	spi_message_add_tail(&spi_mem.spi_xfer[0], &spi_mem.spi_cmd);

	spi_mem.spi_xfer[1].rx_buf = spi_mem.rx_buf;
	spi_mem.spi_xfer[1].len = count;
	spi_mem.spi_xfer[1].cs_change = 1;
	spi_message_add_tail(&spi_mem.spi_xfer[1], &spi_mem.spi_cmd);

	ret = spi_sync(spi_dev, &spi_mem.spi_cmd);

	if (ret != 0) {
		MHL_TX_DBG_ERR(drv_context, "SPI read block failed, " \
				"page: 0x%02x, register: 0x%02x\n",
				page, offset);
	} else {
		memcpy(values, spi_mem.rx_buf, count);
		DUMP_SPI_TRANSFER(page, offset, count, values, false);
	}

	return ret;
}

int mhl_tx_read_reg_block_spi_emsc(void *drv_context, u16 count, u8 *values)
{
	u8		dummy_bytes = 1;
	u16		length;
	int		ret;

	if (count > MAX_SPI_PAYLOAD_SIZE) {
		MHL_TX_DBG_ERR(drv_context, "Requested transfer size is too large\n");
		return -EINVAL;
	}

	length = EMSC_READ_SPI_CMD_SIZE + dummy_bytes + count;
	if (length > MAX_SPI_XFER_BUFFER_SIZE) {
		MHL_TX_DBG_ERR(drv_context, "Requested transfer size is too large\n");
		return -EINVAL;
	}

	spi_message_init(&spi_mem.spi_cmd);
	memset(&spi_mem.spi_xfer, 0, sizeof(spi_mem.spi_xfer));
	spi_mem.tx_buf[0] = spi_op_emsc_read;

	spi_mem.spi_xfer[0].tx_buf = spi_mem.tx_buf;
	spi_mem.spi_xfer[0].len = EMSC_READ_SPI_CMD_SIZE + dummy_bytes;
	spi_message_add_tail(&spi_mem.spi_xfer[0], &spi_mem.spi_cmd);

	spi_mem.spi_xfer[1].rx_buf = spi_mem.rx_buf;
	spi_mem.spi_xfer[1].len = count;
	spi_mem.spi_xfer[1].cs_change = 1;
	spi_message_add_tail(&spi_mem.spi_xfer[1], &spi_mem.spi_cmd);

	ret = spi_sync(spi_dev, &spi_mem.spi_cmd);

	if (ret != 0) {
		MHL_TX_DBG_ERR(drv_context, "SPI eMSC read block failed " );
	} else {
		memcpy(values, spi_mem.rx_buf, count);
		//DUMP_SPI_TRANSFER(page, offset, count, values, false);
	}

	return ret;
}
static int mhl_tx_read_reg_spi(void *drv_context, u8 page, u8 offset)
{
	u8		byte_read;
	int		status;

	status = mhl_tx_read_reg_block_spi(drv_context, page, offset, 1,
										&byte_read);

	return status ? status : byte_read;
}

int mhl_tx_write_reg_block(void *drv_context, u8 page, u8 offset,
							u16 count, u8 *values)
{
	if (use_spi)
		return mhl_tx_write_reg_block_spi(drv_context, page, offset,
										  count, values);
	else
		return mhl_tx_write_reg_block_i2c(drv_context, page, offset,
										  count, values);
}

void si_mhl_tx_platform_get_block_buffer_info(struct block_buffer_info_t  *block_buffer_info)
{
	if (use_spi){
		block_buffer_info->buffer = spi_mem.block_tx_buffers;
		block_buffer_info->req_size = MAX_SPI_EMSC_BLOCK_SIZE;
		block_buffer_info->payload_offset = EMSC_WRITE_SPI_CMD_SIZE;
	}else{
		block_buffer_info->buffer = i2c_mem.block_tx_buffers;
		block_buffer_info->req_size = MAX_I2C_EMSC_BLOCK_SIZE;
		block_buffer_info->payload_offset = MAX_I2C_CMD_SIZE;
	}
}
int mhl_tx_write_emsc_block_spi(void *drv_context,struct block_req *req)
{
	u16		length;
	int		ret;

	//DUMP_SPI_TRANSFER(page, offset, req->count, req->payload->as_bytes, true);


	//dummy bytes will always be zero
	length = EMSC_WRITE_SPI_CMD_SIZE + req->count;

	if (length > MAX_SPI_EMSC_BLOCK_SIZE) {
		MHL_TX_DBG_ERR(drv_context, "Transfer count (%d) is too large!\n",
				req->count);
		return -EINVAL;
	}

	req->platform_header[0] = spi_op_emsc_write;

	ret = spi_write(spi_dev, req->platform_header, length);

	if (ret != 0) {
		MHL_TX_DBG_ERR(drv_context, "SPI write block failed\n");
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

int mhl_tx_write_reg(void *drv_context, u8 page, u8 offset, u8 value)
{
	if (use_spi)
		return mhl_tx_write_reg_spi(drv_context, page, offset, value);
	else
		return mhl_tx_write_reg_i2c(drv_context, page, offset, value);
}

int mhl_tx_read_reg_block(void *drv_context, u8 page, u8 offset, u16 count,
						  u8 *values)
{
	if (use_spi)
		return mhl_tx_read_reg_block_spi(drv_context, page, offset, count,
										values);
	else
		return mhl_tx_read_reg_block_i2c(drv_context, page, offset, count,
										values);
}

int mhl_tx_read_reg(void *drv_context, u8 page, u8 offset)
{
	if (use_spi)
		return mhl_tx_read_reg_spi(drv_context, page, offset);
	else
		return mhl_tx_read_reg_i2c(drv_context, page, offset);
}

int mhl_tx_modify_reg(void *drv_context, u8 page, u8 offset,
					  u8 mask, u8 value)
{
	int	reg_value;
	int	write_status;

	reg_value = mhl_tx_read_reg(drv_context, page, offset);
	if (reg_value < 0)
		return reg_value;

	reg_value &= ~mask;
	reg_value |= mask & value;

	write_status = mhl_tx_write_reg(drv_context, page, offset, reg_value);

	if (write_status < 0)
		return write_status;
	else
		return reg_value;
}

/*
 * Return a value indicating how upstream HPD is
 * implemented on this platform.
 */
hpd_control_mode platform_get_hpd_control_mode(void)
{
	// open drain not as reliable... return HPD_CTRL_OPEN_DRAIN;
	return HPD_CTRL_PUSH_PULL;
}

static int gpio_expander_transfer(u8 offset, u16 count,
						   u8 *values, bool write)
{
    struct i2c_msg			msg[2];
    u8						buf[8];
    int						msg_count;
	int						ret;

	if ((count + 1) > ARRAY_SIZE(buf))
		return -1;

	if (write) {
		buf[0] = offset;
		memmove(&buf[1], values, count);

		msg[0].flags = 0;
		msg[0].addr = GPIO_EXP_ADDR >> 1;
		msg[0].buf = buf;
		msg[0].len = count + 1;

		msg_count = 1;

	} else {

		msg[0].flags = 0;
		msg[0].addr = GPIO_EXP_ADDR >> 1;
		msg[0].buf = &offset;
		msg[0].len = 1;

		msg[1].flags = I2C_M_RD;
		msg[1].addr = GPIO_EXP_ADDR >> 1;
		msg[1].buf = values;
		msg[1].len = count;
		msg_count = 2;
	}

	ret = i2c_transfer(i2c_bus_adapter, msg, msg_count);
	if (ret != msg_count) {
		MHL_TX_DBG_ERR("I2c %s failed ret:%d, page: 0x%02x, "
					   "register: 0x%02x count:0x%x\n"
						,write?"write":"read"
						,ret, GPIO_EXP_ADDR, offset,count);
		ret = -EIO;
	} else {
		ret = 0;
//		DUMP_I2C_TRANSFER(GPIO_EXP_ADDR, offset, count, values, write);
	}

	return ret;
}

static int gpio_expander_init(void)
{
	u8	gpio_exp_mask_init[] = {GPIO_EXP_INTR_MASK_REGS_OFFSET,
    							0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	u8	gpio_exp_pol_invert_init[] = {GPIO_EXP_POL_INVERT_REGS_OFFSET,
									  0x00, 0x00, 0x00, 0x00, 0x00};
	u8	gpio_exp_output_init[] = {GPIO_EXP_OUTPUT_REGS_OFFSET, 0x00, 0x00,
								  GPIO_EXP_BANK_2_OUTPUT_DEFAULT,
								  GPIO_EXP_BANK_3_OUTPUT_DEFAULT,
								  GPIO_EXP_BANK_4_OUTPUT_DEFAULT};
	u8	gpio_exp_io_config_init[] = {GPIO_EXP_IO_CONFIG_REGS_OFFSET,
									 0xFF, 0xFF, 0x00, 0x10, 0x0F};
	int	ret;

	/* First reset GPIO Expander */
	toggle_mhl_tx_reset(10);

	ret = gpio_expander_transfer(gpio_exp_mask_init[0],
								 ARRAY_SIZE(gpio_exp_mask_init) - 1,
								 &gpio_exp_mask_init[1], true);
	if (ret != 0){
	    pr_err("%s():%d gpio_expander_transfer failed, error code %d\n",
	    		__func__,__LINE__, ret);
		return ret;
	}

	ret = gpio_expander_transfer(gpio_exp_pol_invert_init[0],
								 ARRAY_SIZE(gpio_exp_pol_invert_init) - 1,
								 &gpio_exp_pol_invert_init[1], true);
	if (ret != 0){
	    pr_err("%s():%d gpio_expander_transfer failed, error code %d\n",
	    		__func__,__LINE__, ret);
		return ret;
	}
	ret = gpio_expander_transfer(gpio_exp_output_init[0],
								 ARRAY_SIZE(gpio_exp_output_init) - 1,
								 &gpio_exp_output_init[1], true);
	gpio_exp_bank2_output = GPIO_EXP_BANK_2_OUTPUT_DEFAULT;
	gpio_exp_bank3_output = GPIO_EXP_BANK_3_OUTPUT_DEFAULT;
	gpio_exp_bank4_output = GPIO_EXP_BANK_4_OUTPUT_DEFAULT;

	if (ret != 0){
	    pr_err("%s():%d gpio_expander_transfer failed, error code %d\n",
	    		__func__,__LINE__, ret);
		return ret;
	}
	ret = gpio_expander_transfer(gpio_exp_io_config_init[0],
								 ARRAY_SIZE(gpio_exp_io_config_init) - 1,
								 &gpio_exp_io_config_init[1], true);
	if (ret != 0){
	    pr_err("%s():%d gpio_expander_transfer failed, error code %d\n",
	    		__func__,__LINE__, ret);
	}
	return ret;
}

static int __devinit starter_kit_init(void)
{
	int	ret;

	/*
	 * Configure as outputs the GPIOs used to control the direction
	 * of the GPIO pin level shifters on the tincantools daughterboard
	 */
	ret = gpio_request_array(starter_kit_control_gpios,
							ARRAY_SIZE(starter_kit_control_gpios));
	if(ret < 0) {
		pr_err("%s(): gpio_request_array failed, error code %d\n",
				__func__, ret);
	}else{
		ret = gpio_expander_init();
		if (ret == 0){

		} else {
			gpio_free_array(starter_kit_control_gpios,
							ARRAY_SIZE(starter_kit_control_gpios));
		}
	}
	return ret;
}

//+++ztemt
#ifdef CONFIG_OF
static int sii_init_gpio(struct device *dev)
{
	int gpio_reset;
	int gpio_int;
	int ret = 0;

	struct device_node *of_node = NULL;
	of_node = dev->of_node;
	if (!of_node) {
		pr_err("[sii8620]%s: invalid of_node\n", __func__);
		return -EINVAL;
	}

	gpio_reset = of_get_named_gpio(of_node, "sii8620-reset-gpio", 0);
	pr_err("[sii8620]%s gpio_reset=%d\n", __func__, gpio_reset);
	if (gpio_reset < 0) {
		pr_err("[sii8620]%s: Can't get sii-reset-gpio\n", __func__);
		return -EINVAL;
	}	
	gpio_int = of_get_named_gpio(of_node, "sii8620-int-gpio", 0);
	pr_err("[sii8620]%s gpio_int=%d\n", __func__, gpio_int);
	if (gpio_int < 0) {
		pr_err("[sii8620]%s: Can't get sii8620-int-gpio\n", __func__);
		return -EINVAL;
	}
	
	
	ret = gpio_request(gpio_reset, "sii8620_reset_n");
	if (ret) {
		pr_err("[sii8620]%s : failed to request gpio %d\n", __func__,
				gpio_reset);
		goto err1;
	}
	gpio_direction_output(gpio_reset, 0);
	printk("[sii8620]==========%s:line%d,gpio_reset=%d.\n",__func__,__LINE__, gpio_reset);
	ret = gpio_request(gpio_int, "sii8620_int_n");
	if (ret) {
		pr_err("[sii8620]%s : failed to request gpio %d\n", __func__,
				gpio_int);
		goto err2;
	}
	gpio_direction_input(gpio_int);
	gpio_set_value(gpio_reset, 0);
err1:
	gpio_free(gpio_reset);
err2:
	gpio_free(gpio_int);
	return ret;
}
#endif
//----ztemt

static int __devinit si_8620_mhl_tx_i2c_probe(struct i2c_client *client,
					      const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;
	/**********************************************/
	/*struct mhl_tx_ctrl *mhl_ctrl;
	//struct usb_ext_notification *mhl_info = NULL;
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops = NULL;

	mhl_ctrl = devm_kzalloc(&client->dev, sizeof(*mhl_ctrl), GFP_KERNEL);
	if (!mhl_ctrl) {
		pr_err("%s: FAILED: cannot alloc hdmi tx ctrl\n", __func__);
		rc = -ENOMEM;
		goto failed_no_mem;
	}*/
	/**********************************************/
	if (!adapter || !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_info("[ERROR] %s() i2c no function\n", __func__);
		ret =  -EIO;
		goto done;
	}
	
	device_addresses[0].client = client;
	
	i2c_mem.block_tx_buffers =kmalloc(MAX_I2C_EMSC_BLOCK_SIZE * si_mhl_tx_get_num_block_reqs(),GFP_KERNEL);
	if (NULL == i2c_mem.block_tx_buffers) {
		ret = -ENOMEM;
		goto done;
	}

	printk("[sii8620]%s: si_8620_mhl_tx_i2c_probe\n", __func__);
	if(0)
		ret = sii_init_gpio(&client->dev);

//mhl_sel_id
	if(!gpio_is_valid(GPIO_BB_ID_SEL)){
		printk("[sii8620]%s : gpio invalid :%d\n", __func__,
				GPIO_BB_ID_SEL);
		return -EBUSY;
	}
	ret = gpio_request(GPIO_BB_ID_SEL, "sii8620_sel");
	if (ret) {
		printk("[sii8620]%s : failed to request gpio %d\n", __func__,
				GPIO_BB_ID_SEL);
		gpio_free(GPIO_BB_ID_SEL);
		goto done;
	}
	//msleep(200);//200ms
	ret = gpio_direction_output(GPIO_BB_ID_SEL,0);
	if(ret < 0){
		printk("[sii8620]%s : gpio output 1 error :%d\n", __func__,
				GPIO_BB_ID_SEL);
		return -EBUSY;
	}	
	
#ifdef I2C_CLK_TEST
	//i2c clk test
	if(!gpio_is_valid(GPIO_I2C_CLK)){
		printk("[sii8620]%s : gpio invalid :%d\n", __func__,
				GPIO_I2C_CLK);
		return -EBUSY;
	}
	ret = gpio_request(GPIO_I2C_CLK, "sii8620_i2c_clk");
	if (ret) {
		printk("[sii8620]%s : failed to request gpio %d\n", __func__,
				GPIO_I2C_CLK);
		gpio_free(GPIO_I2C_CLK);
		goto done;
	}
#endif
	//power 1v0
	if(!gpio_is_valid(GPIO_BB_POWER_1V0)){
		printk("[sii8620]%s : gpio invalid :%d\n", __func__,
				GPIO_BB_POWER_1V0);
		return -EBUSY;
	}
	ret = gpio_request(GPIO_BB_POWER_1V0, "sii8620_power_1v0");
	if (ret) {
		printk("[sii8620]%s : failed to request gpio %d\n", __func__,
				GPIO_BB_POWER_1V0);
		gpio_free(GPIO_BB_POWER_1V0);
		goto done;
	}
	//msleep(200);//200ms
	ret = gpio_direction_output(GPIO_BB_POWER_1V0,1);
	if(ret < 0){
		printk("[sii8620]%s : gpio output 1 error :%d\n", __func__,
				GPIO_BB_POWER_1V0);
		return -EBUSY;
	}	

	//fw_wake
	if(!gpio_is_valid(GPIO_BB_FW_WAKE)){
		printk("[sii8620]%s : gpio invalid :%d\n", __func__,
				GPIO_BB_FW_WAKE);
		return -EBUSY;
	}
	ret = gpio_request(GPIO_BB_FW_WAKE, "sii8620_fw_wake");
	if (ret) {
		printk("[sii8620]%s : failed to request gpio %d\n", __func__,
				GPIO_BB_FW_WAKE);
		gpio_free(GPIO_BB_FW_WAKE);
		goto done;
	}
	//msleep(200);//200ms
	ret = gpio_direction_output(GPIO_BB_FW_WAKE,1);
	if(ret < 0){
		printk("[sii8620]%s : gpio output 1 error :%d\n", __func__,
				GPIO_BB_FW_WAKE);
		return -EBUSY;
	}
	//INT
	if(!gpio_is_valid(GPIO_MHL_INT)){
		printk("[sii8620]%s : gpio invalid :%d\n", __func__,
				GPIO_MHL_INT);
		return -EBUSY;
	}
	ret = gpio_request(GPIO_MHL_INT, "sii8620_int_n");
	if (ret) {
		printk("[sii8620]%s : failed to request gpio %d\n", __func__,
				GPIO_MHL_INT);
		gpio_free(GPIO_BB_RESET);
		goto done;
	}
	ret = gpio_direction_input(GPIO_MHL_INT);
	if(ret < 0){
		printk("[sii8620]%s : gpio error :%d\n", __func__,
				GPIO_MHL_INT);
		return -EBUSY;
	}
	msleep(50);
	//RST
	if(!gpio_is_valid(GPIO_BB_RESET)){
		printk("[sii8620]%s : gpio invalid :%d\n", __func__,
				GPIO_BB_RESET);
		return -EBUSY;
	}
	ret = gpio_request(GPIO_BB_RESET, "sii8620_reset_n");
	if (ret) {
		printk("[sii8620]%s : failed to request gpio %d\n", __func__,
				GPIO_BB_RESET);
		gpio_free(GPIO_BB_RESET);
		goto done;
	}
	ret = gpio_direction_output(GPIO_BB_RESET,0);
	if(ret < 0){
		printk("[sii8620]%s : gpio output 0 error :%d\n", __func__,
				GPIO_BB_RESET);
		return -EBUSY;
	}
	msleep(200);//200ms
	ret = gpio_direction_output(GPIO_BB_RESET,1);
	if(ret < 0){
		printk("[sii8620]%s : gpio output 1 error :%d\n", __func__,
				GPIO_BB_RESET);
		return -EBUSY;
	}
	msleep(50);
	printk("[sii8620]: gpio_get_value int = %d\n",gpio_get_value(GPIO_MHL_INT));
	printk("[sii8620]: gpio_get_value reset = %d\n",gpio_get_value(GPIO_BB_RESET));
	//printk("[sii8620]%s: sii_init_gpio.ret = %d\n", __func__,ret);

#if 1
	//ret = starter_kit_init();
	if (ret >= 0) 
		{
		drv_info.irq = gpio_to_irq(GPIO_MHL_INT);
		printk("[sii8620]%s: sii_init_gpio.irq = %d\n", __func__,drv_info.irq);
		
		ret = mhl_tx_init(&drv_info, &client->dev);
		return ret;//add by gary
		if (ret){
		    pr_err("%s(): mhl_tx_init failed, error code %d\n",
		    		__func__, ret);
			if(0){
				starter_kit_init();
				gpio_free_array(starter_kit_control_gpios,
						ARRAY_SIZE(starter_kit_control_gpios));
				}
		}
		/******************************************************/
		/*printk("%s: i2c client addr is [%x]\n", __func__, client->addr);
	if (mhl_ctrl->pdata->hdmi_pdev) {
		ret= msm_hdmi_register_mhl(mhl_ctrl->pdata->hdmi_pdev,
					   hdmi_mhl_ops, mhl_ctrl);
		if (ret) {
			printk("%s: register with hdmi failed\n", __func__);
			ret = -EPROBE_DEFER;
			//goto failed_probe_pwr;
		}
	}

	if (!hdmi_mhl_ops || !hdmi_mhl_ops->tmds_enabled ||
	    !hdmi_mhl_ops->set_mhl_max_pclk) {
		printk("%s: func ptr is NULL\n", __func__);
		ret= -EINVAL;
		//goto failed_probe_pwr;
	}
	mhl_ctrl->hdmi_mhl_ops = hdmi_mhl_ops;

	ret = hdmi_mhl_ops->set_mhl_max_pclk(
		mhl_ctrl->pdata->hdmi_pdev, MAX_MHL_PCLK);
	if (ret) {
		printk("%s: can't set max mhl pclk\n", __func__);
		//goto failed_probe_pwr;
	}*/
/******************************************************/
	}
#endif

done:
	if (ret != 0)
		probe_fail = true;


	return ret;
}


static int __devexit si_8620_mhl_tx_remove(struct i2c_client *client)
{
	if(!use_spi){
		kfree(i2c_mem.block_tx_buffers);
	}
	gpio_free_array(starter_kit_control_gpios,
					ARRAY_SIZE(starter_kit_control_gpios));
	return 0;
}

int si_8620_pm_suspend(struct device *dev)
{
	int status;

	status = down_interruptible(&platform_lock);
	if (status)
		goto done;

	status = mhl_handle_power_change_request(dev, false);
	/*
	 * Set MHL/USB switch to USB
	 * NOTE: Switch control is implemented differently on each
	 * version of the starter kit.
	 */
	set_pin(dev,X01_USB_SW_CTRL,0);
	set_pin(dev,X02_USB_SW_CTRL,0);

	up(&platform_lock);
done:
	return status;
}

int si_8620_pm_resume(struct device *dev)
{
	int status;

	status = down_interruptible(&platform_lock);
	if (status)
		goto done;

	set_pin(dev,X01_USB_SW_CTRL,1);
	set_pin(dev,X02_USB_SW_CTRL,1);
	status = mhl_handle_power_change_request(dev, true);

	up(&platform_lock);
done:
	return status;
}

int si_8620_power_control(bool power_up)
{
	struct device	*dev = NULL;
	int				status;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	if (power_up)
		status = si_8620_pm_resume(dev);
	else
		status = si_8620_pm_suspend(dev);
	return status;
}
EXPORT_SYMBOL_GPL(si_8620_power_control);


static const struct i2c_device_id si_8620_mhl_tx_id[] = {
	{MHL_DEVICE_NAME, 0},//sii-8620
	{}
};
#ifdef CONFIG_OF
static struct of_device_id si_8620_mhl_tx_match_table[] = {
     {.compatible = MHL_DRIVER_NAME/*"sii,sii8620"*/,},//sii8620drv
     { },
 };
#endif
MODULE_DEVICE_TABLE(i2c, si_8620_mhl_tx_id);

static struct dev_pm_ops si_8620_tx_pm_ops = {
	.runtime_suspend = si_8620_pm_suspend,
	.runtime_resume = si_8620_pm_resume,
};

static struct i2c_driver si_8620_mhl_tx_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MHL_DRIVER_NAME,
		   .pm = &si_8620_tx_pm_ops,
		   .of_match_table = si_8620_mhl_tx_match_table,
		   },
	.id_table = si_8620_mhl_tx_id,
	.probe = si_8620_mhl_tx_i2c_probe,
	.remove = __devexit_p(si_8620_mhl_tx_remove),
	.command = NULL,
};
#if 0
static struct i2c_board_info __initdata si_8620_i2c_boardinfo[] = {
	{
	   	I2C_BOARD_INFO(MHL_DEVICE_NAME, (TX_PAGE_0 >> 1)),
     	.flags = I2C_CLIENT_WAKE,
		.irq = 344,//343,//gpio_to_irq(GPIO_MHL_INT),//OMAP_GPIO_IRQ(GPIO_MHL_INT),ztemt
	}
};
#endif
static int __devinit si_8620_mhl_tx_spi_probe(struct spi_device *spi)
{
	int ret;

	pr_info("%s(), spi = %p\n", __func__, spi);
	spi->bits_per_word = 8;
	spi_dev = spi;

	spi_mem.tx_buf = kmalloc(MAX_SPI_XFER_BUFFER_SIZE, GFP_KERNEL);
	spi_mem.rx_buf = kmalloc(MAX_SPI_XFER_BUFFER_SIZE, GFP_KERNEL);
	spi_mem.block_tx_buffers =kmalloc(MAX_SPI_EMSC_BLOCK_SIZE * si_mhl_tx_get_num_block_reqs(),GFP_KERNEL);
	if (!spi_mem.tx_buf || !spi_mem.rx_buf || !spi_mem.block_tx_buffers) {
		ret = -ENOMEM;
		goto failed;
	}
	ret = starter_kit_init();
	if (ret >= 0) {
		drv_info.irq = gpio_to_irq(GPIO_MHL_INT);
		ret = mhl_tx_init(&drv_info, &spi_dev->dev);
		if (ret) {
			pr_err("%s(): mhl_tx_init failed, error code %d\n",
					__func__, ret);
			gpio_free_array(starter_kit_control_gpios,
							ARRAY_SIZE(starter_kit_control_gpios));
			goto failed;
		}
		goto done;
	}

failed:
	if (spi_mem.tx_buf){
		kfree(spi_mem.tx_buf);
		spi_mem.tx_buf=NULL;
	}
	if (spi_mem.rx_buf){
		kfree(spi_mem.rx_buf);
		spi_mem.rx_buf=NULL;
	}
	if (spi_mem.block_tx_buffers){
		kfree(spi_mem.block_tx_buffers);
		spi_mem.block_tx_buffers=NULL;
	}
	probe_fail = true;

done:
	return ret;
}

static int __devexit si_8620_mhl_spi_remove(struct spi_device *spi_dev)
{
	pr_info("%s() called\n", __func__);

	gpio_free_array(starter_kit_control_gpios,
					ARRAY_SIZE(starter_kit_control_gpios));
	kfree(spi_mem.tx_buf);
	kfree(spi_mem.rx_buf);
	kfree(spi_mem.block_tx_buffers);
	return 0;
}

static struct spi_driver si_86x0_mhl_tx_spi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MHL_DRIVER_NAME,
		   .pm = &si_8620_tx_pm_ops,
		   },
	.probe = si_8620_mhl_tx_spi_probe,
	.remove = __devexit_p(si_8620_mhl_spi_remove),
};

static struct spi_board_info __initdata si_86x0_spi_board_info = {
		.modalias = MHL_DRIVER_NAME,
		.max_speed_hz = SPI_BUS_SPEED,
		.bus_num = SPI_BUS_NUM,
		.chip_select = SPI_CHIP_SEL,
		.mode = SPI_TRANSFER_MODE,
		.irq = -1
};

static int __init add_spi_device_to_bus(void)
{
	struct spi_master *spi_master;
	int status = 0;

	pr_info("add_spi_device_to_bus called\n");

	spi_master = spi_busnum_to_master(SPI_BUS_NUM);
	if (spi_master == NULL) {
		pr_err("spi_busnum_to_master(%d) returned NULL\n", SPI_BUS_NUM);
		return -1;
	}

	spi_dev = spi_new_device(spi_master, &si_86x0_spi_board_info);
	if (spi_dev == NULL || probe_fail) {
		pr_err("spi_new_device() failed\n");
		if (spi_dev)
			spi_unregister_device(spi_dev);
		status = -1;
		goto exit;
	}

exit:
	put_device(&spi_master->dev);
	return status;
}

static int __init spi_init(void)
{
	int	status;

	status = spi_register_driver(&si_86x0_mhl_tx_spi_driver);
	if (status < 0) {
		pr_err("[ERROR] %s():%d failed !\n", __func__, __LINE__);
		goto exit;
	}

	status = add_spi_device_to_bus();
	if (status < 0) {
		pr_err("Failed to add MHL transmitter as SPI device\n");
		spi_unregister_driver(&si_86x0_mhl_tx_spi_driver);
	}

exit:
if (probe_fail)
	status = -ENODEV;

	return status;
}


static int __init i2c_init(void)
{
	//struct i2c_client	*client;
	int					idx;
	int					ret = -EFAULT;

	printk("mhl i2c_init\n");
	/* "Hotplug" the MHL transmitter device onto the 2nd I2C bus */
	i2c_bus_adapter = i2c_get_adapter(1);
	if (i2c_bus_adapter == NULL) {
		pr_err ("%s() failed to get i2c adapter\n", __func__);
		MHL_TX_DBG_INFO("returning %d\n",ret);
		goto done;
	}

	for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
		if (idx == 0) {
			//client = i2c_new_device(i2c_bus_adapter, &si_8620_i2c_boardinfo[idx]);
			//device_addresses[idx].client = client;
		} else {
			device_addresses[idx].client = i2c_new_dummy(i2c_bus_adapter,
											device_addresses[idx].dev_addr);
		}
		#if 0
		if (device_addresses[idx].client == NULL){
			MHL_TX_DBG_INFO("returning %d\n",ret);
			goto err_exit;
	        }
		#endif
	}

	ret = i2c_add_driver(&si_8620_mhl_tx_i2c_driver);
	if (ret < 0 || probe_fail) {
		if (ret == 0)
			i2c_del_driver(&si_8620_mhl_tx_i2c_driver);
		pr_err("[ERROR] %s():%d failed !\n\nCHECK POWER AND CONNECTION TO CP8620 Starter Kit.\n\n", __func__, __LINE__);
		goto err_exit;
	}

	goto done;

err_exit:
	for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
		if (device_addresses[idx].client != NULL)
			i2c_unregister_device(device_addresses[idx].client);
	}

done:
	if (probe_fail)
		ret = -ENODEV;

	MHL_TX_DBG_INFO("returning %d\n",ret);
	return ret;
}

static int __init si_8620_init(void)
{
	int	ret;

	pr_info("mhl: Starting SiI%d Driver v%s\n", MHL_PRODUCT_NUM, buildVersion);
	pr_info("mhl: %s\n", buildTime);

	printk("mhl: Starting SiI%d Driver v%s\n", MHL_PRODUCT_NUM, buildVersion);
	printk("mhl: %s\n", buildTime);

	sema_init(&platform_lock, 1);

	if (use_heartbeat)
		platform_flags |= PLATFORM_FLAG_USE_HEARTBEAT;

	if (tmds_link_speed == 15)
		platform_flags |= PLATFORM_FLAG_1_5GBPS;
	else if (tmds_link_speed == 3)
		platform_flags |= PLATFORM_FLAG_3GBPS;
	else if (tmds_link_speed == 6)
		platform_flags |= PLATFORM_FLAG_6GBPS;

	/*
	 * NOTE: Even if the user selects to communicate with the MHL
	 * transmitter via SPI we still need I2C to communicate with
	 * other devices on the starter kit board.
	 */
	i2c_bus_adapter = i2c_get_adapter(1);//ztemt 4
	if (i2c_bus_adapter == NULL) {
		pr_err ("%s() failed to get i2c0 adapter\n", __func__);
		return -EFAULT;
	}

	if (use_spi)
		ret = spi_init();
	else
		ret = i2c_init();

	return ret;
}

static void __exit si_8620_exit(void)
{
	int	idx;

	pr_info("si_8620_exit called\n");

	si_8620_power_control(false);

	if (use_spi) {
		mhl_tx_remove(&spi_dev->dev);
		spi_unregister_driver(&si_86x0_mhl_tx_spi_driver);
		spi_unregister_device(spi_dev);
	} else {
		mhl_tx_remove(&device_addresses[0].client->dev);
		MHL_TX_DBG_INFO("client removed\n");
		i2c_del_driver(&si_8620_mhl_tx_i2c_driver);
		MHL_TX_DBG_INFO("i2c driver deleted from context\n");

		for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
			MHL_TX_DBG_INFO("\n");
			if (device_addresses[idx].client != NULL){
				MHL_TX_DBG_INFO("unregistering device:%p\n",device_addresses[idx].client);
				i2c_unregister_device(device_addresses[idx].client);
			}
		}
	}
	MHL_TX_DBG_ERR("driver unloaded.\n");
}

static int debug_level_stack[15];
static unsigned int debug_level_stack_ptr=0;
void push_debug_level(int new_verbosity)
{
	if (debug_level_stack_ptr < ARRAY_SIZE(debug_level_stack)){
		/* stack is initially empty */
		debug_level_stack[debug_level_stack_ptr++]=debug_level;
		debug_level=new_verbosity;
	}else{
		MHL_TX_DBG_ERR("%sdebug_level_stack overflowed%s\n",ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
	}
}
void pop_debug_level(void)
{
	if (debug_level_stack_ptr >0){
		if (debug_level_stack_ptr > ARRAY_SIZE(debug_level_stack)){
			MHL_TX_DBG_ERR("%sdebug_level_stack overflowed%s\n",ANSI_ESC_RED_TEXT,ANSI_ESC_RESET_TEXT);
		}else {
			debug_level = debug_level_stack[--debug_level_stack_ptr];
		}
	}
}

int si_8620_register_callbacks(si_mhl_callback_api_t *p_callbacks)
{
	struct device *dev = NULL;
	int status=0;
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(dev);
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else{
		if (NULL != p_callbacks){
			hw_context->callbacks = *p_callbacks;
		}
	}

	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL(si_8620_register_callbacks);

int si_8620_info_frame_change(
			 hpd_high_callback_status mode_parm
			,avif_or_cea_861_dtd_u *p_avif_or_dtd, size_t avif_or_dtd_max_length
			,vsif_mhl3_or_hdmi_u *p_vsif, size_t vsif_max_length)
{
	struct device *dev = NULL;
	int status;
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	if (use_spi)
		dev = &spi_dev->dev;
	else
		dev = &device_addresses[0].client->dev;

	dev_context = dev_get_drvdata(dev);
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else{
		size_t xfer_size;

		memset(&hw_context->vsif_mhl3_or_hdmi_from_callback
				,0,sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback));
		memset(&hw_context->avif_or_dtd_from_callback
				,0,sizeof(hw_context->avif_or_dtd_from_callback));

		if (sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback)
			< vsif_max_length)
		{
			xfer_size = sizeof(hw_context->vsif_mhl3_or_hdmi_from_callback);
		}else{
			xfer_size = vsif_max_length;
		}
		memcpy(&hw_context->vsif_mhl3_or_hdmi_from_callback
				,p_vsif
				,xfer_size);

		if (sizeof(hw_context->avif_or_dtd_from_callback)
			< avif_or_dtd_max_length)
		{
			xfer_size = sizeof(hw_context->avif_or_dtd_from_callback);
		}else{
			xfer_size = avif_or_dtd_max_length;
		}
		memcpy(&hw_context->avif_or_dtd_from_callback
				,p_avif_or_dtd
				,xfer_size);

		status = si_mhl_tx_drv_set_display_mode(dev_context,mode_parm);
	}

	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL(si_8620_info_frame_change);

late_initcall(si_8620_init);
module_exit(si_8620_exit);

MODULE_DESCRIPTION("Silicon Image MHL Transmitter driver");
MODULE_AUTHOR("Silicon Image <http://www.siliconimage.com>");
MODULE_LICENSE("GPL");
