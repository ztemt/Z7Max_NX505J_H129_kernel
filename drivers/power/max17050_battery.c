/*
 * MAX17050 battery driver
 *
 * Copyright (C) 2013¡rwangshuai <wang.shuai12@zte.com.cn>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <max17050_battery.h>
#include <bq24192_charger.h>


#define MAX17050_NAME		"maxin17050"

#define MAX17050_REG_STATUS		0x00
#define MAX17050_REG_VALRT_TH		0x01
#define MAX17050_REG_TALRT_TH		0x02
#define MAX17050_REG_SOCALRT_TH		0x03
#define MAX17050_REG_ATRATE		0x04
#define MAX17050_REG_REMCAPREP		0x05
#define MAX17050_REG_SOCREP		0x06
#define MAX17050_REG_AGE		0x07
#define MAX17050_REG_TEMPERATURE	0x08
#define MAX17050_REG_VCELL		0x09
#define MAX17050_REG_CURRENT		0x0A
#define MAX17050_REG_AVERAGECURRENT	0x0B
#define MAX17050_REG_SOCMIX		0x0D
#define MAX17050_REG_SOCAV		0x0E
#define MAX17050_REG_REMCAPMIX		0x0F
#define MAX17050_REG_FULLCAP		0x10
#define MAX17050_REG_TTE		0x11
#define MAX17050_REG_QRESIDUAL00	0x12
#define MAX17050_REG_FULLSOCTHR		0x13
#define MAX17050_REG_RCELL		0x14
#define MAX17050_REG_AERAGETEMPERATURE	0x16
#define MAX17050_REG_CYCLES		0x17
#define MAX17050_REG_DESIGNCAP		0x18
#define MAX17050_REG_AVERAGEVCELL	0x19
#define MAX17050_REG_MAXMINTEMPERATURE	0x1A
#define MAX17050_REG_MAXMINVCELL	0x1B
#define MAX17050_REG_MAXMINCURRENT	0x1C
#define MAX17050_REG_CONFIG		0x1D
#define MAX17050_REG_ICHGTERM		0x1E
#define MAX17050_REG_REMCAPAV		0x1F
#define MAX17050_REG_VERSION		0x21
#define MAX17050_REG_QRESIDUAL10	0x22
#define MAX17050_REG_FULLCAPNOM		0x23
#define MAX17050_REG_TEMPNOM		0x24
#define MAX17050_REG_TEMPLIM		0x25
#define MAX17050_REG_AIN		0x27
#define MAX17050_REG_LEARNCFG		0x28
#define MAX17050_REG_FILTERCFG		0x29
#define MAX17050_REG_RELAXCFG		0x2A
#define MAX17050_REG_MISCCFG		0x2B
#define MAX17050_REG_TGAIN		0x2C
#define MAX17050_REG_TOFF		0x2D
#define MAX17050_REG_CGAIN		0x2E
#define MAX17050_REG_COFF		0x2F
#define MAX17050_REG_QRESIDUAL20	0x32
#define MAX17050_REG_IAVG_EMPTY		0x36
#define MAX17050_REG_FCTC		0x37
#define MAX17050_REG_RCOMP0		0x38
#define MAX17050_REG_TEMPCO		0x39
#define MAX17050_REG_V_EMPTY		0x3A
#define MAX17050_REG_FSTAT		0x3D
#define MAX17050_REG_TIMER		0x3E
#define MAX17050_REG_SHDNTIMER		0x3F
#define MAX17050_REG_QRESIDUAL30	0x42
#define MAX17050_REG_DQACC		0x45
#define MAX17050_REG_DPACC		0x46
#define MAX17050_REG_QH			0x4D
#define MAX17050_REG_CHAR_TBL		0x80
#define MAX17050_REG_VFOCV		0xFB
#define MAX17050_REG_SOCVF		0xFF

/* MAX17050_REG_STATUS */
#define MAX17050_R_POR			0x0002
#define MAX17050_R_BST			0x0008
#define MAX17050_R_VMN			0x0100
#define MAX17050_R_TMN			0x0200
#define MAX17050_R_SMN			0x0400
#define MAX17050_R_BI			0x0800
#define MAX17050_R_VMX			0x1000
#define MAX17050_R_TMX			0x2000
#define MAX17050_R_SMX			0x4000
#define MAX17050_R_BR			0x8000

/* MAX17050_REG_CONFIG */
#define MAX17050_R_BER			0x0001
#define MAX17050_R_BEI			0x0002
#define MAX17050_R_AEN			0x0004
#define MAX17050_R_FTHRM		0x0008
#define MAX17050_R_ETHRM		0x0010
#define MAX17050_R_ALSH			0x0020
#define MAX17050_R_I2CSH		0x0040
#define MAX17050_R_SHDN			0x0080
#define MAX17050_R_TEX			0x0100
#define MAX17050_R_TEN			0x0200
#define MAX17050_R_AINSH		0x0400
#define MAX17050_R_ALRTP		0x0800
#define MAX17050_R_VS			0x1000
#define MAX17050_R_TS			0x2000
#define MAX17050_R_SS			0x4000

#define RETRY_CNT		3
#define LEARNED_PARAM_LEN	50
#define CUST_MODE_LEN		48

#define START_MONITOR_MS	3000
//uart debug
#define ZTEMT_UART_DEBUG_ENABLE

#define MAX_INFO 1
#define MAX_DEBUG 4
//log level < maxlog_level will show
int maxlog_level = 3;  
module_param(maxlog_level, int, 0644);

#define MAXLOG_INFO(fmt, args...) \
		if (MAX_INFO < maxlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)
	
#define MAXLOG_DEBUG(fmt, args...) \
		if (MAX_DEBUG < maxlog_level) \
			printk(KERN_WARNING "_%s: "  fmt,__func__, ##args)


struct max17050_chip {
	struct device			*dev;
	struct i2c_client *i2c;
	struct delayed_work	   batt_worker;
	struct power_supply	   *batt_psy;
	struct device_node *dev_node;
	struct max17050_config_data *pdata;
	struct dentry		*dent;
	char learned_parameter[LEARNED_PARAM_LEN];
	int r_sns;
	int irq;
	int irq_gpio;
	int first_resume;
	int batt_soc;
	int batt_status;
	int batt_health;
	bool current_sensing;
	bool batt_por;
	bool is_sleep;
};

static struct max17050_chip   *max_chip = NULL;

struct max17050_config_data {
	u16 filtercfg;
	u16 relaxcfg;
	u16 fullsocthr;
	u16 rcomp0;
	u16 tmpco;
	u16 termcurr;
	u16 tgain;
	u16 toff;
	u16 vempty;
	u16 qrtable00;
	u16 qrtable10;
	u16 qrtable20;
	u16 qrtable30;
	u16 capacity;
	u16 vf_fullcap;
	u16 custome_model[CUST_MODE_LEN];
};

enum power_supply_property max17050_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
};

extern int qpnp_get_battery_temp(void);
extern int qpnp_is_batt_present(void);
static int max17050_hw_init(struct max17050_chip *max17050, struct max17050_config_data *pdata);

/******************************************************** 
 *					 I2C I/O function 				              *
 *********************************************************/
static inline int max17050_write_reg(struct max17050_chip *chip, u8 reg, u16 val)
{
	val = __cpu_to_le16(val);

	return i2c_smbus_write_i2c_block_data(chip->i2c, reg, sizeof(u16), (u8 *)&val);
}

static inline int max17050_read_reg(struct max17050_chip *chip, u8 reg, u16 *val)
{
	int ret;
	u16 tmp;

	ret = i2c_smbus_read_i2c_block_data(chip->i2c, reg, sizeof(u16), (u8 *)&tmp);
	*val = __le16_to_cpu(tmp);

	return ret;
}

static inline int max17050_write_16regs(struct max17050_chip *max17050, u8 reg, u16 *buf)
{
	u16 tmp[16];
	int i;

	for (i = 0; i < 16; i++) {
		tmp[i] = __cpu_to_le16(buf[i]);
	}

	return i2c_smbus_write_i2c_block_data(max17050->i2c, reg, 16 * sizeof(u16), (u8 *)tmp);
}

static inline int max17050_read_16regs(struct max17050_chip *max17050, u8 reg, u16 *buf)
{
	u16 tmp[16];
	int ret;
	int i;

	ret = i2c_smbus_read_i2c_block_data(max17050->i2c, reg, 16 * sizeof(u16), (u8 *)tmp);
	if (likely(ret >= 0)) {
		for (i = 0; i < 16; i++) {
			buf[i] = __le16_to_cpu(tmp[i]);
		}
	}

	return ret;
}

static inline int max17050_write_verify_reg(struct max17050_chip *max17050, u8 reg, u16 val)
{
	int i;
	int ret;
	u16 tmp;

	for (i = 0; i < RETRY_CNT; i++) {
		ret = max17050_write_reg(max17050, reg, val);
		if (unlikely(ret < 0))
			return ret;
			
		ret = max17050_read_reg(max17050, reg, &tmp);
		if (unlikely(ret < 0))
			return ret;

		if (likely(val == tmp))
			return 0;
	}
	
	return -EIO;
}

//#define MAXIN_SHUTDOWN_MODE
#ifdef MAXIN_SHUTDOWN_MODE
static int  max17050_masked_write(struct max17050_chip *chip, u8 reg,
							u16 mask, u16 val)
{
	int rc;
	u16 buf;

	rc = max17050_read_reg(chip,reg,&buf);
	if (rc<0) {
		pr_err("max17050_read_reg failed: reg=0x%x, rc=%d\n", reg, rc);
		return rc;
	}
	
	buf &= ~mask;
	buf |= val & mask;

	rc = max17050_write_reg(chip,reg,buf);
	if (rc<0) {
		pr_err("max17050_write_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	
	return 0;
}

#define MAX_SHURDOWN_TIME_MASK     0xe000
#define MAX_SHURDOWN_MODE_MASK     0x0080
#define MAX_SHURDOWN_SHIT  7
int max17050_shutdown_mode(int enable)
{
    u16 temp;
	int rc;
	
    if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return 0;
	}
	if(enable)
        temp = 1 << MAX_SHURDOWN_SHIT;
	else
		temp = 0;
  
    rc = max17050_masked_write(max_chip,MAX17050_REG_CONFIG,MAX_SHURDOWN_MODE_MASK,temp);
	if (rc) {
		pr_err("max17050_masked_write failed to modify shdn, rc=%d\n", rc);
		return rc;
	}

	rc = max17050_masked_write(max_chip,MAX17050_REG_SHDNTIMER,MAX_SHURDOWN_TIME_MASK,0);
	if (rc) {
		pr_err("max17050_masked_write failed to modify shdn, rc=%d\n", rc);
		return rc;
	}
	mdelay(10);

	return 0;	
}
EXPORT_SYMBOL_GPL(max17050_shutdown_mode);
static int maxin_shutdown_notify(struct notifier_block *this, unsigned long code,
			  void *unused)
{
    printk("%s: go to shutdown...\n",__func__);
	max17050_shutdown_mode(1);
	
	return NOTIFY_DONE;
}
static struct notifier_block maxin_notifier = {
	.notifier_call = maxin_shutdown_notify,
};
#endif
int max17050_get_batt_voltage(void)
{
    int ret;
	u16 value;
	int temp;

	if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return 0;
	}

	ret = max17050_read_reg(max_chip, MAX17050_REG_VCELL, &value);
	if (ret<0) {
		pr_err("%s:fail to read batt vol\n",__func__);
		return 0;
	}

	/* unit = 0.625mV */
	temp = (short)(value >> 3) * 625/1000;
	return temp*1000;
}

int max17050_get_batt_temp(void)
{
    int ret;
	u16 value;
	int temp;

	if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return DEFAULT_TEMP;
	}
	
    ret = max17050_read_reg(max_chip, MAX17050_REG_TEMPERATURE, &value);
	if (ret<0) {
		pr_err("%s:fail to read batt temp\n",__func__);
		return DEFAULT_TEMP;
	}
	
	/* unit = tenths of degree Celsius */
	temp = ((short)value * 10) / 256;
	return temp;
}

static int max17050_set_batt_temp(struct max17050_chip *chip,int batt_temp)
{
    int ret;
	u16 value;

    value = (batt_temp * 256) / 10;
	ret = max17050_write_reg(chip,MAX17050_REG_TEMPERATURE,value);
	if (ret<0) {
		pr_err("%s:fail to read batt temp\n",__func__);
		return -1;
	}
	
	return 0;
}
static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);
	return soc;
}

int max17050_get_batt_soc(void)
{
    int ret;
	u16 value;
	u8 reg;
	int temp;

	if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}
	
	reg = max_chip->current_sensing ? MAX17050_REG_SOCREP : MAX17050_REG_SOCVF;
    ret = max17050_read_reg(max_chip,reg,&value);
	if (ret<0) {
		pr_err("%s:fail to read batt soc\n",__func__);
		return DEFAULT_SOC;
	}

	/* unit = (1 / 256) % */
	temp = ((short)value + 0x0080) >> 8;

	return bound_soc(temp);
}

int max17050_report_batt_capacity(void)
{
	if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}

	if(max_chip->first_resume)
		return max17050_get_batt_soc();
	
	MAXLOG_DEBUG("report soc=%d\n",max_chip->batt_soc);
	return max_chip->batt_soc;
}


int max17050_get_ibatt_now(void)
{
    int ret;
	u16 value;
	int ibatt = 0;

	if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}
	
	if (max_chip->r_sns > 0) {
		ret = max17050_read_reg(max_chip, MAX17050_REG_CURRENT, &value);
		if (ret<0) {
		    pr_err("%s:fail to read ibatt now\n",__func__);
		    return 1;
	    }
		/* unit = 1.5625uV / RSENSE */
		ibatt = (short)value * 15625 / max_chip->r_sns / 10000;
	}
	return ibatt*1000;
}

int max17050_get_prop_batt_status(void)
{
	return bq_prop_batt_status();
}

int max17050_get_prop_batt_health(void)
{
    return bq_prop_batt_health();
}
static int max17050_get_property(struct power_supply *psy,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct max17050_chip *chip = max_chip;
	u16 value;
	int ret = 0;

    if(!chip || chip->is_sleep){
		pr_err("%s:chip call before init\n",__func__);
		return -1;
	}
		
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max17050_get_prop_batt_status();
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq_prop_charging_type();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max17050_get_prop_batt_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = qpnp_is_batt_present();
		break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = max17050_read_reg(chip, MAX17050_REG_CYCLES, &value);
		if (likely(ret >= 0)) {
			val->intval = value;
		}
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = max17050_read_reg(chip, MAX17050_REG_MAXMINVCELL, &value);
		if (likely(ret >= 0)) {
			/* unit = 20mV */
			val->intval = (int)(value >> 8) * 20000;
		}
		break;
	
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:		
		ret = max17050_read_reg(chip, MAX17050_REG_MAXMINVCELL, &value);
		if (likely(ret >= 0)) {
			/* unit = 20mV */
			val->intval = (int)(value & 0xFF) * 20000;
		}
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max17050_get_batt_voltage();
		break;
		
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = max17050_read_reg(chip, MAX17050_REG_AVERAGEVCELL, &value);
		if (likely(ret >= 0)) {
			/* unit = 0.625mV */
			val->intval = (int)(value >> 3) * 625;
		}
		break;

	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chip->r_sns > 0) {
			ret = max17050_read_reg(chip, MAX17050_REG_MAXMINCURRENT, &value);
			if (likely(ret >= 0)) {
				/* unit = 0.4mV / RSENSE */
				val->intval = (int)(value >> 8) * 400 * 1000 / chip->r_sns;
			}
		}		
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = max17050_get_ibatt_now();
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (chip->r_sns > 0) {
			ret = max17050_read_reg(chip, MAX17050_REG_AVERAGECURRENT, &value);
			if (likely(ret >= 0)) {
				/* unit = 1.5625uV / RSENSE */
				val->intval = (int)value * 15625 / chip->r_sns / 1000;
			}
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (chip->r_sns > 0) {
			ret = max17050_read_reg(chip, MAX17050_REG_DESIGNCAP, &value);
			if (likely(ret >= 0)) {
				/* unit = 5.0uVh / RSENSE */
				val->intval = (int)value * 5 * 1000 / chip->r_sns;
			}
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (chip->r_sns > 0) {
			ret = max17050_read_reg(chip, MAX17050_REG_FULLCAPNOM, &value);
			if (likely(ret >= 0)) {
				/* unit = 5.0uVh / RSENSE */
				val->intval = (int)value * 5 * 1000 / chip->r_sns;
			}
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (chip->r_sns > 0) {
			ret = max17050_read_reg(chip, MAX17050_REG_REMCAPREP, &value);
			if (likely(ret >= 0)) {
				/* unit = 5.0uVh / RSENSE */
				val->intval = (int)value * 5 * 1000 / chip->r_sns;
			}
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_AVG:
		if (chip->r_sns > 0) {
			ret = max17050_read_reg(chip, MAX17050_REG_REMCAPAV, &value);
			if (likely(ret >= 0)) {
				/* unit = 5.0uVh / RSENSE */
				val->intval = (int)value * 5 * 1000 / chip->r_sns;
			}
		}
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max17050_report_batt_capacity();
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = qpnp_get_battery_temp();
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = max17050_read_reg(chip, MAX17050_REG_TTE, &value);
		if (likely(ret >= 0)) {
			/* unit = 3min */
			val->intval = value * 180;
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int
max17050_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	int rc = 0;

	pr_debug("%s psp=%d intval=%d\n",__func__,psp,val->intval);
	return rc;
}

int max17050_set_power_supply(struct power_supply *batt_psy)
{
    if(!batt_psy)
		return -1;
		
    batt_psy->properties = max17050_props;
    batt_psy->num_properties = ARRAY_SIZE(max17050_props);
	batt_psy->get_property = max17050_get_property;
	batt_psy->set_property = max17050_set_property;

	return 0;
}

static void update_power_supply(struct max17050_chip *chip)
{
	if (chip->batt_psy == NULL || chip->batt_psy < 0)
		chip->batt_psy = power_supply_get_by_name("battery");

	if (chip->batt_psy > 0)
		power_supply_changed(chip->batt_psy);
}
static int max_hwinit = 0;
module_param(max_hwinit, int, 0644);

#ifdef ZTEMT_UART_DEBUG_ENABLE
static int debug_uart = 3;
module_param(debug_uart, int, 0644);
#endif
static void max17050_dump_regs(struct max17050_chip *chip)
{
    int i;
	u16 val;
	static int first_flag = 1;

	if(first_flag){
		first_flag = 0;
		printk("MDREG ");
		for(i=0;i<=0x4d;i++){
			printk("%4x ",i);
		}
		printk("%4x %4x \n",0xfb, 0xff);
	}
	printk("MDREG ");
	
	for(i=0;i<=0x4d;i++)
	{
		max17050_read_reg(chip, i, &val);
        printk("%4x ",val);
	}

	max17050_read_reg(chip,0xfb, &val);
    printk("%4x ",val);

	max17050_read_reg(chip,0xff, &val);
    printk("%4x \n",val);
}

#define BATT_LOW_MONITOR_MS	10000
#define BATT_NORMAL_MONITOR_MS	20000
#define BATT_SLOW_MONITOR_MS	60000
static int max17050_get_delay_time(int battery_soc)
{
    if(battery_soc > 90)
		return BATT_SLOW_MONITOR_MS;
    else if(battery_soc > 20)
		return BATT_NORMAL_MONITOR_MS;
	else 
		return BATT_LOW_MONITOR_MS;
}

static void max17050_batt_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct max17050_chip *chip = container_of(dwork,struct max17050_chip,batt_worker);
	int power_supply_change = 0;
    int batt_soc,batt_temp,batt_ma,batt_mv;
	int last_batt_soc;
	int new_batt_status,new_batt_health;
	int usb_in;
	int chg_st;
	int rc;
	u16 val;

#ifdef ZTEMT_UART_DEBUG_ENABLE
	if(debug_uart){
		console_printk[0] = 7;
		debug_uart--;
	}
#endif

	rc = max17050_read_reg(chip, MAX17050_REG_STATUS, &val);
	if (unlikely(rc < 0)) {
		dev_err(&chip->i2c->dev, "cannot read STATUS\n");
	}

	if (val & MAX17050_R_POR) {
		dev_info(&chip->i2c->dev, "detected POR\n");
		max17050_hw_init(chip,chip->pdata);
	}

    if(max_hwinit){
		max17050_hw_init(chip,chip->pdata);
		max_hwinit = 0;
	}

	batt_temp = qpnp_get_battery_temp();
	rc = max17050_set_batt_temp(chip,batt_temp);
	if(rc<0)
		pr_err("fail to write batt temp\n");

	batt_soc = max17050_get_batt_soc();
	batt_ma = max17050_get_ibatt_now()/1000;
	batt_mv = max17050_get_batt_voltage()/1000;
	usb_in = bq24192_is_charger_online();
	chg_st = bq24192_get_chg_status();
	
	last_batt_soc = chip->batt_soc;
	
	//after charging full for a long time, soc < 100 and charger is online, so set soc to 100%
	if( usb_in && last_batt_soc==100 && (batt_soc==99 || batt_soc==98) ){	
		if( batt_ma > -10 ){
			batt_soc = 100;
			MAXLOG_INFO("%s set soc to 100\n",__func__);
		}
	}

	//after charging done and stop charging,but  soc < 100 and charger is online, so set soc  to 100%
	if(chg_st==BQ_CHGING_DONE && (batt_ma>=0 && batt_ma<=10) && (batt_soc>=95 && batt_soc<100)){
        batt_soc = 100;
		MAXLOG_INFO("%s chg done set soc to 100\n",__func__);
	}

	if(last_batt_soc >= 0){
		if(chip->first_resume){
			last_batt_soc = batt_soc;
            chip->first_resume = 0;
		}else if( last_batt_soc < batt_soc && batt_ma>=0 )
			last_batt_soc++;
		else if( last_batt_soc > batt_soc && batt_ma<0 )
			last_batt_soc--;
	}else
	    last_batt_soc = batt_soc;
	
	if(chip->batt_soc != last_batt_soc){
		chip->batt_soc = bound_soc(last_batt_soc);
		power_supply_change = 1;
	}

	new_batt_status = max17050_get_prop_batt_status();
	if(chip->batt_status != new_batt_status){
		chip->batt_status = new_batt_status;
		power_supply_change = 1;
	}

	new_batt_health = max17050_get_prop_batt_health();
	if(chip->batt_health != new_batt_health){
		chip->batt_health = new_batt_health;
		power_supply_change = 1;
	}

	if(power_supply_change)
		update_power_supply(chip);

	MAXLOG_INFO("BATT soc=%d temp=%d i=%d vol=%d usb_in=%d\n",chip->batt_soc,batt_temp,batt_ma,batt_mv,usb_in);

	if (MAX_DEBUG < maxlog_level)
		max17050_dump_regs(chip);

	schedule_delayed_work(&chip->batt_worker,
			  round_jiffies_relative(msecs_to_jiffies(max17050_get_delay_time(chip->batt_soc))));
}

#if 1
static int debug_reg = 0;
module_param(debug_reg, int, 0644);


static int max17050_debug;
static int max17050_debug_mode(const char *val, struct kernel_param *kp)
{
	int ret;
    u16 value = 0;
	
	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!max_chip) {
		pr_err("%s:called before init\n",__func__);
		return 1;
	}
	
	printk("__%s: max17050_debug=%d!\n",__func__,max17050_debug);
	switch(max17050_debug){
    case 0:
		ret = max17050_read_reg(max_chip, debug_reg, &value);
		printk("__%s: debug reg[0x%x]=0x%x\n",__func__,debug_reg,value);
		break;
	case 1:
		value = max17050_get_batt_temp();
		printk("__%s: batt_soc=%d\n",__func__,value);
		break;
	default:
		break;
	};
	
	return 0;
}
module_param_call(max17050_debug, max17050_debug_mode, param_get_uint,
					&max17050_debug, 0644);
#endif

static int
max17050_battery_read_dt_props(struct max17050_chip *chip,struct max17050_config_data *pdata)
{
    int rc;
	u32 tmp;
	const u8 *data;
	int len;
	int i;

	rc = of_property_read_u32(chip->dev_node, "maxin-rsns-momh", &tmp);
	chip->r_sns = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-irq-gpio", &tmp);
	chip->irq_gpio = (!rc ? tmp : 0);

	chip->current_sensing = of_property_read_bool(chip->dev_node, "maxin-current-sensing");

	rc = of_property_read_u32(chip->dev_node, "maxin-filter-cfg", &tmp);
	pdata->filtercfg = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-relax-cfg", &tmp);
	pdata->relaxcfg = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-fullsoc-thr", &tmp);
	pdata->fullsocthr = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-rccomp0", &tmp);
	pdata->rcomp0 = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-tmpco", &tmp);
	pdata->tmpco = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-term-curr", &tmp);
	pdata->termcurr = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-tgain", &tmp);
	pdata->tgain = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-toff", &tmp);
	pdata->toff = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-vempty", &tmp);
	pdata->vempty = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-qrtable00", &tmp);
	pdata->qrtable00 = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-qrtable10", &tmp);
	pdata->qrtable10 = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-qrtable20", &tmp);
	pdata->qrtable20 = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-qrtable30", &tmp);
	pdata->qrtable30 = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-capacity", &tmp);
	pdata->capacity = (!rc ? tmp : 0);

	rc = of_property_read_u32(chip->dev_node, "maxin-vf-fullcap", &tmp);
	pdata->vf_fullcap = (!rc ? tmp : 0);

	data = of_get_property(chip->dev_node, "maxin-custome-model", &len);
	if ((!data) || (len != CUST_MODE_LEN*2)) {
		pr_err("%s:%d, Unable to read maxin-custome-model \n",__func__, __LINE__);
		return -EINVAL;
	}
	
	for (i = 0; i < len; i+=2)
		pdata->custome_model[i/2] = (data[i]<<8) + data[i+1];

	return 0;
}

static int max17050_save_learned_parameter(struct max17050_chip *max17050)
{
	u16 rcomp0, tempco, fullcap, cycles, fullcapnom, iavg_empty, qrtable0, qrtable1, qrtable2, qrtable3;
	int ret;

	/* Step 20. Save Learned Parameters
	 * It is recommended to save the learned capacity parameters every time
	 * bit 6 of the Cycles register toggles (so that it is saved every 64% change in the battery)
	 * so that if power is lost the values can easily be restored.
	 */
	ret = max17050_read_reg(max17050, MAX17050_REG_RCOMP0, &rcomp0);
	ret |= max17050_read_reg(max17050, MAX17050_REG_TEMPCO, &tempco);
	ret |= max17050_read_reg(max17050, MAX17050_REG_FULLCAP, &fullcap);
	ret |= max17050_read_reg(max17050, MAX17050_REG_CYCLES, &cycles);
	ret |= max17050_read_reg(max17050, MAX17050_REG_FULLCAPNOM, &fullcapnom);
	ret |= max17050_read_reg(max17050, MAX17050_REG_IAVG_EMPTY, &iavg_empty);
	ret |= max17050_read_reg(max17050, MAX17050_REG_QRESIDUAL00, &qrtable0);
	ret |= max17050_read_reg(max17050, MAX17050_REG_QRESIDUAL10, &qrtable1);
	ret |= max17050_read_reg(max17050, MAX17050_REG_QRESIDUAL20, &qrtable2);
	ret |= max17050_read_reg(max17050, MAX17050_REG_QRESIDUAL30, &qrtable3);
	if (unlikely(ret < 0)) {
		dev_err(&max17050->i2c->dev, "cannot read learned parameter\n");
		return -EIO;
	}

	snprintf(max17050->learned_parameter, LEARNED_PARAM_LEN,
		"%04X %04X %04X %04X %04X %04X %04X %04X %04X %04X",
		rcomp0, tempco, fullcap, cycles, fullcapnom, iavg_empty,
		qrtable0, qrtable1, qrtable2, qrtable3);

	return 0;
}

static int max17050_restore_learned_parameter(struct max17050_chip *max17050)
{
	unsigned int rcomp0, tempco, fullcap, cycles, fullcapnom, iavg_empty, qrtable0, qrtable1, qrtable2, qrtable3;
	u16 fullcap0, remcap, val;
	int ret;

	/* Step 21. Restoring Capacity Parameters
	 */
	sscanf(max17050->learned_parameter, "%x %x %x %x %x %x %x %x %x %x",
		&rcomp0, &tempco, &fullcap, &cycles, &fullcapnom, &iavg_empty,
		&qrtable0, &qrtable1, &qrtable2, &qrtable3);

	/* If power is lost, then the Capacity information can be easily restored with the following procedure.
	 */
	ret = max17050_write_reg(max17050, MAX17050_REG_RCOMP0, rcomp0);
	ret |= max17050_write_reg(max17050, MAX17050_REG_TEMPCO, tempco);
	ret |= max17050_write_reg(max17050, MAX17050_REG_IAVG_EMPTY, iavg_empty);
	ret |= max17050_write_reg(max17050, MAX17050_REG_FULLCAPNOM, fullcapnom);
	ret |= max17050_write_reg(max17050, MAX17050_REG_QRESIDUAL00, qrtable0);
	ret |= max17050_write_reg(max17050, MAX17050_REG_QRESIDUAL10, qrtable1);
	ret |= max17050_write_reg(max17050, MAX17050_REG_QRESIDUAL20, qrtable2);
	ret |= max17050_write_reg(max17050, MAX17050_REG_QRESIDUAL30, qrtable3);
	if (unlikely(ret < 0)) {
		return -EIO;
	}

	/* Step 22. Wait 350ms
	 */
	mdelay(350);

	/* Step 23. Restore FullCap
	 */
	ret = max17050_read_reg(max17050, 0x35, &fullcap0);
	ret |= max17050_read_reg(max17050, MAX17050_REG_SOCMIX, &val);
	if (unlikely(ret < 0)) {
		return -EIO;
	}

	remcap = val * fullcap0 / 25600;
	ret = max17050_write_reg(max17050, MAX17050_REG_REMCAPMIX, remcap);
	ret |= max17050_write_reg(max17050, MAX17050_REG_FULLCAP, fullcap);
	ret |= max17050_write_reg(max17050, MAX17050_REG_DQACC, fullcapnom / 4);
	ret |= max17050_write_reg(max17050, MAX17050_REG_DPACC, 0x3200);
	if (unlikely(ret < 0)) {
		return -EIO;
	}

	/* Step 22. Wait 350ms
	 */
	mdelay(350);

	/* Step 25. Restore Cycles Register
	 */
	ret = max17050_write_reg(max17050, MAX17050_REG_CYCLES, cycles);
	if (cycles > 0xFF) {
		ret |= max17050_write_reg(max17050, MAX17050_REG_LEARNCFG, 0x0676);
	}
	if (unlikely(ret < 0)) {
		dev_err(&max17050->i2c->dev, "cannot write CYCLES\n");
		return -EIO;
	}

	return 0;
}
static ssize_t max17050_show_learned_parameter(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max17050_chip *max17050 = dev_get_drvdata(dev);
	int ret;

	ret = max17050_save_learned_parameter(max17050);
	if (unlikely(ret < 0)) {
		dev_err(dev, "cannot read learned_parameter\n");
		return ret;
	}
	
	return sprintf(buf, "%s\n", max17050->learned_parameter);
}

static ssize_t max17050_set_learned_parameter(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max17050_chip *max17050 = dev_get_drvdata(dev);
	int ret;
	
	strncpy(max17050->learned_parameter, buf, LEARNED_PARAM_LEN);
	ret = max17050_restore_learned_parameter(max17050);
	if (unlikely(ret < 0)) {
		dev_err(dev, "cannot write learned_parameter\n");
		return ret;
	}
	
	return count;
}
//path: /sys/bus/i2c/drivers/max17050/1-0036
const static DEVICE_ATTR(learned_parameter, S_IRUGO | S_IWUSR,
		max17050_show_learned_parameter, max17050_set_learned_parameter);


static ssize_t max17050_show_power_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct max17050_chip *max17050 = dev_get_drvdata(dev);
	
	return sprintf(buf, "%d\n", max17050->batt_por);
}
static ssize_t max17050_store_power_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct max17050_chip *max17050 = dev_get_drvdata(dev);
	
	max17050->batt_por = *buf;
	return 1;
}

const static DEVICE_ATTR(batt_por, S_IRUGO | S_IWUSR,
	            max17050_show_power_status, max17050_store_power_status);


static int max17050_hw_init(struct max17050_chip *max17050, struct max17050_config_data *pdata)
{
	u16 vfsoc, remcap, repcap;
	u16 val;
	u16 tmp[CUST_MODE_LEN];
	int i, n;
	u32 sum;
	int ret;

	MAXLOG_INFO(" start...\n");
	/*
	* INITIALIZE REGISTERS TO RECOMMENDED CONFIGURATION
	* -------------------------------------------------
	* The MAX77696 Fuel Gauge should be initialized prior to being used.
	* The following three registers should be written to these values in order
	* for the MAX77696 Fuel Gauge to perform at its best. These values are
	* written to RAM, so they must be written to the device any time that power
	* is applied or restored to the device. Some registers are updated
	* internally, so it is necessary to verify that the register was written
	* correctly to prevent data collisions.
	*/

	/* Step 0. Check for POR
	 */
	ret = max17050_read_reg(max17050, MAX17050_REG_STATUS, &val);
	if (unlikely(ret < 0)) {
		dev_err(&max17050->i2c->dev, "cannot read POR\n");
		return ret;
	}

	if (!(val & MAX17050_R_POR) && !max_hwinit) {
		dev_info(&max17050->i2c->dev, "POR is not set\n");
		return 0;
	}
	
	max17050->batt_por = 1;

	/* Delay at least 500ms
	 */
	msleep(500);

	/* Initialize Configuration
	 */
	max17050_write_reg(max17050, MAX17050_REG_CONFIG,
			MAX17050_R_TS | MAX17050_R_TEN | MAX17050_R_ETHRM | MAX17050_R_TEX);

	max17050_write_reg(max17050, MAX17050_REG_FILTERCFG, pdata->filtercfg);
	if(pdata->relaxcfg)
	    max17050_write_reg(max17050, MAX17050_REG_RELAXCFG, pdata->relaxcfg);
	max17050_write_reg(max17050, MAX17050_REG_LEARNCFG, 0x2606);
	max17050_write_reg(max17050, MAX17050_REG_FULLSOCTHR, pdata->fullsocthr);

	/*
	* LOAD CUSTOM MODEL AND PARAMETERS
	* --------------------------------
	* The custom model that is stored in the MAX77696 Fuel Gauge is also
	* written to RAM and so it must be written to the device any time that
	* power is applied or restored to the device. When the device is powered
	* on, the host software must first unlock write access to the model, write
	* the model, verify the model was written properly, and then lock access to
	* the model. After the model is loaded correctly, simply write a few
	* registers with customized parameters that will be provided by Maxim.
	*/

	for (i = 0; i < RETRY_CNT; i++) {
		/* Step 4. Unlock Model Access
		 */
		max17050_write_reg(max17050, 0x62, 0x0059);
		max17050_write_reg(max17050, 0x63, 0x00C4);

		/* Step 5. Write/Read/Verify the Custom Model
		 * Once the model is unlocked, the host software must write the 48 word model to the MAX17042/7.
		 * The model islocated between memory locations 0x80h and 0xAFh.
		 */
		max17050_write_16regs(max17050, 0x80, &pdata->custome_model[0]);
		max17050_write_16regs(max17050, 0x90, &pdata->custome_model[16]);
		max17050_write_16regs(max17050, 0xA0, &pdata->custome_model[32]);
		/* The model can be read directly back from the MAX17050.
		 * So simply read the 48 words of the model back from the device to verify if it was written correctly.
		 */
		max17050_read_16regs(max17050, 0x80, &tmp[0]);
		max17050_read_16regs(max17050, 0x90, &tmp[16]);
		max17050_read_16regs(max17050, 0xA0, &tmp[32]);

		/* If any of the values do not matc, return to step 4.
		 */
		ret = memcmp(&pdata->custome_model[0], &tmp[0], sizeof(tmp));
		if (likely(ret == 0))
			break;
	}
	if (unlikely(i >= RETRY_CNT)) {
		dev_err(&max17050->i2c->dev, "cannot write model data!!\n");
		return -EIO;
	}

	/* Steps 6 & 7 are deleted, numbering continues at step 8 for legacy reasons
	 */

	for (i = 0; i < RETRY_CNT; i++) {
		/* Step 8. Lock Model Access
		 */
		max17050_write_reg(max17050, 0x62, 0x0000);
		max17050_write_reg(max17050, 0x63, 0x0000);

		/* Step 9. Verify that Model Access is locked
		 * If the model remains unlocked, the MAX17042/7 will not be able to monitor the capacity of the battery.
		 * Therefore it is very critical that the Model Access is locked.
		 * To verify it is locked, simply read back the model as in Step 5.
		 */
		max17050_read_16regs(max17050, 0x80, &tmp[0]);
		max17050_read_16regs(max17050, 0x90, &tmp[16]);
		max17050_read_16regs(max17050, 0xA0, &tmp[32]);
		/* However, this time, all values should be read as 0x00h.
		 * If any values are non-zero, repeat Step 8 to make sure the Model Access is locked.
		 */
		for (sum = 0, n = 0; n < CUST_MODE_LEN; n++) {
			sum += tmp[n];
		}
		if (sum == 0)
			break;
	}
	if (unlikely(i >= RETRY_CNT)) {
		dev_err(&max17050->i2c->dev, "cannot unlock model data\n");
		return -EIO;
	}

	/* Step 10. Write Custom Parameters
	 */
	max17050_write_verify_reg(max17050, MAX17050_REG_RCOMP0, pdata->rcomp0);
	max17050_write_verify_reg(max17050, MAX17050_REG_TEMPCO, pdata->tmpco);
	max17050_write_reg(max17050, MAX17050_REG_ICHGTERM, pdata->termcurr);
	max17050_write_reg(max17050, MAX17050_REG_TGAIN, pdata->tgain);
	max17050_write_reg(max17050, MAX17050_REG_TOFF, pdata->toff);
	max17050_write_verify_reg(max17050, MAX17050_REG_V_EMPTY, pdata->vempty);
	max17050_write_verify_reg(max17050, MAX17050_REG_QRESIDUAL00, pdata->qrtable00);
	max17050_write_verify_reg(max17050, MAX17050_REG_QRESIDUAL10, pdata->qrtable10);
	max17050_write_verify_reg(max17050, MAX17050_REG_QRESIDUAL20, pdata->qrtable20);
	max17050_write_verify_reg(max17050, MAX17050_REG_QRESIDUAL30, pdata->qrtable30);
	
	/* Step 11. Update Full Capacity Parameters
	 */
	max17050_write_verify_reg(max17050, MAX17050_REG_FULLCAP, pdata->capacity);
	max17050_write_reg(max17050, MAX17050_REG_DESIGNCAP, pdata->vf_fullcap);
	max17050_write_verify_reg(max17050, MAX17050_REG_FULLCAPNOM, pdata->vf_fullcap);

	/* Step 13. Delay at least 350ms
	 */
	msleep(350);

	/* Step 14. Write VFSOC and QH values to VFSOC0 and QH0
	 */
	/* Read VFSOC */
	max17050_read_reg(max17050, MAX17050_REG_SOCVF, &vfsoc);
	/* Enable Write Access to VFSOC0 */
	max17050_write_reg(max17050, 0x60, 0x0080);
	/* Write and Verify VFSOC0 */
	max17050_write_reg(max17050, 0x48, vfsoc);
	/* Disable Write Access to VFSOC0 */
	max17050_write_reg(max17050, 0x60,0x0000);
	/* Read Qh register */
	max17050_read_reg(max17050, MAX17050_REG_QH, &val);
	/* Write Qh to Qh0 */
	max17050_write_reg(max17050, 0x4C, val);

	/* Step 15.5 Advance to Coulomb-Counter Mode
	 * Advancing the cycles register to a higher values makes the fuelgauge behave more like a coulomb counter.
	 * MAX17050 supports quicker insertion error healing by supporting starting from a lower learn stage.
	 * To Advance to Coulomb-Counter Mode,
	 * simply write the Cycles register to a value of 160% for MAX17042 and 96% for MAX17050.
	 */
	max17050_write_verify_reg(max17050, MAX17050_REG_CYCLES, 0x0060);

	/* Step 16. Load New Capacity Parameters */
	remcap = vfsoc * pdata->vf_fullcap / 25600;
	max17050_write_verify_reg(max17050, MAX17050_REG_REMCAPMIX, remcap);
	/* ModelScaling is used if your FullPoint VFSOC is scaled down.
	 * This was used as a fix for the MAX17042 early charge termination. MAX17050/50 will use ModelScaling = 1
	 */
	repcap = remcap * (pdata->capacity / pdata->vf_fullcap) / 1;
	max17050_write_verify_reg(max17050, MAX17050_REG_REMCAPREP, repcap);
	/* Write dQ_acc to 200% of Capacity and dP_acc to 200% */
	val = pdata->vf_fullcap / 4;
	max17050_write_verify_reg(max17050, MAX17050_REG_DQACC, val);
	max17050_write_verify_reg(max17050, MAX17050_REG_DPACC, 0x3200);
	max17050_write_verify_reg(max17050, MAX17050_REG_FULLCAP, pdata->capacity);
	max17050_write_reg(max17050, MAX17050_REG_DESIGNCAP, pdata->vf_fullcap);
	max17050_write_verify_reg(max17050, MAX17050_REG_FULLCAPNOM, pdata->vf_fullcap);
	max17050_write_reg(max17050, MAX17050_REG_SOCREP, vfsoc);

	/* Step 17. Initialization Complete
	 * Clear the POR bit to indicate that the custommodel and parameters were successfully loaded.
	 */
	max17050_read_reg(max17050, MAX17050_REG_STATUS, &val);
	max17050_write_verify_reg(max17050, MAX17050_REG_STATUS, val & ~MAX17050_R_POR);

	/* write CONFIG register
	 */
	val =	MAX17050_R_AEN |
		MAX17050_R_ETHRM |
		MAX17050_R_TEN |
		MAX17050_R_TEX;
	ret = max17050_write_reg(max17050, MAX17050_REG_CONFIG, val);

	MAXLOG_INFO("param init end.\n");

	return ret;
}

//#define MAXIN_17050_INT
#ifdef MAXIN_17050_INT
/********************************************************
aler init,use SOCrep ,see p53 in FG mannual
*********************/
static int max17050_alert_init(struct max17050_chip *max17050, int soc)
{
	
	u16 val = 0;
	int ret;	
	
    /*disable alert*/
    ret = max17050_read_reg(max17050, MAX17050_REG_CONFIG, &val); 
    val &= ~(MAX17050_R_AEN |MAX17050_R_ALSH);
   
    /*configure Alert pin ploarity, active low*/
    val &= ~MAX17050_R_ALRTP;
  
    ret |= max17050_write_reg(max17050, MAX17050_REG_CONFIG, val);
	/* Using RepSOC */
	ret |=max17050_read_reg(max17050, MAX17050_REG_MISCCFG, &val);
	val = val & (~(0x03));
	ret |= max17050_write_reg(max17050, MAX17050_REG_MISCCFG, val);
	
	/* set sAlert threshold */
	val = (0xff<<8) | (soc & 0xff);
	ret |= max17050_write_reg(max17050, MAX17050_REG_SOCALRT_TH, val);
 
	/* disable VAlert and TAlert */
	ret |= max17050_write_reg(max17050, MAX17050_REG_VALRT_TH, 0xff00);
    ret |= max17050_write_reg(max17050, MAX17050_REG_TALRT_TH, 0x7f80);
  
	mdelay(100);

	/* Enable SOC alerts */
	 ret |=max17050_read_reg(max17050, MAX17050_REG_CONFIG, &val); 
     val &= MAX17050_R_AEN;
	 ret |= max17050_write_reg(max17050, MAX17050_REG_CONFIG, val);
	 if (unlikely(ret < 0)) {
		dev_err(&max17050->i2c->dev, "failed to init alert\n");
		return -EIO;
	}

	return 0;
}

static irqreturn_t max17050_isr(int irq, void *irq_data)
{
	struct max17050_chip *max17050 = irq_data;

	printk("%s\n",__func__);
	update_power_supply(max17050);

	return IRQ_HANDLED;
}
#endif

/***********************************************************
*  for debug reg , path: sys/kernel/debug/max17050
*  reg: the reg to read or write
*  data: 'echo x > data' to write the reg  and 'cat data 'to read the reg
************************************************************/
static u8 maxin_reg;
static int get_reg_addr(void *data, u64 * val)
{
	*val = maxin_reg;
	return 0;
}

static int set_reg_addr(void *data, u64 val)
{
	maxin_reg = val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(set_reg_fops, get_reg_addr, set_reg_addr, "0x%02llx\n");

static int get_reg_data(void *data, u64 * val)
{
	struct max17050_chip *chip = (struct max17050_chip *)data;
	int ret;
	u16 value;

    ret = max17050_read_reg(chip, maxin_reg, &value);
	if (ret<0) {
		pr_err("%s:fail to read batt temp\n",__func__);
		return ret;
	}

	*val = value;
	return 0;
}

static int set_reg_data(void *data, u64 val)
{
	struct max17050_chip *chip = (struct max17050_chip *)data;
	int ret;
	u16  value = val;

	ret = max17050_write_reg(chip, maxin_reg, value);
	if (ret<0) {
	   pr_err("%s:fail to read batt temp\n",__func__);
	   return ret;
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rw_reg_fops, get_reg_data, set_reg_data, "0x%02llx\n");

static void max_create_debugfs_entries(struct max17050_chip *chip)
{

	chip->dent = debugfs_create_dir("max17050", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("max17050 couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("reg", 0644, chip->dent, chip, &set_reg_fops);
	debugfs_create_file("data", 0644, chip->dent, chip, &rw_reg_fops);
	return;
}

static int  max17050_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	struct max17050_chip *chip;
	struct max17050_config_data *pdata;
	int ret;
	
	printk("max17050_battery_probe\n");

	chip = kzalloc(sizeof(struct max17050_chip),GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate max17050_chip\n");
		return -ENOMEM;
	}

	pdata = kzalloc(sizeof(struct max17050_config_data),GFP_KERNEL);
	if (!pdata) {
		pr_err("Cannot allocate max17050_config_data\n");
		return -ENOMEM;
	}

	chip->pdata = pdata;
	chip->i2c = client;
	chip->dev_node = client->dev.of_node;
	chip->batt_soc = -1;
	i2c_set_clientdata(client, chip);
    
	max17050_battery_read_dt_props(chip,pdata);
	
	INIT_DELAYED_WORK(&chip->batt_worker,max17050_batt_worker);

	ret = max17050_hw_init(chip, pdata);
	if (unlikely(ret < 0)) {
		dev_err(&chip->i2c->dev, "failed to max17050_hw_init!\n");
		return ret;
	}

    #ifdef MAXIN_17050_INT
	ret = max17050_alert_init(chip,1);//jelphi,soc alert = 1%
	if (unlikely(ret < 0)) {
		  dev_err(&chip->i2c->dev, "failed init alert setting\n");
	  }
	
	chip->irq = gpio_to_irq(chip->irq_gpio);
	ret = request_irq(chip->irq,max17050_isr,
		    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, MAX17050_NAME, chip);
	if (unlikely(ret < 0)) {
		dev_err(&chip->i2c->dev, "failed to reqeust IRQ\n");
	}
    #endif
	
	(void)max17050_save_learned_parameter(chip);
	
	ret = device_create_file(&chip->i2c->dev, &dev_attr_learned_parameter);
	if (unlikely(ret < 0)) {
		dev_err(&chip->i2c->dev, "failed: cannot create learned_parameter.\n");
	}
	ret = device_create_file(&chip->i2c->dev, &dev_attr_batt_por);
	if (unlikely(ret < 0)) {
		dev_err(&chip->i2c->dev, "failed: cannot create power_lost.\n");
	}
	
	max_create_debugfs_entries(chip);

	max_chip = chip;
	max17050_batt_worker(&chip->batt_worker.work);

    pr_info("probe:r_sns=%d cur_sns=%d irq_gpio=%d\n",chip->r_sns,chip->current_sensing,chip->irq_gpio);
    return 0;

}

static int max17050_battery_remove(struct i2c_client *client)
{	
	struct max17050_chip *max17050 = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&max17050->batt_worker);
	kfree(max17050->pdata);
	kfree(max17050);
	max_chip = NULL;

	return 0;
}

static int max17050_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	struct max17050_chip *chip = i2c_get_clientdata(cl);
	
	MAXLOG_DEBUG(" goto suspend.\n");
	chip->is_sleep = 1;	
	cancel_delayed_work_sync(&chip->batt_worker);
	return 0;
};

static int max17050_resume(struct i2c_client *cl)
{
	struct max17050_chip *chip = i2c_get_clientdata(cl);

	MAXLOG_DEBUG(" goto resume.\n");
	chip->is_sleep = 0; 
	chip->first_resume = 1;
	schedule_delayed_work(&chip->batt_worker,
				  round_jiffies_relative(msecs_to_jiffies(500)));
	return 0;
};

static struct of_device_id  max17050_match_table[] = {
	{ .compatible = "maxim,max17050",},
	{}
};

static const struct i2c_device_id  max17050_id[] = {
	{ "max17050", 1 },
	{},
};

static struct i2c_driver max17050_battery_driver = {
	.driver = {
		.name = "max17050",
		.of_match_table = max17050_match_table,
	},
	.id_table 	= max17050_id,
	.probe 		= max17050_battery_probe,
	.remove 	= max17050_battery_remove,

	.suspend	= max17050_suspend,
	.resume 	= max17050_resume,
};


static int __init max17050_battery_init(void)
{
	printk( "%s:enter...\n", __func__);

	return i2c_add_driver(&max17050_battery_driver);
}

static void __exit bmax17050_battery_exit(void)
{
	printk( "%s:max17050 is exiting\n", __func__);

	i2c_del_driver(&max17050_battery_driver);
}

module_init(max17050_battery_init);


module_exit(bmax17050_battery_exit);

MODULE_AUTHOR("ztemt-swang");
MODULE_DESCRIPTION("max17050 battery driver");
MODULE_LICENSE("GPL");
