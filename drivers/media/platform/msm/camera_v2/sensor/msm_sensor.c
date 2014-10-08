/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_sd.h"
#include "camera.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif
//#define CONFIG_OIS_DEBUG
#ifdef CONFIG_OIS_DEBUG
#define CDBG_OIS(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG_OIS(fmt, args...) do { } while (0)
#endif

#if defined(CONFIG_IMX135)
int IMX135_update_wb_register_from_otp(struct msm_sensor_ctrl_t *s_ctrl);
int ofei_imx135_read_test(struct msm_sensor_ctrl_t *s_ctrl);
#endif


static int32_t msm_camera_get_power_settimgs_from_sensor_lib(
	struct msm_camera_power_ctrl_t *power_info,
	struct msm_sensor_power_setting_array *power_setting_array)
{
	int32_t rc = 0;
	uint32_t size;
	struct msm_sensor_power_setting *ps;
	bool need_reverse = 0;

	if ((NULL == power_info->power_setting) ||
		(0 == power_info->power_setting_size)) {

		ps = power_setting_array->power_setting;
		size = power_setting_array->size;
		if ((NULL == ps) || (0 == size)) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -EINVAL;
			goto FAILED_1;
		}

		power_info->power_setting =
		kzalloc(sizeof(*ps) * size, GFP_KERNEL);
		if (!power_info->power_setting) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto FAILED_1;
		}
		memcpy(power_info->power_setting,
			power_setting_array->power_setting,
			sizeof(*ps) * size);
		power_info->power_setting_size = size;
	}

	ps = power_setting_array->power_down_setting;
	size = power_setting_array->size_down;
	if (NULL == ps || 0 == size) {
		ps = power_info->power_setting;
		size = power_info->power_setting_size;
		need_reverse = 1;
	}

	power_info->power_down_setting =
	kzalloc(sizeof(*ps) * size, GFP_KERNEL);
	if (!power_info->power_down_setting) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_UP;
	}
	memcpy(power_info->power_down_setting,
		ps,
		sizeof(*ps) * size);
	power_info->power_down_setting_size = size;

	if (need_reverse) {
		int c, end = size - 1;
		struct msm_sensor_power_setting power_down_setting_t;
		for (c = 0; c < size/2; c++) {
			power_down_setting_t =
				power_info->power_down_setting[c];
			power_info->power_down_setting[c] =
				power_info->power_down_setting[end];
			power_info->power_down_setting[end] =
				power_down_setting_t;
			end--;
		}
	}

	return 0;
FREE_UP:
	kfree(power_info->power_setting);
FAILED_1:
	return rc;
}

static int32_t msm_sensor_get_dt_data(struct device_node *of_node,
	struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0, i = 0, ret = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3];

	s_ctrl->sensordata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!s_ctrl->sensordata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = s_ctrl->sensordata;

	rc = of_property_read_string(of_node, "qcom,sensor-name",
		&sensordata->sensor_name);
	CDBG("%s qcom,sensor-name %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SENSORDATA;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&s_ctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, s_ctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		s_ctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	rc = msm_sensor_get_sub_module_index(of_node, &sensordata->sensor_info);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SENSORDATA;
	}

	/* Get sensor mount angle */
	rc = of_property_read_u32(of_node, "qcom,mount-angle",
		&sensordata->sensor_info->sensor_mount_angle);
	CDBG("%s qcom,mount-angle %d, rc %d\n", __func__,
		sensordata->sensor_info->sensor_mount_angle, rc);
	if (rc < 0) {
		/* Invalidate mount angle flag */
		pr_err("%s Default sensor mount angle %d\n",
					__func__, __LINE__);
		sensordata->sensor_info->is_mount_angle_valid = 0;
		sensordata->sensor_info->sensor_mount_angle = 0;
		rc = 0;
	} else {
		sensordata->sensor_info->is_mount_angle_valid = 1;
	}

	rc = of_property_read_u32(of_node, "qcom,sensor-position",
		&sensordata->sensor_info->position);
	CDBG("%s qcom,sensor-position %d, rc %d\n", __func__,
		sensordata->sensor_info->position, rc);
	if (rc < 0) {
		pr_err("%s Default sensor position %d\n", __func__, __LINE__);
		sensordata->sensor_info->position = 0;
		rc = 0;
	}

	rc = of_property_read_u32(of_node, "qcom,sensor-mode",
		&sensordata->sensor_info->modes_supported);
	CDBG("%s qcom,sensor-mode %d, rc %d\n", __func__,
		sensordata->sensor_info->modes_supported, rc);
	if (rc < 0) {
		pr_err("%s Default sensor mode %d\n", __func__, __LINE__);
		sensordata->sensor_info->modes_supported = 0;
		rc = 0;
	}

	rc = msm_sensor_get_dt_csi_data(of_node, &sensordata->csi_lane_params);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SENSOR_INFO;
	}

	rc = msm_camera_get_dt_vreg_data(of_node,
			&sensordata->power_info.cam_vreg,
			&sensordata->power_info.num_vreg);
	if (rc < 0)
		goto FREE_CSI;

	rc = msm_camera_get_dt_power_setting_data(of_node,
			sensordata->power_info.cam_vreg,
			sensordata->power_info.num_vreg,
			&sensordata->power_info);


	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_VREG;
	}


	rc = msm_camera_get_power_settimgs_from_sensor_lib(
			&sensordata->power_info,
			&s_ctrl->power_setting_array);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_VREG;
	}

	sensordata->power_info.gpio_conf = kzalloc(
			sizeof(struct msm_camera_gpio_conf), GFP_KERNEL);
	if (!sensordata->power_info.gpio_conf) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto FREE_PS;
	}
	gconf = sensordata->power_info.gpio_conf;

	gpio_array_size = of_gpio_count(of_node);
	CDBG("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size) {
		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_CONF;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			CDBG("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_CONF;
		}

		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_REQ_TBL;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto FREE_GPIO_SET_TBL;
		}
	}
	rc = msm_sensor_get_dt_actuator_data(of_node,
					     &sensordata->actuator_info);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_GPIO_PIN_TBL;
	}

	sensordata->slave_info = kzalloc(sizeof(struct msm_camera_slave_info),
		GFP_KERNEL);
	if (!sensordata->slave_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto FREE_ACTUATOR_INFO;
	}

	rc = of_property_read_u32_array(of_node, "qcom,slave-id",
		id_info, 3);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto FREE_SLAVE_INFO;
	}

	sensordata->slave_info->sensor_slave_addr = id_info[0];
	sensordata->slave_info->sensor_id_reg_addr = id_info[1];
	sensordata->slave_info->sensor_id = id_info[2];
	CDBG("%s:%d slave addr %x sensor reg %x id %x\n", __func__, __LINE__,
		sensordata->slave_info->sensor_slave_addr,
		sensordata->slave_info->sensor_id_reg_addr,
		sensordata->slave_info->sensor_id);

	/*Optional property, don't return error if absent */
	ret = of_property_read_string(of_node, "qcom,vdd-cx-name",
		&sensordata->misc_regulator);
	CDBG("%s qcom,misc_regulator %s, rc %d\n", __func__,
		 sensordata->misc_regulator, ret);

	kfree(gpio_array);

	return rc;

FREE_SLAVE_INFO:
	kfree(s_ctrl->sensordata->slave_info);
FREE_ACTUATOR_INFO:
	kfree(s_ctrl->sensordata->actuator_info);
FREE_GPIO_PIN_TBL:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
FREE_GPIO_SET_TBL:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_set_tbl);
FREE_GPIO_REQ_TBL:
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
FREE_GPIO_CONF:
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
FREE_PS:
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->power_info.power_down_setting);
FREE_VREG:
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
FREE_CSI:
	kfree(s_ctrl->sensordata->csi_lane_params);
FREE_SENSOR_INFO:
	kfree(s_ctrl->sensordata->sensor_info);
FREE_SENSORDATA:
	kfree(s_ctrl->sensordata);
	kfree(gpio_array);
	return rc;
}

static void msm_sensor_misc_regulator(
	struct msm_sensor_ctrl_t *sctrl, uint32_t enable)
{
	int32_t rc = 0;
	if (enable) {
		sctrl->misc_regulator = (void *)rpm_regulator_get(
			&sctrl->pdev->dev, sctrl->sensordata->misc_regulator);
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(sctrl->misc_regulator,
				RPM_REGULATOR_MODE_HPM);
			if (rc < 0) {
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
				rpm_regulator_put(sctrl->misc_regulator);
			}
		} else {
			pr_err("%s: Failed to vote for rpm regulator on %s: %d\n",
				__func__,
				sctrl->sensordata->misc_regulator, rc);
		}
	} else {
		if (sctrl->misc_regulator) {
			rc = rpm_regulator_set_mode(
				(struct rpm_regulator *)sctrl->misc_regulator,
				RPM_REGULATOR_MODE_AUTO);
			if (rc < 0)
				pr_err("%s: Failed to set for rpm regulator on %s: %d\n",
					__func__,
					sctrl->sensordata->misc_regulator, rc);
			rpm_regulator_put(sctrl->misc_regulator);
		}
	}
}

int32_t msm_sensor_free_sensor_data(struct msm_sensor_ctrl_t *s_ctrl)
{
	if (!s_ctrl->pdev && !s_ctrl->sensor_i2c_client->client)
		return 0;
	kfree(s_ctrl->sensordata->slave_info);
	kfree(s_ctrl->sensordata->cam_slave_info);
	kfree(s_ctrl->sensordata->actuator_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->gpio_num_info);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_set_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf->cam_gpio_req_tbl);
	kfree(s_ctrl->sensordata->power_info.gpio_conf);
	kfree(s_ctrl->sensordata->power_info.cam_vreg);
	kfree(s_ctrl->sensordata->power_info.power_setting);
	kfree(s_ctrl->sensordata->csi_lane_params);
	kfree(s_ctrl->sensordata->sensor_info);
	kfree(s_ctrl->sensordata->power_info.clk_info);
	kfree(s_ctrl->sensordata);
	return 0;
}

static struct msm_cam_clk_info cam_8960_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_clk", 24000000},
};

static struct msm_cam_clk_info cam_8610_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};

static struct msm_cam_clk_info cam_8974_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_src_clk", 24000000},
	[SENSOR_CAM_CLK] = {"cam_clk", 0},
};
#if defined(CONFIG_IMX135_GBAO_LC898122) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_Z5S)
unsigned long otp_duration = HZ/1000;
#endif
/* ZTEMT: Jinghongliang Add for Manual AF Mode ----Start*/
extern void ZtemtMoveFocus(unsigned short reg_addr, unsigned char write_data_8);
/* ZTEMT: Jinghongliang Add for Manual AF Mode ----End*/

#if defined(CONFIG_IMX135_GBAO)  || defined(CONFIG_IMX135_Z5S)
extern void	SetH1cMod( unsigned char	UcSetNum );
extern void RegReadA(unsigned short reg_addr, unsigned char *read_data_8);
extern void RegWriteA(unsigned short reg_addr, unsigned char write_data_8);
extern void RamReadA(unsigned short ram_addr, void *read_data_16);
extern void RamWriteA(unsigned short ram_addr, unsigned short write_data_16);
extern void RamRead32A(unsigned short ram_addr, void *read_data_32);
extern void RamWrite32A(unsigned short ram_addr, unsigned long write_data_32);

extern unsigned char RtnCen(unsigned char	UcCmdPar);
extern void SetPanTiltMode(unsigned char UcPnTmod);

extern void OisEna(void);
extern void	IniSet( void );
extern void	SrvCon( unsigned char	UcDirSel, unsigned char	UcSwcCon );
extern void	S2cPro( unsigned char uc_mode );
extern void msm_ois_init_cci(void);
extern void msm_ois_release_cci(void);

int ois_init_flag_up=0;
                   
unsigned char read_otp_ready_flag=0;

/* 16bits RAM */
unsigned short  hall_offset_x=0; 			   
unsigned short  hall_offset_y=0; 
unsigned short  hall_bias_x=0; 
unsigned short  hall_bias_y=0; 
unsigned short  hall_ad_offset_x=0; 
unsigned short  hall_ad_offset_y=0; 
unsigned short  loop_gain_x=0; 
unsigned short  loop_gain_y=0; 

/* 8bits Register */
unsigned char gyro_offset_x_msb=0;
unsigned char gyro_offset_x_lsb=0;			   
unsigned char gyro_offset_y_msb=0;
unsigned char gyro_offset_y_lsb=0;

/* 32bits RAM */ 
unsigned long  gyro_gain_x=0;
unsigned char  gyro_gain_x_31_24=0;
unsigned char  gyro_gain_x_23_16=0;
unsigned char  gyro_gain_x_15_8=0;
unsigned char  gyro_gain_x_7_0=0;
unsigned long  gyro_gain_y=0;
unsigned char  gyro_gain_y_31_24=0;
unsigned char  gyro_gain_y_23_16=0;
unsigned char  gyro_gain_y_15_8=0;
unsigned char  gyro_gain_y_7_0=0;

/* 8bits Register */
unsigned char osc_value=1;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
unsigned short af_start_value = 0;
unsigned short af_infinity_value = 0;
unsigned short af_macro_value = 0;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#define OTP_PAGE_ADDR			0x3B02
#define	OTP_READ_MODE_ADDR		0x3B00
#define	OTP_READ_READY_ADDR		0x3B01
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
#define AF_START_CURRENT        0x3B04
#define AF_START_INFINITY       0x3B06
#define AF_START_MACRO          0X3B08
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#define HALL_OFFSET_X_ADDR		0x3B14
#define HALL_OFFSET_Y_ADDR		0x3B16
#define HALL_BIAS_X_ADDR		0x3B18
#define HALL_BIAS_Y_ADDR		0x3B1A
#define HALL_AD_OFFSET_X_ADDR	0x3B1C
#define HALL_AD_OFFSET_Y_ADDR	0x3B1E
#define LOOP_GAIN_X_ADDR		0x3B20
#define LOOP_GAIN_Y_ADDR		0x3B22

#define	GYRO_OFFSET_X_MSB_ADDR	0x3B28
#define	GYRO_OFFSET_X_LSB_ADDR	0x3B29
#define	GYRO_OFFSET_Y_MSB_ADDR	0x3B2A
#define	GYRO_OFFSET_Y_LSB_ADDR	0x3B2B

#define GYRO_GAIN_X_31_24_ADDR	0x3B34
#define GYRO_GAIN_X_23_16_ADDR	0x3B35
#define GYRO_GAIN_X_15_8_ADDR	0x3B36
#define GYRO_GAIN_X_7_0_ADDR	0x3B37
#define GYRO_GAIN_Y_31_24_ADDR	0x3B38
#define GYRO_GAIN_Y_23_16_ADDR	0x3B39
#define GYRO_GAIN_Y_15_8_ADDR	0x3B3A
#define GYRO_GAIN_Y_7_0_ADDR	0x3B3B

#define OSC_VALUE_ADDR			0x3B2C

//add code for ois version D,just need for old module
#if 0
unsigned short  read_loop_gain_x=0;
unsigned short  read_loop_gain_y=0;
unsigned long  read_gyro_gain_x =0;
unsigned long  read_loop_gain_x_multipy =0;
unsigned long  read_loop_gain_y_multipy =0;

unsigned short  temp_loop_gain_x=0;
unsigned short  temp_loop_gain_y=0;

unsigned short UsRltVal1=0;
unsigned short UsRltVal2=0;
#endif
//end code for ois version D,just need for old module
static void imx135_ois_otp(struct work_struct *work)
{
	struct msm_sensor_ctrl_t *s_ctrl = container_of(to_delayed_work(work),
					struct msm_sensor_ctrl_t, zte_otp_worker);
	int rc;
	uint16_t page_number  = 14;
	int32_t count = 0;
	mutex_lock(&s_ctrl->zte_otp_mutex);
	msm_ois_init_cci();
	//printk("sss e\n");
	IniSet();
	//printk("sss x\n");
	do{
		CDBG_OIS("<ZTEMT_CAM>%s, page number is %d\n",__func__,page_number);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
					page_number, MSM_CAMERA_I2C_BYTE_DATA);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
					0x01, MSM_CAMERA_I2C_BYTE_DATA);
		mdelay(10);
		for(count = 0;count < 10;count++){
			/* read the OTP ready flag */
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
					(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);
			
			if((read_otp_ready_flag & 0x01) == 0x01){
			/* check the correct page */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,OSC_VALUE_ADDR,
				     (uint16_t *)&osc_value, MSM_CAMERA_I2C_BYTE_DATA);
				CDBG_OIS("osc_value = 0x%x,count = %d\n",osc_value,count);
				break;
			}
			mdelay(10);
		}
		page_number = page_number -1;
	}while(osc_value == 0x00 && (page_number > 11));
	if ((read_otp_ready_flag & 0x01) == 0x01){
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,HALL_OFFSET_X_ADDR,
				(uint16_t *)&hall_offset_x, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("hall_offset_x = 0x%x\n",hall_offset_x);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,HALL_OFFSET_Y_ADDR,
				(uint16_t *)&hall_offset_y, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("hall_offset_y = 0x%x\n",hall_offset_y);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,HALL_BIAS_X_ADDR,
				(uint16_t *)&hall_bias_x, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("hall_bias_x = 0x%x\n",hall_bias_x);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,HALL_BIAS_Y_ADDR,
				(uint16_t *)&hall_bias_y, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("hall_bias_y = 0x%x\n",hall_bias_y);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,HALL_AD_OFFSET_X_ADDR,
				(uint16_t *)&hall_ad_offset_x, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("hall_ad_offset_x = 0x%x\n",hall_ad_offset_x);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,HALL_AD_OFFSET_Y_ADDR,
				(uint16_t *)&hall_ad_offset_y, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("hall_ad_offset_y = 0x%x\n",hall_ad_offset_y);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,LOOP_GAIN_X_ADDR,
				(uint16_t *)&loop_gain_x, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("loop_gain_x = 0x%x\n",loop_gain_x);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,LOOP_GAIN_Y_ADDR,
				(uint16_t *)&loop_gain_y, MSM_CAMERA_I2C_WORD_DATA);
		CDBG_OIS("loop_gain_y = 0x%x\n",loop_gain_y);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_OFFSET_X_MSB_ADDR,
				(uint16_t *)&gyro_offset_x_msb, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_offset_x_msb = 0x%x\n",gyro_offset_x_msb);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_OFFSET_X_LSB_ADDR,
				(uint16_t *)&gyro_offset_x_lsb, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_offset_x_lsb = 0x%x\n",gyro_offset_x_lsb);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_OFFSET_Y_MSB_ADDR,
				(uint16_t *)&gyro_offset_y_msb, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_offset_y_msb = 0x%x\n",gyro_offset_y_msb);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_OFFSET_Y_LSB_ADDR,
				(uint16_t *)&gyro_offset_y_lsb, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_offset_y_lsb = 0x%x\n",gyro_offset_y_lsb);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,OSC_VALUE_ADDR,
				(uint16_t *)&osc_value, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("osc_value = 0x%x\n",osc_value);

	}else{
		printk("OIS OTP Read OSC_VALUE Failed!\n");
	}

	page_number = 14;
	read_otp_ready_flag = 0;
	do{
		CDBG_OIS("<ZTEMT_CAM>%s, page_Gyro_gain number is %d\n",__func__,page_number);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
					page_number, MSM_CAMERA_I2C_BYTE_DATA);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
					0x01, MSM_CAMERA_I2C_BYTE_DATA);
		mdelay(10);
		for(count = 0;count < 10;count++){
			/* read the OTP ready flag */
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
					(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);
			
			if((read_otp_ready_flag & 0x01) == 0x01){
				/* check the correct page */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						s_ctrl->sensor_i2c_client,GYRO_GAIN_X_31_24_ADDR,
						(uint16_t *)&gyro_gain_x_31_24, MSM_CAMERA_I2C_BYTE_DATA);
				CDBG_OIS("gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24);
				break;
			}
		mdelay(10);
		}
		page_number = page_number -1;
	}while(gyro_gain_x_31_24 == 0x00 && (page_number > 11));

	if ((read_otp_ready_flag & 0x01) == 0x01){
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_X_31_24_ADDR,
				(uint16_t *)&gyro_gain_x_31_24, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_X_23_16_ADDR,
				(uint16_t *)&gyro_gain_x_23_16, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_x_23_16 = 0x%x\n",gyro_gain_x_23_16);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_X_15_8_ADDR,
				(uint16_t *)&gyro_gain_x_15_8, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_x_15_8 = 0x%x\n",gyro_gain_x_15_8);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_X_7_0_ADDR,
				(uint16_t *)&gyro_gain_x_7_0, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_x_7_0 = 0x%x\n",gyro_gain_x_7_0);
		
		gyro_gain_x = (gyro_gain_x_31_24 <<24) | (gyro_gain_x_23_16 <<16)|(gyro_gain_x_15_8<<8)|gyro_gain_x_7_0;

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_31_24_ADDR,
				(uint16_t *)&gyro_gain_y_31_24, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_y_31_24 = 0x%x\n",gyro_gain_y_31_24);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_23_16_ADDR,
				(uint16_t *)&gyro_gain_y_23_16, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_y_23_16 = 0x%x\n",gyro_gain_y_23_16);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_15_8_ADDR,
				(uint16_t *)&gyro_gain_y_15_8, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_y_15_8 = 0x%x\n",gyro_gain_y_15_8);
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,GYRO_GAIN_Y_7_0_ADDR,
				(uint16_t *)&gyro_gain_y_7_0, MSM_CAMERA_I2C_BYTE_DATA);
		CDBG_OIS("gyro_gain_y_7_0 = 0x%x\n",gyro_gain_y_7_0);
		
		gyro_gain_y = (gyro_gain_y_31_24 <<24) | (gyro_gain_y_23_16 <<16)|(gyro_gain_y_15_8<<8)|gyro_gain_y_7_0;
	}else{
		printk("OIS OTP Read GYRO GAIN Failed!\n");
	}
	RamWriteA(0x1114 ,hall_offset_x );
	RamWriteA(0x1116 ,hall_offset_y);
	RamWriteA(0x1115 ,hall_bias_x );
	RamWriteA(0x1117 ,hall_bias_y );
	RamWriteA(0x1102 ,hall_ad_offset_x );
	RamWriteA(0x1105 ,hall_ad_offset_y );
	RamWriteA(0x132A ,loop_gain_x );
	RamWriteA(0x136A ,loop_gain_y );
	RegWriteA(0x03A0,gyro_offset_x_msb);
	RegWriteA(0x03A1,gyro_offset_x_lsb);
	RegWriteA(0x03A2,gyro_offset_y_msb);
	RegWriteA(0x03A3,gyro_offset_y_lsb);
	RamWrite32A(0x1828,gyro_gain_x);
	RamWrite32A(0x1928,gyro_gain_y);
	RegWriteA(0x0264,osc_value);
	//add code for ois version D,just need for old module
#if 0
	RamReadA(0x132A ,&read_loop_gain_x );
	RamReadA(0x136A ,&read_loop_gain_y );
	read_loop_gain_x_multipy = read_loop_gain_x;
	read_loop_gain_x_multipy = read_loop_gain_x_multipy *56 / 100;   
	read_loop_gain_y_multipy = read_loop_gain_y;
	read_loop_gain_y_multipy = read_loop_gain_y_multipy  *56 / 100;
	temp_loop_gain_x= read_loop_gain_x_multipy & 0xffff;
	temp_loop_gain_y= read_loop_gain_y_multipy & 0xffff;
	RamWriteA(0x132A ,temp_loop_gain_x );
	RamWriteA(0x136A ,temp_loop_gain_y );
	RegWriteA(0x011A,0x01);
	RamReadA(0x1828, &UsRltVal1);
	RamReadA(0x1928, &UsRltVal2);
	RegWriteA(0x011A,0x00);
	if (UsRltVal1 > 0x5998 || UsRltVal1 <0x3332)
		RamWrite32A(0x1828 ,0x3f0ccccd);
	if (UsRltVal2 > 0xcccd || UsRltVal2 <0xa667)
		RamWrite32A(0x1928 ,0xbf0ccccd);
#endif
 //end code for ois version D,just need for old module
	RtnCen(0x00);
	SetPanTiltMode(1);
	OisEna();
	mutex_unlock(&s_ctrl->zte_otp_mutex);
}
#endif
#ifdef CONFIG_IMX135_GBAO_LC898122
extern void read_ois_byte_data_lc898122(unsigned short reg_addr, unsigned char *read_data_8);
extern void read_ois_word_data_lc898122(unsigned short reg_addr, uint16_t *read_data_16);

extern void	SetH1cMod_lc898122( unsigned char	UcSetNum );
extern void RegReadA_lc898122(unsigned short reg_addr, unsigned char *read_data_8);
extern void RegWriteA_lc898122(unsigned short reg_addr, unsigned char write_data_8);
extern void RamReadA_lc898122(unsigned short ram_addr, void *read_data_16);
extern void RamWriteA_lc898122(unsigned short ram_addr, unsigned short write_data_16);
extern void RamRead32A_lc898122(unsigned short ram_addr, void *read_data_32);
extern void RamWrite32A_lc898122(unsigned short ram_addr, unsigned long write_data_32);

extern unsigned char RtnCen_lc898122(unsigned char	UcCmdPar);
extern void SetPanTiltMode_lc898122(unsigned char UcPnTmod);

extern void OisEna_lc898122(void);
extern void	IniSet_lc898122( void );
extern void	SrvCon_lc898122( unsigned char	UcDirSel, unsigned char	UcSwcCon );
extern void	S2cPro_lc898122( unsigned char uc_mode );
extern void msm_ois_init_cci_lc898122(void);
extern void msm_ois_release_cci_lc898122(void);

extern void RamAccFixMod_lc898122( unsigned char UcAccMod );
extern void IniSetAf_lc898122( void );
extern void	SetH1cMod_lc898122( unsigned char	UcSetNum );

int ois_init_flag_up_lc898122=0;
                   
unsigned char read_otp_ready_flag_lc898122=0;

/* 16bits RAM */
unsigned short  hall_offset_x_lc898122=0; 			   
unsigned short  hall_offset_y_lc898122=0; 
unsigned short  hall_bias_x_lc898122=0; 
unsigned short  hall_bias_y_lc898122=0; 
unsigned short  hall_ad_offset_x_lc898122=0; 
unsigned short  hall_ad_offset_y_lc898122=0; 
unsigned short  loop_gain_x_lc898122=0; 
unsigned short  loop_gain_y_lc898122=0; 

/* 8bits Register */
unsigned char gyro_offset_x_msb_lc898122=0;
unsigned char gyro_offset_x_lsb_lc898122=0;			   
unsigned char gyro_offset_y_msb_lc898122=0;
unsigned char gyro_offset_y_lsb_lc898122=0;

/* 32bits RAM */ 
unsigned long  gyro_gain_x_lc898122=0;
unsigned char  gyro_gain_x_31_24_lc898122=0;
unsigned char  gyro_gain_x_23_16_lc898122=0;
unsigned char  gyro_gain_x_15_8_lc898122=0;
unsigned char  gyro_gain_x_7_0_lc898122=0;
unsigned long  gyro_gain_y_lc898122=0;
unsigned char  gyro_gain_y_31_24_lc898122=0;
unsigned char  gyro_gain_y_23_16_lc898122=0;
unsigned char  gyro_gain_y_15_8_lc898122=0;
unsigned char  gyro_gain_y_7_0_lc898122=0;

/* 8bits Register */
unsigned char osc_value_lc898122=1;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
unsigned short af_start_value_lc898122 = 0;
unsigned short af_infinity_value_lc898122 = 0;
unsigned short af_macro_value_lc898122 = 0;
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/

#define OTP_PAGE_ADDR_LC898122			0x3B02
#define	OTP_READ_MODE_ADDR_LC898122		0x3B00
#define	OTP_READ_READY_ADDR_LC898122		0x3B01
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
#define AF_START_CURRENT_LC898122        0x20
#define AF_START_INFINITY_LC898122       0x22
#define AF_START_MACRO_LC898122          0x24
/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/


#define HALL_OFFSET_X_ADDR_LC898122		0x30
#define HALL_OFFSET_Y_ADDR_LC898122		0x32
#define HALL_BIAS_X_ADDR_LC898122		0x34
#define HALL_BIAS_Y_ADDR_LC898122		0x36
#define HALL_AD_OFFSET_X_ADDR_LC898122	0x38
#define HALL_AD_OFFSET_Y_ADDR_LC898122	0x3A
#define LOOP_GAIN_X_ADDR_LC898122		0x3C
#define LOOP_GAIN_Y_ADDR_LC898122		0x3E

#define	GYRO_OFFSET_X_MSB_ADDR_LC898122	0x44
#define	GYRO_OFFSET_X_LSB_ADDR_LC898122	0x45
#define	GYRO_OFFSET_Y_MSB_ADDR_LC898122	0x46
#define	GYRO_OFFSET_Y_LSB_ADDR_LC898122	0x47

#define GYRO_GAIN_X_31_24_ADDR_LC898122	0x49
#define GYRO_GAIN_X_23_16_ADDR_LC898122	0x4A
#define GYRO_GAIN_X_15_8_ADDR_LC898122	0x4B
#define GYRO_GAIN_X_7_0_ADDR_LC898122	0x4C
#define GYRO_GAIN_Y_31_24_ADDR_LC898122	0x4D
#define GYRO_GAIN_Y_23_16_ADDR_LC898122	0x4E
#define GYRO_GAIN_Y_15_8_ADDR_LC898122	0x4F
#define GYRO_GAIN_Y_7_0_ADDR_LC898122	0x50

#define OSC_VALUE_ADDR_LC898122			0x48

int ois_otp_lc898122_flag = 0;

static void imx135_ois_otp_lc898122(struct work_struct *work)
{
	struct msm_sensor_ctrl_t *s_ctrl = container_of(to_delayed_work(work),
					struct msm_sensor_ctrl_t, zte_otp_worker);

	mutex_lock(&s_ctrl->zte_otp_mutex);
	msm_ois_init_cci_lc898122();
	
	IniSetAf_lc898122();
	IniSet_lc898122();

	if (ois_otp_lc898122_flag == 0)
	{
		read_ois_word_data_lc898122(HALL_OFFSET_X_ADDR_LC898122,(uint16_t *)&hall_offset_x_lc898122);
	       CDBG_OIS("hall_offset_x = 0x%x\n",hall_offset_x_lc898122);
		read_ois_word_data_lc898122(HALL_OFFSET_Y_ADDR_LC898122,(uint16_t *)&hall_offset_y_lc898122);
	       CDBG_OIS("hall_offset_y = 0x%x\n",hall_offset_y_lc898122);
		read_ois_word_data_lc898122(HALL_BIAS_X_ADDR_LC898122,(uint16_t *)&hall_bias_x_lc898122);
	       CDBG_OIS("hall_bias_x = 0x%x\n",hall_bias_x_lc898122);
		read_ois_word_data_lc898122(HALL_BIAS_Y_ADDR_LC898122,(uint16_t *)&hall_bias_y_lc898122);
	       CDBG_OIS("hall_bias_y = 0x%x\n",hall_bias_y_lc898122);
		read_ois_word_data_lc898122(HALL_AD_OFFSET_X_ADDR_LC898122,(uint16_t *)&hall_ad_offset_x_lc898122);
	       CDBG_OIS("hall_ad_offset_x = 0x%x\n",hall_ad_offset_x_lc898122);
		read_ois_word_data_lc898122(HALL_AD_OFFSET_Y_ADDR_LC898122,(uint16_t *)&hall_ad_offset_y_lc898122);
	       CDBG_OIS("hall_ad_offset_y = 0x%x\n",hall_ad_offset_y_lc898122);
		read_ois_word_data_lc898122(LOOP_GAIN_X_ADDR_LC898122,(uint16_t *)&loop_gain_x_lc898122);
	       CDBG_OIS("loop_gain_x = 0x%x\n",loop_gain_x_lc898122);
		read_ois_word_data_lc898122(LOOP_GAIN_Y_ADDR_LC898122,(uint16_t *)&loop_gain_y_lc898122);
		CDBG_OIS("loop_gain_y = 0x%x\n",loop_gain_y_lc898122);

		
	       read_ois_byte_data_lc898122(GYRO_OFFSET_X_MSB_ADDR_LC898122,(unsigned char *)&gyro_offset_x_msb_lc898122);
	       CDBG_OIS("gyro_offset_x_msb = 0x%x\n",gyro_offset_x_msb_lc898122);
		read_ois_byte_data_lc898122(GYRO_OFFSET_X_LSB_ADDR_LC898122,(unsigned char *)&gyro_offset_x_lsb_lc898122);
	       CDBG_OIS("gyro_offset_x_lsb = 0x%x\n",gyro_offset_x_lsb_lc898122);
		read_ois_byte_data_lc898122(GYRO_OFFSET_Y_MSB_ADDR_LC898122,(unsigned char *)&gyro_offset_y_msb_lc898122);
	       CDBG_OIS("gyro_offset_y_msb = 0x%x\n",gyro_offset_y_msb_lc898122);
		read_ois_byte_data_lc898122(GYRO_OFFSET_Y_LSB_ADDR_LC898122,(unsigned char *)&gyro_offset_y_lsb_lc898122);
	       CDBG_OIS("gyro_offset_y_lsb = 0x%x\n",gyro_offset_y_lsb_lc898122);
		read_ois_byte_data_lc898122(OSC_VALUE_ADDR_LC898122,(unsigned char *)&osc_value_lc898122);
	       CDBG_OIS("osc_value = 0x%x\n",osc_value_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_31_24_ADDR_LC898122,(unsigned char *)&gyro_gain_x_31_24_lc898122);
	       CDBG_OIS("gyro_gain_x_31_24 = 0x%x\n",gyro_gain_x_31_24_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_23_16_ADDR_LC898122,(unsigned char *)&gyro_gain_x_23_16_lc898122);
	       CDBG_OIS("gyro_gain_x_23_16 = 0x%x\n",gyro_gain_x_23_16_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_15_8_ADDR_LC898122,(unsigned char *)&gyro_gain_x_15_8_lc898122);
	       CDBG_OIS("gyro_gain_x_15_8 = 0x%x\n",gyro_gain_x_15_8_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_X_7_0_ADDR_LC898122,(unsigned char *)&gyro_gain_x_7_0_lc898122);
	       CDBG_OIS("gyro_gain_x_7_0 = 0x%x\n",gyro_gain_x_7_0_lc898122);

		gyro_gain_x_lc898122 = (gyro_gain_x_31_24_lc898122 <<24) | (gyro_gain_x_23_16_lc898122 <<16)|(gyro_gain_x_15_8_lc898122<<8)|gyro_gain_x_7_0_lc898122;

		
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_31_24_ADDR_LC898122,(unsigned char *)&gyro_gain_y_31_24_lc898122);
	       CDBG_OIS("gyro_gain_y_31_24 = 0x%x\n",gyro_gain_y_31_24_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_23_16_ADDR_LC898122,(unsigned char *)&gyro_gain_y_23_16_lc898122);
	       CDBG_OIS("gyro_gain_y_23_16 = 0x%x\n",gyro_gain_y_23_16_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_15_8_ADDR_LC898122,(unsigned char *)&gyro_gain_y_15_8_lc898122);
	       CDBG_OIS("gyro_gain_y_15_8 = 0x%x\n",gyro_gain_y_15_8_lc898122);
		read_ois_byte_data_lc898122(GYRO_GAIN_Y_7_0_ADDR_LC898122,(unsigned char *)&gyro_gain_y_7_0_lc898122);
		CDBG_OIS("gyro_gain_y_7_0 = 0x%x\n",gyro_gain_y_7_0_lc898122);
			
		gyro_gain_y_lc898122 = (gyro_gain_y_31_24_lc898122 <<24) | (gyro_gain_y_23_16_lc898122 <<16)|(gyro_gain_y_15_8_lc898122<<8)|gyro_gain_y_7_0_lc898122;

		ois_otp_lc898122_flag = 1;
		
	}
	
	RamAccFixMod_lc898122(0x01);
	
	RamWriteA_lc898122(0x1479 ,hall_offset_x_lc898122 );
	RamWriteA_lc898122(0x14F9 ,hall_offset_y_lc898122);
	RamWriteA_lc898122(0x147A ,hall_bias_x_lc898122 );
	RamWriteA_lc898122(0x14FA ,hall_bias_y_lc898122 );
	RamWriteA_lc898122(0x1450 ,hall_ad_offset_x_lc898122 );
	RamWriteA_lc898122(0x14D0 ,hall_ad_offset_y_lc898122 );
	RamWriteA_lc898122(0x10D3 ,loop_gain_x_lc898122 );
	RamWriteA_lc898122(0x11D3 ,loop_gain_y_lc898122 );

	RamAccFixMod_lc898122(0x00);
	
	RegWriteA_lc898122(0x02A0,gyro_offset_x_msb_lc898122);
	RegWriteA_lc898122(0x02A1,gyro_offset_x_lsb_lc898122);
	RegWriteA_lc898122(0x02A2,gyro_offset_y_msb_lc898122);
	RegWriteA_lc898122(0x02A3,gyro_offset_y_lsb_lc898122);
	
	RamWrite32A_lc898122(0x1020,gyro_gain_x_lc898122);
	RamWrite32A_lc898122(0x1120,gyro_gain_y_lc898122);
	
	RegWriteA_lc898122(0x0257,osc_value_lc898122);

       RamWriteA_lc898122(0x0304 ,af_infinity_value_lc898122 );
	   
	RtnCen_lc898122(0x00);
	SetPanTiltMode_lc898122(1);
	OisEna_lc898122();

	mutex_unlock(&s_ctrl->zte_otp_mutex);
}
#endif


#ifdef CONFIG_IMX135_Z5S
int ois_init_flag_down_z5s=0;
#endif
#ifdef CONFIG_IMX135_GBAO
int ois_init_flag_down=0;
#endif
#ifdef CONFIG_IMX135_GBAO_LC898122
int ois_init_flag_down_lc898122=0;
#endif

#if defined(CONFIG_IMX135_Z5S_069) ||defined(CONFIG_IMX135_Z5S) || defined(CONFIG_IMX135) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122)
int temp_i2c_data_type = 0 ;
static int RegRead8byte_adaptive(uint16_t reg_addr, struct msm_sensor_ctrl_t *s_ctrl)
{
	uint8_t data[8];
	int32_t rc=0;
	enum msm_camera_i2c_reg_addr_type addr_type;
	addr_type = s_ctrl->sensor_i2c_client->addr_type;
	memset(data, 0x00, 8);
	s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	rc =  s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq
	 (
		s_ctrl->sensor_i2c_client,
		reg_addr, &data[0],
		8);
	if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
	}
	s_ctrl->sensor_i2c_client->addr_type = addr_type;
	CDBG("sss %x %x %x %x\n", data[0],data[1],data[2],data[3]);
	return rc;
}
static void zte_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
#ifdef CONFIG_IMX135_Z5S
	if (strcmp(s_ctrl->sensordata->sensor_name,"imx135_z5s") == 0) {
		if (ois_init_flag_down_z5s==0)
		   	ois_init_flag_down_z5s=1;
		else {
			mutex_lock(&s_ctrl->zte_otp_mutex);
		       RtnCen(0x00);
			SrvCon(0x00,0);  
			SrvCon(0x01,0);  
			msm_ois_release_cci();
			mutex_unlock(&s_ctrl->zte_otp_mutex);
			if (s_ctrl->zte_otp_enable == true)
				cancel_delayed_work_sync(&s_ctrl->zte_otp_worker);
		}
	}
#endif
#ifdef CONFIG_IMX135_GBAO
	if (strcmp(s_ctrl->sensordata->sensor_name,"imx135_gbao") == 0) {
		if (ois_init_flag_down==0)
			ois_init_flag_down=1;
		else {
			mutex_lock(&s_ctrl->zte_otp_mutex);
			RtnCen(0x00);
			SrvCon(0x00,0);  
			SrvCon(0x01,0);  
			msm_ois_release_cci();
			mutex_unlock(&s_ctrl->zte_otp_mutex);
			if (s_ctrl->zte_otp_enable == true)
				cancel_delayed_work_sync(&s_ctrl->zte_otp_worker);
		}
	}
#endif
#ifdef CONFIG_IMX135_GBAO_LC898122
	if (strcmp(s_ctrl->sensordata->sensor_name,"imx135_gbao_lc898122") == 0) {
		if (ois_init_flag_down_lc898122==0)
			ois_init_flag_down_lc898122=1;
		else {
			mutex_lock(&s_ctrl->zte_otp_mutex);
			
			   RtnCen_lc898122(0x00);
			SrvCon_lc898122(0x00,0);  
			SrvCon_lc898122(0x01,0);  
			RegWriteA_lc898122(0x0304,0x00);
			   RegWriteA_lc898122(0x0305,0x00);
			
			msm_ois_release_cci_lc898122();
			mutex_unlock(&s_ctrl->zte_otp_mutex);
			if (s_ctrl->zte_otp_enable == true)
				cancel_delayed_work_sync(&s_ctrl->zte_otp_worker);
		}
	}
#endif

}


#endif

int msm_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_power_ctrl_t *power_info;
	enum msm_camera_device_type_t sensor_device_type;
	struct msm_camera_i2c_client *sensor_i2c_client;

	if (!s_ctrl) {
		pr_err("%s:%d failed: s_ctrl %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	sensor_device_type = s_ctrl->sensor_device_type;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;

	if (!power_info || !sensor_i2c_client) {
		pr_err("%s:%d failed: power_info %p sensor_i2c_client %p\n",
			__func__, __LINE__, power_info, sensor_i2c_client);
		return -EINVAL;
	}
	
#if defined(CONFIG_IMX135_Z5S_069) ||defined(CONFIG_IMX135_Z5S) || defined(CONFIG_IMX135) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122)
	zte_power_down(s_ctrl);
#endif
	return msm_camera_power_down(power_info, sensor_device_type,
		sensor_i2c_client);
}
int msm_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;
	struct msm_camera_power_ctrl_t *power_info;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;
#if defined(CONFIG_IMX135_GBAO) ||defined(CONFIG_IMX135_Z5S)
	int page_number = 14;
	int count = 0;
#endif
#if defined(CONFIG_IMX135_Z5S_069) ||defined(CONFIG_IMX135_Z5S) || defined(CONFIG_IMX135) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122)
	int temp_i2c_data_type = 0 ;
	unsigned char ois_version_flag_lc898122 =0; 
#endif

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	power_info = &s_ctrl->sensordata->power_info;
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!power_info || !sensor_i2c_client || !slave_info ||
		!sensor_name) {
		pr_err("%s:%d failed: %p %p %p %p\n",
			__func__, __LINE__, power_info,
			sensor_i2c_client, slave_info, sensor_name);
		return -EINVAL;
	}

	rc = msm_camera_power_up(power_info, s_ctrl->sensor_device_type,
		sensor_i2c_client);
	if (rc < 0)
		return rc;
	
#if defined(CONFIG_IMX135_Z5S_069) ||defined(CONFIG_IMX135_Z5S) || defined(CONFIG_IMX135) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122)
      if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_z5s", 32)) {
		s_ctrl->sensor_i2c_client->cci_client->sid = 0x1c >> 1;
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
			}
		}
		rc = RegRead8byte_adaptive(0x02, s_ctrl);
		if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
		}
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
		}
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	}
       if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_z5s_069", 32)) {
		s_ctrl->sensor_i2c_client->cci_client->sid = 0x18 >> 1;
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
			}
		}
		rc = RegRead8byte_adaptive(0x94, s_ctrl);
		if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
		}
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
		}
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	}
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135", 32)) {
		s_ctrl->sensor_i2c_client->cci_client->sid = 0x32 >> 1;
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
			}
		}
		rc = RegRead8byte_adaptive(0x91, s_ctrl);
		if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
		}
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
		}
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	}
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_gbao", 32)) {
		s_ctrl->sensor_i2c_client->cci_client->sid = 0x1c >> 1;
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
			}
		}
		rc = RegRead8byte_adaptive(0x91, s_ctrl);
		if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
		}
		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
		}
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	}

	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_gbao_lc898122", 32)) {
	
			  s_ctrl->sensor_i2c_client->cci_client->sid = 0x48 >> 1;
		temp_i2c_data_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;

		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
			}
		}
		
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
							 s_ctrl->sensor_i2c_client,0x027E,
					(uint16_t *)&ois_version_flag_lc898122, MSM_CAMERA_I2C_BYTE_DATA);
		
		   printk("ois_version_flag_lc898122 = 0x%x\n",ois_version_flag_lc898122);
		
		if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
		}
		if ( ois_version_flag_lc898122 != 0x93) {
				   rc = -1;
				pr_err("%s	ois_version_flag_lc898122 0x93 cci_init failed,rc =%d\n", __func__,rc);
				goto power_up_failed;
		}

		if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
		}
		
		s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
		s_ctrl->sensor_i2c_client->addr_type = temp_i2c_data_type;

	}
	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
				s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
			if (rc < 0) {
				pr_err("%s cci_init failed\n", __func__);
				goto power_up_failed;
			}
	}
		
#endif
	rc = msm_sensor_check_id(s_ctrl);
#ifdef CONFIG_IMX135_Z5S
	if (strcmp(sensor_name,"imx135_z5s") == 0) {
	    if (ois_init_flag_up==0) {
		   	ois_init_flag_up=1;
		/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
		if (s_ctrl->zte_otp_enable == true){
		do{
			CDBG_OIS("<ZTEMT_CAM>%s, read OTP page number is %d\n",__func__,page_number);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
					page_number, MSM_CAMERA_I2C_BYTE_DATA);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
					s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
					0x01, MSM_CAMERA_I2C_BYTE_DATA);
			mdelay(10);
			for(count = 0;count < 10;count++){
				/* read the OTP ready flag */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
					s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
					(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);

				if((read_otp_ready_flag & 0x01) == 0x01){
				/* check the correct page */
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,AF_START_CURRENT,
				     (uint16_t *)&af_start_value, MSM_CAMERA_I2C_WORD_DATA);
				CDBG_OIS("af_start_value = 0x%x,count = %d\n",af_start_value,count);
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,AF_START_INFINITY,
				     (uint16_t *)&af_infinity_value, MSM_CAMERA_I2C_WORD_DATA);
				CDBG_OIS("af_infinity_value = 0x%x,count = %d\n",af_infinity_value,count);
				rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				     s_ctrl->sensor_i2c_client,AF_START_MACRO,
				     (uint16_t *)&af_macro_value, MSM_CAMERA_I2C_WORD_DATA);
				CDBG_OIS("af_macro_value = 0x%x,count = %d\n",af_macro_value,count);
				break;
				}
				mdelay(10);
			}
			page_number = page_number -1;
		  }while(page_number > 11);
		}
		/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/
		} else {
			if (s_ctrl->zte_otp_enable == true)
				schedule_delayed_work(&s_ctrl->zte_otp_worker,
					otp_duration);
		}
			}
	#endif
	#ifdef CONFIG_IMX135_GBAO
	if (strcmp(sensor_name,"imx135_gbao") == 0) {
		if (ois_init_flag_up==0) {
		   	ois_init_flag_up=1;
			/*ZTEMT: Jinghongliang Add for Read AF OTP  ---Start*/
			if (s_ctrl->zte_otp_enable == true){
				do{
					CDBG_OIS("<ZTEMT_CAM>%s, read OTP page number is %d\n",__func__,page_number);
					rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
							s_ctrl->sensor_i2c_client,OTP_PAGE_ADDR,
							page_number, MSM_CAMERA_I2C_BYTE_DATA);

					rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
							s_ctrl->sensor_i2c_client,OTP_READ_MODE_ADDR,
							0x01, MSM_CAMERA_I2C_BYTE_DATA);
					mdelay(10);
					for(count = 0;count < 10;count++){
						/* read the OTP ready flag */
						rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
							s_ctrl->sensor_i2c_client,OTP_READ_READY_ADDR,
							(uint16_t *)&read_otp_ready_flag, MSM_CAMERA_I2C_BYTE_DATA);

						if((read_otp_ready_flag & 0x01) == 0x01){
						/* check the correct page */
						rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						     s_ctrl->sensor_i2c_client,AF_START_CURRENT,
						     (uint16_t *)&af_start_value, MSM_CAMERA_I2C_WORD_DATA);
						CDBG_OIS("af_start_value = 0x%x,count = %d\n",af_start_value,count);
						rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						     s_ctrl->sensor_i2c_client,AF_START_INFINITY,
						     (uint16_t *)&af_infinity_value, MSM_CAMERA_I2C_WORD_DATA);
						CDBG_OIS("af_infinity_value = 0x%x,count = %d\n",af_infinity_value,count);
						rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
						     s_ctrl->sensor_i2c_client,AF_START_MACRO,
						     (uint16_t *)&af_macro_value, MSM_CAMERA_I2C_WORD_DATA);
						CDBG_OIS("af_macro_value = 0x%x,count = %d\n",af_macro_value,count);
						break;
						}
						mdelay(10);
					}	
					if ((af_start_value > 0) && (af_infinity_value > 0) && (af_macro_value > 0))
						break;
					page_number = page_number -1;
				  }while(page_number > 11);
			}
		/*ZTEMT: Jinghongliang Add for Read AF OTP  ---End*/
		} else {
			if (s_ctrl->zte_otp_enable == true)
				schedule_delayed_work(&s_ctrl->zte_otp_worker,
					otp_duration);
		}
	      
	}
	#endif
	#ifdef CONFIG_IMX135_GBAO_LC898122
	if (strcmp(sensor_name,"imx135_gbao_lc898122") == 0) {
	    if (ois_init_flag_up_lc898122==0) {
		   	ois_init_flag_up_lc898122=1;
			/*ZTEMT: Add for Read AF OTP  ---Start*/
			if (s_ctrl->zte_otp_enable == true){

		       s_ctrl->sensor_i2c_client->cci_client->sid = 0xA0 >> 1;
			temp_i2c_data_type = s_ctrl->sensor_i2c_client->addr_type;
			s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			                     s_ctrl->sensor_i2c_client,AF_START_CURRENT_LC898122,
						(uint16_t *)&af_start_value_lc898122, MSM_CAMERA_I2C_WORD_DATA);
			
		       rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			                     s_ctrl->sensor_i2c_client,AF_START_INFINITY_LC898122,
						(uint16_t *)&af_infinity_value_lc898122, MSM_CAMERA_I2C_WORD_DATA);
				
		       rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			                     s_ctrl->sensor_i2c_client,AF_START_MACRO_LC898122,
						(uint16_t *)&af_macro_value_lc898122, MSM_CAMERA_I2C_WORD_DATA);
					   
			if (rc < 0) {
					pr_err("%s cci_init failed\n", __func__);
					//goto power_up_failed;
			}
			
			s_ctrl->sensor_i2c_client->cci_client->sid = s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;

			s_ctrl->sensor_i2c_client->addr_type = temp_i2c_data_type;
			}
			/*ZTEMT: Add for Read AF OTP  ---End*/
			} else {
				if (s_ctrl->zte_otp_enable == true)
					schedule_delayed_work(&s_ctrl->zte_otp_worker,
						otp_duration);
			}
	      
	}
	#endif
	if (rc < 0)
		msm_camera_power_down(power_info, s_ctrl->sensor_device_type,
					sensor_i2c_client);
	return rc;
#if defined(CONFIG_IMX135_Z5S_069) ||defined(CONFIG_IMX135_Z5S) || defined(CONFIG_IMX135) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122)
	power_up_failed:	
		msm_camera_power_down(power_info, s_ctrl->sensor_device_type,
			sensor_i2c_client);
	return -1;
#endif
}

int msm_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	sensor_i2c_client = s_ctrl->sensor_i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id %x:\n", __func__, chipid,
		slave_info->sensor_id);
	if (chipid != slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static struct msm_sensor_ctrl_t *get_sctrl(struct v4l2_subdev *sd)
{
	return container_of(container_of(sd, struct msm_sd_subdev, sd),
		struct msm_sensor_ctrl_t, msm_sd);
}

static void msm_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &s_ctrl->stop_setting);
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return;
}

static int msm_sensor_get_af_status(struct msm_sensor_ctrl_t *s_ctrl,
			void __user *argp)
{
	/* TO-DO: Need to set AF status register address and expected value
	We need to check the AF status in the sensor register and
	set the status in the *status variable accordingly*/
	return 0;
}

static long msm_sensor_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	void __user *argp = (void __user *)arg;
	if (!s_ctrl) {
		pr_err("%s s_ctrl NULL\n", __func__);
		return -EBADF;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG:
		return s_ctrl->func_tbl->sensor_config(s_ctrl, argp);
	case VIDIOC_MSM_SENSOR_GET_AF_STATUS:
		return msm_sensor_get_af_status(s_ctrl, argp);
	case VIDIOC_MSM_SENSOR_RELEASE:
		msm_sensor_stop_stream(s_ctrl);
		return 0;
	case MSM_SD_SHUTDOWN:
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

int msm_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
		int s_index = 0;
		if (copy_from_user(&sensor_slave_info,
				(void *)cdata->cfg.setting,
				sizeof(sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;
		p_ctrl = &s_ctrl->sensordata->power_info;

		/* Update power up sequence */
		size = sensor_slave_info.power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(*tmp) * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;


		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info.power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(sensor_slave_info.power_setting_array.
				power_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (s_index = 0; s_index <
			p_ctrl->power_setting_size; s_index++) {
			CDBG("%s i %d power up setting %d %d %ld %d\n",
				__func__,
				s_index,
				p_ctrl->power_setting[s_index].seq_type,
				p_ctrl->power_setting[s_index].seq_val,
				p_ctrl->power_setting[s_index].config_val,
				p_ctrl->power_setting[s_index].delay);
		}

		/* Update power down sequence */
		if (!sensor_slave_info.power_setting_array.power_down_setting ||
			0 == size) {
			pr_err("%s: Missing dedicated power down sequence\n",
				__func__);
			break;
		}
		size = sensor_slave_info.power_setting_array.size_down;

		if (p_ctrl->power_down_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(*tmp) * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_down_setting);
			p_ctrl->power_down_setting = tmp;
		}
		p_ctrl->power_down_setting_size = size;


		rc = copy_from_user(p_ctrl->power_down_setting, (void *)
			sensor_slave_info.power_setting_array.
			power_down_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(sensor_slave_info.power_setting_array.
				power_down_setting);
			rc = -EFAULT;
			break;
		}
		for (s_index = 0; s_index <
			p_ctrl->power_down_setting_size; s_index++) {
			CDBG("%s i %d power DOWN setting %d %d %ld %d\n",
				__func__,
				s_index,
				p_ctrl->power_down_setting[s_index].seq_type,
				p_ctrl->power_down_setting[s_index].seq_val,
				p_ctrl->power_down_setting[s_index].config_val,
				p_ctrl->power_down_setting[s_index].delay);
		}

		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_SLAVE_READ_I2C: {
		struct msm_camera_i2c_read_config read_config;
		uint16_t local_data = 0;
		uint16_t orig_slave_addr = 0, read_slave_addr = 0;
		if (copy_from_user(&read_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_read_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		read_slave_addr = read_config.slave_addr;
		CDBG("%s:CFG_SLAVE_READ_I2C:", __func__);
		CDBG("%s:slave_addr=0x%x reg_addr=0x%x, data_type=%d\n",
			__func__, read_config.slave_addr,
			read_config.reg_addr, read_config.data_type);
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				read_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				read_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				read_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client,
				read_config.reg_addr,
				&local_data, read_config.data_type);
		if (rc < 0) {
			pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
			break;
		}
		if (copy_to_user((void __user *)read_config.data,
			(void *)&local_data, sizeof(uint16_t))) {
			pr_err("%s:%d copy failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SLAVE_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_array_write_config write_config;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		uint16_t write_slave_addr = 0;
		uint16_t orig_slave_addr = 0;

		if (copy_from_user(&write_config,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_array_write_config))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:CFG_SLAVE_WRITE_I2C_ARRAY:", __func__);
		CDBG("%s:slave_addr=0x%x, array_size=%d\n", __func__,
			write_config.slave_addr,
			write_config.conf_array.size);

		if (!write_config.conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(write_config.conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting,
				(void *)(write_config.conf_array.reg_setting),
				write_config.conf_array.size *
				sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		write_config.conf_array.reg_setting = reg_setting;
		write_slave_addr = write_config.slave_addr;
		if (s_ctrl->sensor_i2c_client->cci_client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->cci_client->sid;
			s_ctrl->sensor_i2c_client->cci_client->sid =
				write_slave_addr >> 1;
		} else if (s_ctrl->sensor_i2c_client->client) {
			orig_slave_addr =
				s_ctrl->sensor_i2c_client->client->addr;
			s_ctrl->sensor_i2c_client->client->addr =
				write_slave_addr >> 1;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		CDBG("%s:orig_slave_addr=0x%x, new_slave_addr=0x%x",
				__func__, orig_slave_addr,
				write_slave_addr >> 1);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &(write_config.conf_array));
		if (s_ctrl->sensor_i2c_client->cci_client) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				orig_slave_addr;
		} else if (s_ctrl->sensor_i2c_client->client) {
			s_ctrl->sensor_i2c_client->client->addr =
				orig_slave_addr;
		} else {
			pr_err("%s: error: no i2c/cci client found.", __func__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		if (!conf_array.size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_up) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 1);

			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_POWER_DOWN:
		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}
		if (s_ctrl->func_tbl->sensor_power_down) {
			if (s_ctrl->sensordata->misc_regulator)
				msm_sensor_misc_regulator(s_ctrl, 0);

			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
			if (rc < 0) {
				pr_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		} else {
			rc = -EFAULT;
		}
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;

		if (!stop_setting->size) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting,
			stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	/* ZTEMT: Jinghongliang Add for Manual AF Mode ----Start */
	case CFG_SET_MANUAL_AF_ZTEMT: {
			int32_t value = 0;
			int32_t lens_position = 0;
			uint16_t MSB = 0;
			uint16_t LSB = 0;
			uint16_t addr = 0;
			uint16_t data = 0;
			if(copy_from_user(&value,
				(void *)cdata->cfg.setting,sizeof(int32_t))){
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
			break;
			}else{
				printk("<ZTEMT_CAM> Manual AF value = %d \n",value);
				if(value < 0 || value > 79){     /* if over total steps*/
					break;
				}
				
				if(value < 5){
					lens_position = 100+10*value;
				}else{
				    lens_position = 140 + 7*(value-4);
				}
				if(value == 79)
					lens_position = 900;   /* push the VCM to Macro position*/

				#ifdef CONFIG_IMX214
				MSB = ( lens_position & 0x0300 ) >> 8;
				LSB = lens_position & 0xFF;
				lens_position = lens_position | 0xF400;
				addr = (lens_position & 0xFF00) >> 8;
				data = lens_position & 0xFF;
				printk("<<<ZTEMT_JHL>>> addr = 0x%x, data = 0x%x\n",addr,data);
			    ZtemtMoveFocus(addr,data);
				#else
				MSB = ( lens_position & 0x0300 ) >> 8;
				LSB = lens_position & 0xFF;
				lens_position = lens_position | 0xF400;
				addr = (lens_position & 0xFF00) >> 8;
				data = lens_position & 0xFF;
				printk("<<<ZTEMT_JHL>>> This sensor not support Manual AF\n");
				#endif
			}
		break;
	  }
	/* ZTEMT: Jinghongliang Add for Manual AF Mode ----End */
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int msm_sensor_check_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc;

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0)
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
	return rc;
}

static int msm_sensor_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);
	mutex_lock(s_ctrl->msm_sensor_mutex);
	if (!on && s_ctrl->sensor_state == MSM_SENSOR_POWER_UP) {
		s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
	}
	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

static int msm_sensor_v4l2_enum_fmt(struct v4l2_subdev *sd,
	unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	struct msm_sensor_ctrl_t *s_ctrl = get_sctrl(sd);

	if ((unsigned int)index >= s_ctrl->sensor_v4l2_subdev_info_size)
		return -EINVAL;

	*code = s_ctrl->sensor_v4l2_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops msm_sensor_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops msm_sensor_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops msm_sensor_subdev_ops = {
	.core = &msm_sensor_subdev_core_ops,
	.video  = &msm_sensor_subdev_video_ops,
};

static struct msm_sensor_fn_t msm_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
#if defined(CONFIG_IMX135) || defined(CONFIG_IMX135_GBAO) || defined(CONFIG_IMX135_GBAO_LC898122) || defined(CONFIG_IMX135_Z5S) 
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
#endif

};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_write_conf_tbl = msm_camera_qup_i2c_write_conf_tbl,
};

int32_t msm_sensor_platform_probe(struct platform_device *pdev, void *data)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl =
		(struct msm_sensor_ctrl_t *)data;
	struct msm_camera_cci_client *cci_client = NULL;
	uint32_t session_id;
	unsigned long mount_pos;
	s_ctrl->pdev = pdev;
	CDBG("%s called data %p\n", __func__, data);
	CDBG("%s pdev name %s\n", __func__, pdev->id_entry->name);
	if (pdev->dev.of_node) {
		rc = msm_sensor_get_dt_data(pdev->dev.of_node, s_ctrl);
		if (rc < 0) {
			pr_err("%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
	}
	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	s_ctrl->sensor_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}
	/* TODO: get CCI subdev */
	cci_client = s_ctrl->sensor_i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	cci_client->sid =
		s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;
	if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
		s_ctrl->sensor_i2c_client->i2c_func_tbl =
			&msm_sensor_cci_func_tbl;
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;
	s_ctrl->sensordata->power_info.clk_info =
		kzalloc(sizeof(cam_8974_clk_info), GFP_KERNEL);
	if (!s_ctrl->sensordata->power_info.clk_info) {
		pr_err("%s:%d failed nomem\n", __func__, __LINE__);
		kfree(cci_client);
		return -ENOMEM;
	}
	memcpy(s_ctrl->sensordata->power_info.clk_info, cam_8974_clk_info,
		sizeof(cam_8974_clk_info));
	s_ctrl->sensordata->power_info.clk_info_size =
		ARRAY_SIZE(cam_8974_clk_info);
#ifdef CONFIG_IMX135_Z5S
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_z5s", 32)) {
		
		if (s_ctrl->zte_otp_enable == true) {
			INIT_DELAYED_WORK(&s_ctrl->zte_otp_worker, imx135_ois_otp);
			mutex_init(&s_ctrl->zte_otp_mutex);
		}
	}
#endif
#ifdef CONFIG_IMX135_GBAO
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_gbao", 32)) {
	     
		if (s_ctrl->zte_otp_enable == true) {
			INIT_DELAYED_WORK(&s_ctrl->zte_otp_worker, imx135_ois_otp);
			mutex_init(&s_ctrl->zte_otp_mutex);
		}
	}
#endif
#ifdef CONFIG_IMX135_GBAO_LC898122
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135_gbao_lc898122", 32)) {
	     
		if (s_ctrl->zte_otp_enable == true) {
			INIT_DELAYED_WORK(&s_ctrl->zte_otp_worker, imx135_ois_otp_lc898122);
		       mutex_init(&s_ctrl->zte_otp_mutex);
		}
	}
#endif
	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		kfree(s_ctrl->sensordata->power_info.clk_info);
		kfree(cci_client);
		return rc;
	}
	#ifdef CONFIG_IMX214 //added for eeprom judge
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx214", 32)) {
		uint16_t temp_eeprom = 0;
		enum msm_camera_i2c_reg_addr_type temp_addr_type;
		cci_client->sid = 0xA0 >> 1;
		temp_addr_type = s_ctrl->sensor_i2c_client->addr_type;
		s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	    s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client, 0x13,
				&temp_eeprom, MSM_CAMERA_I2C_BYTE_DATA);
		cci_client->sid =
			s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
		s_ctrl->sensor_i2c_client->addr_type = temp_addr_type;
		if (temp_eeprom == 0xFF) {
			s_ctrl->sensordata->sensor_info->subdev_id[SUB_MODULE_EEPROM] = -1;
		}
		
		printk("csh temp=%x\n", temp_eeprom);
		#if 0
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
				s_ctrl->sensor_i2c_client, 0x0018,
				&temp_eeprom, MSM_CAMERA_I2C_WORD_DATA);
		
		printk("csh temp=%x\n", temp_eeprom);
		#endif
	}
	#endif
	CDBG("%s %s probe succeeded\n", __func__,
		s_ctrl->sensordata->sensor_name);
	v4l2_subdev_init(&s_ctrl->msm_sd.sd,
		s_ctrl->sensor_v4l2_subdev_ops);
	snprintf(s_ctrl->msm_sd.sd.name,
		sizeof(s_ctrl->msm_sd.sd.name), "%s",
		s_ctrl->sensordata->sensor_name);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, pdev);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name =
		s_ctrl->msm_sd.sd.name;

	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos |
	((s_ctrl->sensordata->sensor_info->sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	rc = camera_init_v4l2(&s_ctrl->pdev->dev, &session_id);
	CDBG("%s rc %d session_id %d\n", __func__, rc, session_id);
	s_ctrl->sensordata->sensor_info->session_id = session_id;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	msm_sd_register(&s_ctrl->msm_sd);
#if defined(CONFIG_IMX135)
	if (!strncmp(s_ctrl->sensordata->sensor_name, "imx135", 32)) {
		IMX135_update_wb_register_from_otp(s_ctrl);
	}
#endif
	CDBG("%s:%d\n", __func__, __LINE__);

	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	CDBG("%s:%d\n", __func__, __LINE__);
	return rc;
}

int msm_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id, struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint32_t session_id;
	unsigned long mount_pos;

	CDBG("%s %s_i2c_probe called\n", __func__, client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s %s i2c_check_functionality failed\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!client->dev.of_node) {
		CDBG("msm_sensor_i2c_probe: of_node is NULL");
		s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
		if (!s_ctrl) {
			pr_err("%s:%d sensor ctrl structure NULL\n", __func__,
				__LINE__);
			return -EINVAL;
		}
		s_ctrl->sensordata = client->dev.platform_data;
	} else {
		CDBG("msm_sensor_i2c_probe: of_node exisists");
		rc = msm_sensor_get_dt_data(client->dev.of_node, s_ctrl);
		if (rc < 0) {
			pr_err("%s failed line %d\n", __func__, __LINE__);
			return rc;
		}
	}

	s_ctrl->sensor_device_type = MSM_CAMERA_I2C_DEVICE;
	if (s_ctrl->sensordata == NULL) {
		pr_err("%s %s NULL sensor data\n", __func__, client->name);
		return -EFAULT;
	}

	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		s_ctrl->sensordata->power_info.dev = &client->dev;
		if (s_ctrl->sensordata->slave_info->sensor_slave_addr)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensordata->slave_info->
				sensor_slave_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;
	if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
		s_ctrl->sensor_i2c_client->i2c_func_tbl =
			&msm_sensor_qup_func_tbl;
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;

	if (!client->dev.of_node) {
		s_ctrl->sensordata->power_info.clk_info =
			kzalloc(sizeof(cam_8960_clk_info), GFP_KERNEL);
		if (!s_ctrl->sensordata->power_info.clk_info) {
			pr_err("%s:%d failed nomem\n", __func__, __LINE__);
			return -ENOMEM;
		}
		memcpy(s_ctrl->sensordata->power_info.clk_info,
			cam_8960_clk_info, sizeof(cam_8960_clk_info));
		s_ctrl->sensordata->power_info.clk_info_size =
			ARRAY_SIZE(cam_8960_clk_info);
	} else {
		s_ctrl->sensordata->power_info.clk_info =
			kzalloc(sizeof(cam_8610_clk_info), GFP_KERNEL);
		if (!s_ctrl->sensordata->power_info.clk_info) {
			pr_err("%s:%d failed nomem\n", __func__, __LINE__);
			return -ENOMEM;
		}
		memcpy(s_ctrl->sensordata->power_info.clk_info,
			cam_8610_clk_info, sizeof(cam_8610_clk_info));
		s_ctrl->sensordata->power_info.clk_info_size =
			ARRAY_SIZE(cam_8610_clk_info);
	}

	rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s %s power up failed\n", __func__, client->name);
		kfree(s_ctrl->sensordata->power_info.clk_info);
		return rc;
	}

	CDBG("%s %s probe succeeded\n", __func__, client->name);
	snprintf(s_ctrl->msm_sd.sd.name,
		sizeof(s_ctrl->msm_sd.sd.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->msm_sd.sd, client,
		s_ctrl->sensor_v4l2_subdev_ops);
	v4l2_set_subdevdata(&s_ctrl->msm_sd.sd, client);
	s_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&s_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	s_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	s_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_SENSOR;
	s_ctrl->msm_sd.sd.entity.name =
		s_ctrl->msm_sd.sd.name;

	mount_pos = s_ctrl->sensordata->sensor_info->position << 16;
	mount_pos = mount_pos |
	((s_ctrl->sensordata->sensor_info->sensor_mount_angle / 90) << 8);
	s_ctrl->msm_sd.sd.entity.flags = mount_pos | MEDIA_ENT_FL_DEFAULT;

	rc = camera_init_v4l2(&s_ctrl->sensor_i2c_client->client->dev,
		&session_id);
	CDBG("%s rc %d session_id %d\n", __func__, rc, session_id);
	s_ctrl->sensordata->sensor_info->session_id = session_id;
	s_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x3;
	msm_sd_register(&s_ctrl->msm_sd);
	CDBG("%s:%d\n", __func__, __LINE__);

	s_ctrl->func_tbl->sensor_power_down(s_ctrl);
	return rc;
}

int32_t msm_sensor_init_default_params(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t                       rc = -ENOMEM;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_cam_clk_info      *clk_info = NULL;

	/* Validate input parameters */
	if (!s_ctrl) {
		pr_err("%s:%d failed: invalid params s_ctrl %p\n", __func__,
			__LINE__, s_ctrl);
		return -EINVAL;
	}

	if (!s_ctrl->sensor_i2c_client) {
		pr_err("%s:%d failed: invalid params sensor_i2c_client %p\n",
			__func__, __LINE__, s_ctrl->sensor_i2c_client);
		return -EINVAL;
	}

	/* Initialize cci_client */
	s_ctrl->sensor_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!s_ctrl->sensor_i2c_client->cci_client) {
		pr_err("%s:%d failed: no memory cci_client %p\n", __func__,
			__LINE__, s_ctrl->sensor_i2c_client->cci_client);
		return -ENOMEM;
	}

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = s_ctrl->sensor_i2c_client->cci_client;

		/* Get CCI subdev */
		cci_client->cci_subdev = msm_cci_get_subdev();

		/* Update CCI / I2C function table */
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl)
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_cci_func_tbl;
	} else {
		if (!s_ctrl->sensor_i2c_client->i2c_func_tbl) {
			CDBG("%s:%d\n", __func__, __LINE__);
			s_ctrl->sensor_i2c_client->i2c_func_tbl =
				&msm_sensor_qup_func_tbl;
		}
	}

	/* Update function table driven by ioctl */
	if (!s_ctrl->func_tbl)
		s_ctrl->func_tbl = &msm_sensor_func_tbl;

	/* Update v4l2 subdev ops table */
	if (!s_ctrl->sensor_v4l2_subdev_ops)
		s_ctrl->sensor_v4l2_subdev_ops = &msm_sensor_subdev_ops;

	/* Initialize clock info */
	clk_info = kzalloc(sizeof(cam_8974_clk_info), GFP_KERNEL);
	if (!clk_info) {
		pr_err("%s:%d failed no memory clk_info %p\n", __func__,
			__LINE__, clk_info);
		rc = -ENOMEM;
		goto FREE_CCI_CLIENT;
	}
	memcpy(clk_info, cam_8974_clk_info, sizeof(cam_8974_clk_info));
	s_ctrl->sensordata->power_info.clk_info = clk_info;
	s_ctrl->sensordata->power_info.clk_info_size =
		ARRAY_SIZE(cam_8974_clk_info);

	return 0;

FREE_CCI_CLIENT:
	kfree(cci_client);
	return rc;
}
