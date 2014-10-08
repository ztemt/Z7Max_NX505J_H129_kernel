/*
** ========================================================================
** Copyright 2013 Texas Instruments Inc.
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
** ========================================================================
*/
/*
** File:
**     drv2605.c
**
** Description:
**     DRV2605 chip driver
**
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/slab.h>
#include <linux/types.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>

#include <linux/syscalls.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>

#include <linux/sched.h>

#include <linux/i2c/drv2605.h>

#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>

#include <linux/workqueue.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/of_gpio.h>


#ifdef VIBRATOR_DEBUG
#define vibrator_debug(fmt, args...) printk(KERN_DEBUG "[drv2605]"fmt, ##args)
#else
#define vibrator_debug(fmt, args...) do {} while(0)
#endif

#ifdef VIBRATOR_MULTI_USERMODE
unsigned char nubia_wave_sequence[] = {
	WAVEFORM_SEQUENCER_REG, 		15,
	WAVEFORM_SEQUENCER_REG2,		150,
	WAVEFORM_SEQUENCER_REG3,		15,
	WAVEFORM_SEQUENCER_REG4,		150,
	WAVEFORM_SEQUENCER_REG5,		15,
	WAVEFORM_SEQUENCER_REG6,		150,
	WAVEFORM_SEQUENCER_REG7,		15,
	WAVEFORM_SEQUENCER_REG8,		150,
};

unsigned char nubia_wave_sequence1[] = {
	WAVEFORM_SEQUENCER_REG, 		1,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence2[] = {
	WAVEFORM_SEQUENCER_REG, 		2,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence3[] = {
	WAVEFORM_SEQUENCER_REG, 		49,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence4[] = {
	WAVEFORM_SEQUENCER_REG, 		34,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence5[] = {
	WAVEFORM_SEQUENCER_REG, 		24,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence6[] = {
	WAVEFORM_SEQUENCER_REG, 		6,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence7[] = {
	WAVEFORM_SEQUENCER_REG, 		74,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence8[] = {
	WAVEFORM_SEQUENCER_REG, 		47,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};

unsigned char nubia_wave_sequence9[] = {
	WAVEFORM_SEQUENCER_REG, 		5,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};
unsigned char nubia_wave_sequence10[] = {
	WAVEFORM_SEQUENCER_REG, 		4,
	WAVEFORM_SEQUENCER_REG2,		0,
	WAVEFORM_SEQUENCER_REG3,		0,
	WAVEFORM_SEQUENCER_REG4,		0,
	WAVEFORM_SEQUENCER_REG5,		0,
	WAVEFORM_SEQUENCER_REG6,		0,
	WAVEFORM_SEQUENCER_REG7,		0,
	WAVEFORM_SEQUENCER_REG8,		0,
};
#endif

#ifdef VIBRATOR_AUTO_CALIBRATE
static const unsigned char autocal_sequence[] = {
	MODE_REG,                       AUTO_CALIBRATION,
	REAL_TIME_PLAYBACK_REG,         REAL_TIME_PLAYBACK_STRENGTH,
	GO_REG,                         GO,
};
#endif

static int drv260x_resume(struct i2c_client *client)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	gpio_set_value(pDrv2605data->PlatData.GpioEnable, GPIO_LEVEL_HIGH);
	return 0;
}

static int drv260x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	gpio_set_value(pDrv2605data->PlatData.GpioEnable, GPIO_LEVEL_LOW);
	return 0;
}

static int drv260x_write_reg_val(struct i2c_client *client,const unsigned char* data, unsigned int size)
{
	int i = 0;
	int err = 0;

	if (size % 2 != 0)
	return -EINVAL;

	vibrator_debug("%s:", __func__);

	while (i < size)
	{
#ifdef VIBRATOR_DEBUG
		printk(" 0x%x:0x%x", data[i], data[i+1]);
#endif
		err = i2c_smbus_write_byte_data(client, data[i], data[i+1]);
		if(err < 0){
			printk(KERN_ERR"%s, err=%d\n", __FUNCTION__, err);
			break;
		}
		i+=2;
	}

#ifdef VIBRATOR_DEBUG
	printk("\n");
#endif

	return err;
}

static void drv260x_set_go_bit(struct i2c_client *client,char val)
{
	char go[] =
	{
		GO_REG, val
	};
	drv260x_write_reg_val(client, go, sizeof(go));
}

static unsigned char drv260x_read_reg(struct i2c_client *client, unsigned char reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static unsigned char drv260x_setbit_reg(struct i2c_client *client, unsigned char reg, unsigned char mask, unsigned char value)
{
	unsigned char temp = 0;
	unsigned char buff[2];
	unsigned char regval = drv260x_read_reg(client,reg);

	temp = regval & ~mask;
	temp |= value & mask;

	if(temp != regval){
		buff[0] = reg;
		buff[1] = temp;

		return drv260x_write_reg_val(client, buff, 2);
	}else
		return 2;
}

static void drv2605_poll_go_bit(struct i2c_client *client)
{
	while (drv260x_read_reg(client, GO_REG) == GO)
	schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));
}

static void drv2605_select_library(struct i2c_client *client, char lib)
{
	char library[] =
	{
		LIBRARY_SELECTION_REG, lib
	};
	drv260x_write_reg_val(client, library, sizeof(library));
}

static void drv260x_set_rtp_val(struct i2c_client *client, char value)
{
	char rtp_val[] =
	{
		REAL_TIME_PLAYBACK_REG, value
	};
	drv260x_write_reg_val(client, rtp_val, sizeof(rtp_val));
}

static void drv2605_set_waveform_sequence(struct i2c_client *client, unsigned char* seq, unsigned int size)
{
	unsigned char data[WAVEFORM_SEQUENCER_MAX + 1];

	if (size > WAVEFORM_SEQUENCER_MAX)
	return;

	memset(data, 0, sizeof(data));
	memcpy(&data[1], seq, size);
	data[0] = WAVEFORM_SEQUENCER_REG;

	i2c_master_send(client, data, sizeof(data));
}

static void drv260x_change_mode(struct i2c_client *client, char mode)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	unsigned char tmp[2] = {MODE_REG, mode};

	if((mode == MODE_PATTERN_RTP_ON) || (mode == MODE_SEQ_RTP_ON))
		tmp[1] = MODE_REAL_TIME_PLAYBACK;
	else if((mode == MODE_PATTERN_RTP_OFF) || (mode == MODE_SEQ_RTP_OFF))
		tmp[1] = MODE_INTERNAL_TRIGGER;

	if(((mode == MODE_INTERNAL_TRIGGER) || (mode == MODE_PATTERN_RTP_OFF)
		|| (mode == MODE_SEQ_RTP_OFF))
		&& ((pDrv2605data->mode == MODE_PATTERN_RTP_OFF) ||
		(pDrv2605data->mode == MODE_INTERNAL_TRIGGER) ||
		(pDrv2605data->mode == MODE_SEQ_RTP_OFF))) {
	} else if(mode != pDrv2605data->mode) {
		//printk("%s, new mode=%d, old mode=%d, reg=0x%x\n", __FUNCTION__,mode, pDrv2605data->mode, tmp[1]);
		drv260x_write_reg_val(client, tmp, sizeof(tmp));
		if(tmp[1] == MODE_STANDBY) {
			schedule_timeout_interruptible(msecs_to_jiffies(10));
		}else if(pDrv2605data->mode == MODE_STANDBY) {
			schedule_timeout_interruptible(msecs_to_jiffies(1));
		}
	}

	pDrv2605data->mode = mode;
}

/* --------------------------------------------------------------------------------- */
#define YES 1
#define NO  0

static void setAudioHapticsEnabled(struct i2c_client *client, int enable);

static struct Haptics {
	struct wake_lock wklock;
	struct pwm_device *pwm_dev;
	struct hrtimer timer;
	struct mutex lock;
	struct work_struct work;
	struct work_struct work_play_eff;
	unsigned char sequence[8];
	volatile int should_stop;
	struct timed_output_dev to_dev;
	int testdata;
} vibdata;

static struct i2c_client *this_client;

static int vibrator_get_time(struct timed_output_dev *dev)
{
	//struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);

	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_off(struct i2c_client *client)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	if (pDrv2605data->vibrator_is_playing) {
		pDrv2605data->vibrator_is_playing = NO;
		if (pDrv2605data->audio_haptics_enabled)
		{
			if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) != MODE_AUDIOHAPTIC)
				setAudioHapticsEnabled(client, YES);
		} else {
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
			drv260x_change_mode(client, MODE_STANDBY);
		}
	}

#ifdef VIBRATOR_MULTI_USERMODE
	pDrv2605data->usermode = 0;
#endif

	wake_unlock(&vibdata.wklock);
}

#ifdef VIBRATOR_MULTI_USERMODE
/* Real-Time Playback (RTP) Mode */
static void vibrator_enable_rtp(struct timed_output_dev *dev, int value)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	//struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);
	char mode;

	mutex_lock(&vibdata.lock);
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);

	if (value) {
		wake_lock(&vibdata.wklock);

		mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
		/* Only change the mode if not already in RTP mode; RTP input already set at init */
		if (mode != MODE_REAL_TIME_PLAYBACK)
		{
			if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
			setAudioHapticsEnabled(client, NO);

			drv260x_set_rtp_val(client, REAL_TIME_PLAYBACK_STRENGTH);
			drv260x_change_mode(client, MODE_REAL_TIME_PLAYBACK);
			pDrv2605data->vibrator_is_playing = YES;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
		}

		if (value > 0) {
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	}	else {
		vibrator_off(client);
	}

	mutex_unlock(&vibdata.lock);
}

/* Internal Trigger (default) Mode */
static void vibrator_enable_internal_trigger(struct timed_output_dev *dev, int value)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	//struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);
	char mode;

	mutex_lock(&vibdata.lock);
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);

	wake_lock(&vibdata.wklock);

	if (value) {
		switch(pDrv2605data->usermode){
		case 1:
			drv260x_write_reg_val(client, nubia_wave_sequence1, sizeof(nubia_wave_sequence1));
			break;
		case 2:
			drv260x_write_reg_val(client, nubia_wave_sequence2, sizeof(nubia_wave_sequence2));
			break;
		case 3:
			drv260x_write_reg_val(client, nubia_wave_sequence3, sizeof(nubia_wave_sequence3));
			break;
		case 4:
			drv260x_write_reg_val(client, nubia_wave_sequence4, sizeof(nubia_wave_sequence4));
			break;
		case 5:
			drv260x_write_reg_val(client, nubia_wave_sequence5, sizeof(nubia_wave_sequence5));
			break;
		case 6:
			drv260x_write_reg_val(client, nubia_wave_sequence6, sizeof(nubia_wave_sequence6));
			break;
		case 7:
			drv260x_write_reg_val(client, nubia_wave_sequence7, sizeof(nubia_wave_sequence7));
			break;
		case 8:
			drv260x_write_reg_val(client, nubia_wave_sequence8, sizeof(nubia_wave_sequence8));
			break;
		case 9:
			drv260x_write_reg_val(client, nubia_wave_sequence9, sizeof(nubia_wave_sequence9));
			break;
		case 10:
			drv260x_write_reg_val(client, nubia_wave_sequence10, sizeof(nubia_wave_sequence10));
			break;
		default:
			printk(KERN_ERR "%s: Unknown usermode(%d).\n", __func__, pDrv2605data->usermode);
			break;
		}

		mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
		if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
			setAudioHapticsEnabled(client, NO);

		drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
		pDrv2605data->vibrator_is_playing = YES;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_SEQUENCE_PLAYBACK);			

		drv260x_set_go_bit(client, GO);
		if (value > 0) {
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	} else {
		vibrator_off(client);
	}

	mutex_unlock(&vibdata.lock);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	vibrator_debug("%s(%d):usermode=%d,value=%d\n", __func__, __LINE__,
			pDrv2605data->usermode, value);

	if(pDrv2605data->usermode){
		vibrator_enable_internal_trigger(dev, value);
	}	else {
		vibrator_enable_rtp(dev, value);
	}
}

#else
static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	//struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);
	char mode;

	mutex_lock(&vibdata.lock);
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);

	if (value) {
		wake_lock(&vibdata.wklock);

		mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
		/* Only change the mode if not already in RTP mode; RTP input already set at init */
		if (mode != MODE_REAL_TIME_PLAYBACK)
		{
			if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC)
			setAudioHapticsEnabled(client, NO);

			drv260x_set_rtp_val(client, REAL_TIME_PLAYBACK_STRENGTH);
			drv260x_change_mode(client, MODE_REAL_TIME_PLAYBACK);
			pDrv2605data->vibrator_is_playing = YES;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
		}

		if (value > 0) {
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	}
	else
		vibrator_off(client);

	mutex_unlock(&vibdata.lock);
}
#endif

#ifdef VIBRATOR_MULTI_USERMODE
static ssize_t vibrator_usermode_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", pDrv2605data->usermode);
}

static ssize_t vibrator_usermode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	unsigned long value;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	pDrv2605data->usermode = value;

	return size;
}

static struct device_attribute vibrator_attributes[] = {
	__ATTR(usermode, 0644, vibrator_usermode_show, vibrator_usermode_store),
};

static int vibrator_create_sysfs(struct device *dev)
{
	int i;

	for(i=0; i<ARRAY_SIZE(vibrator_attributes); i++) {
		if (device_create_file(dev, vibrator_attributes+i))
		goto error;
	}
	return 0;

error:
	for(; i>=0; i--) {
		device_remove_file(dev, vibrator_attributes+i);
	}
	dev_err(dev, "%s:Unable to create sysfs\n", __func__);
	return -1;
}
#endif

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *work)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	if(pDrv2605data->mode == MODE_PATTERN_RTP_ON) {
		drv260x_change_mode(client, MODE_PATTERN_RTP_OFF);
		if(pDrv2605data->repeat_times == 0) {
			drv260x_change_mode(client, MODE_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
		} else {
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)pDrv2605data->silience_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	} else if(pDrv2605data->mode == MODE_PATTERN_RTP_OFF) {
		if(pDrv2605data->repeat_times > 0) {
			pDrv2605data->repeat_times--;
			drv260x_change_mode(client, MODE_PATTERN_RTP_ON);
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)pDrv2605data->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		} else {
			drv260x_change_mode(client, MODE_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
		}
	} else if((pDrv2605data->mode == MODE_SEQ_RTP_OFF)||((pDrv2605data->mode == MODE_SEQ_RTP_ON))) {
		if(pDrv2605data->RTPSeq.RTPindex < pDrv2605data->RTPSeq.RTPCounts){
			int RTPTime = pDrv2605data->RTPSeq.RTPData[pDrv2605data->RTPSeq.RTPindex] >> 8;
			int RTPVal = pDrv2605data->RTPSeq.RTPData[pDrv2605data->RTPSeq.RTPindex] & 0x00ff ;
			if(RTPTime != 0) {
				//printk("%s, RTP SEQ[%d]=amp=0x%x, time=%d \n", __FUNCTION__, pDrv2605data->RTPSeq.RTPindex, RTPVal, RTPTime);
				drv260x_set_rtp_val(client,  RTPVal);
				if(pDrv2605data->mode == MODE_SEQ_RTP_OFF)
					drv260x_change_mode(client, MODE_SEQ_RTP_ON);

				hrtimer_start(&vibdata.timer, ns_to_ktime((u64)RTPTime * NSEC_PER_MSEC), HRTIMER_MODE_REL);
				pDrv2605data->RTPSeq.RTPindex++;
			} else {
				drv260x_change_mode(client, MODE_STANDBY);
				pDrv2605data->vibrator_is_playing = NO;
				switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
			}
		} else {
			drv260x_change_mode(client, MODE_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
		}
	} else {
		vibrator_off(client);
	}
}

/* ----------------------------------------------------------------------------- */

static void play_effect(struct work_struct *work)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	switch_set_state(&pDrv2605data->sw_dev, SW_STATE_SEQUENCE_PLAYBACK);

	if (pDrv2605data->audio_haptics_enabled &&
			((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)){
		setAudioHapticsEnabled(client, NO);
	}

	drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
	drv2605_set_waveform_sequence(client, vibdata.sequence, sizeof(vibdata.sequence));
	drv260x_set_go_bit(client, GO);

	while(drv260x_read_reg(client, GO_REG) == GO && !vibdata.should_stop)
	schedule_timeout_interruptible(msecs_to_jiffies(GO_BIT_POLL_INTERVAL));

	wake_unlock(&vibdata.wklock);
	if (pDrv2605data->audio_haptics_enabled)
	{
		setAudioHapticsEnabled(client, YES);
	} else {
		drv260x_change_mode(client, MODE_STANDBY);
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
	}
}

static void setAudioHapticsEnabled(struct i2c_client *client, int enable)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	vibrator_debug("%s(%d):enable=%d\n", __func__, __LINE__, enable);

	if (enable)
	{
		drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

		drv260x_setbit_reg(client, Control1_REG, Control1_REG_AC_COUPLE_MASK,
			AC_COUPLE_ENABLED );

		drv260x_setbit_reg(client, Control3_REG, Control3_REG_PWMANALOG_MASK,
			INPUT_ANALOG);

		drv260x_change_mode(client, MODE_AUDIOHAPTIC);
		pDrv2605data->audio_haptics_enabled = YES;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_AUDIO2HAPTIC);
	} else {
		drv260x_change_mode(client, MODE_STANDBY); // Disable audio-to-haptics
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

		// Chip needs to be brought out of standby to change the registers
		drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

		drv260x_setbit_reg(client, Control1_REG, Control1_REG_AC_COUPLE_MASK,
			AC_COUPLE_DISABLED);

		drv260x_setbit_reg(client, Control3_REG, Control3_REG_PWMANALOG_MASK,
			INPUT_PWM);

		pDrv2605data->audio_haptics_enabled = NO;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
	}
}

static ssize_t drv260x_read(struct file* filp, char* buff, size_t length, loff_t* offset)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	int ret = 0;

	if(pDrv2605data->pReadValue != NULL){

		ret = copy_to_user(buff,pDrv2605data->pReadValue, pDrv2605data->ReadLen);
		if (ret != 0){
			printk("%s, copy_to_user err=%d \n", __FUNCTION__, ret);
		} else {
			ret = pDrv2605data->ReadLen;
		}
		pDrv2605data->ReadLen = 0;
		kfree(pDrv2605data->pReadValue);
		pDrv2605data->pReadValue = NULL;

	} else {
		buff[0] = pDrv2605data->read_val;
		ret = 1;
	}

	return ret;
}

static bool isforDebug(int cmd){
	return ((cmd == HAPTIC_CMDID_REG_WRITE)
	||(cmd == HAPTIC_CMDID_REG_READ)
	||(cmd == HAPTIC_CMDID_REG_SETBIT));
}

static ssize_t drv260x_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	vibrator_debug("%s(%d):buff[0]=%c(%d)\n", __func__, __LINE__, buff[0], buff[0]);

	mutex_lock(&vibdata.lock);

	if(isforDebug(buff[0])){
	} else {
		hrtimer_cancel(&vibdata.timer);

		vibdata.should_stop = YES;
		cancel_work_sync(&vibdata.work_play_eff);
		cancel_work_sync(&vibdata.work);

		if (pDrv2605data->vibrator_is_playing)
		{
			pDrv2605data->vibrator_is_playing = NO;
			drv260x_change_mode(client, MODE_STANDBY);
		}
	}

	switch(buff[0])
	{
	case HAPTIC_CMDID_PLAY_SINGLE_EFFECT:
	case HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE:
	{
		memset(&vibdata.sequence, 0, sizeof(vibdata.sequence));
		if (!copy_from_user(&vibdata.sequence, &buff[1], len - 1))
		{
			vibdata.should_stop = NO;
			wake_lock(&vibdata.wklock);
			schedule_work(&vibdata.work_play_eff);
		}
		break;
	}
	case HAPTIC_CMDID_PLAY_TIMED_EFFECT:
	{
		unsigned int value = 0;
		char mode;

		value = buff[2];
		value <<= 8;
		value |= buff[1];

		if (value)
		{
			wake_lock(&vibdata.wklock);
			mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
			if (mode != MODE_REAL_TIME_PLAYBACK)
			{
				if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC){
					setAudioHapticsEnabled(client, NO);
				}

				drv260x_set_rtp_val(client, REAL_TIME_PLAYBACK_STRENGTH);
				drv260x_change_mode(client, MODE_REAL_TIME_PLAYBACK);
				switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
				pDrv2605data->vibrator_is_playing = YES;
			}

			if (value > 0)
			{
				if (value > MAX_TIMEOUT)
					value = MAX_TIMEOUT;
				hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
			}
		}
		break;
	}
	case HAPTIC_CMDID_PATTERN_RTP:
	{
		char mode;
		unsigned char strength = 0;

		pDrv2605data->vibration_time = (int)((((int)buff[2])<<8) | (int)buff[1]);
		pDrv2605data->silience_time = (int)((((int)buff[4])<<8) | (int)buff[3]);
		pDrv2605data->repeat_times = buff[5];
		strength = buff[6];

		if(pDrv2605data->vibration_time > 0) {
			mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
				if (mode != MODE_REAL_TIME_PLAYBACK) {
					if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC) {
						setAudioHapticsEnabled(client, NO);
					} else if(mode == MODE_STANDBY) {
						drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
					}

					drv260x_set_rtp_val(client, strength);
					drv260x_change_mode(client, MODE_PATTERN_RTP_ON);
					if(pDrv2605data->repeat_times > 0)
						pDrv2605data->repeat_times--;
					switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
					pDrv2605data->vibrator_is_playing = YES;
				}

				if (pDrv2605data->vibration_time > MAX_TIMEOUT)
					pDrv2605data->vibration_time = MAX_TIMEOUT;

				hrtimer_start(&vibdata.timer, ns_to_ktime((u64)pDrv2605data->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
		break;
	}
	case HAPTIC_CMDID_RTP_SEQUENCE:
	{
		memset(&pDrv2605data->RTPSeq, 0, sizeof(struct RTP_Seq));
		pDrv2605data->RTPSeq.RTPCounts = buff[1];
		if(pDrv2605data->RTPSeq.RTPCounts < 17) {
			if(copy_from_user(pDrv2605data->RTPSeq.RTPData, &buff[2], pDrv2605data->RTPSeq.RTPCounts*2) != 0) {
				break;
			}
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_RTP_PLAYBACK);
		drv260x_change_mode(client, MODE_SEQ_RTP_OFF);
		schedule_work(&vibdata.work);
		}
		break;
	}
	case HAPTIC_CMDID_STOP:
	{
		if (pDrv2605data->vibrator_is_playing)
		{
			pDrv2605data->vibrator_is_playing = NO;
			if (pDrv2605data->audio_haptics_enabled)
			{
				setAudioHapticsEnabled(client, YES);
			} else {
					switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
					drv260x_change_mode(client, MODE_STANDBY);
			}
		}
		vibdata.should_stop = YES;
		break;
	}
	case HAPTIC_CMDID_GET_DEV_ID:
	{
		/* Dev ID includes 2 parts, upper word for device id, lower word for chip revision */
		int revision = (drv260x_read_reg(client, SILICON_REVISION_REG) & SILICON_REVISION_MASK);
		pDrv2605data->read_val = (pDrv2605data->device_id >> 1) | revision;
		break;
	}
	case HAPTIC_CMDID_RUN_DIAG:
	{
		char diag_seq[] =
		{
			MODE_REG, MODE_DIAGNOSTICS,
			GO_REG,   GO
		};

		if (pDrv2605data->audio_haptics_enabled &&
				((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)){
				setAudioHapticsEnabled(client, NO);
		}

		drv260x_write_reg_val(client, diag_seq, sizeof(diag_seq));
		drv2605_poll_go_bit(client);
		pDrv2605data->read_val = (drv260x_read_reg(client, STATUS_REG) & DIAG_RESULT_MASK) >> 3;
		break;
	}
	case HAPTIC_CMDID_AUDIOHAPTIC_ENABLE:
	{
		if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) != MODE_AUDIOHAPTIC)
		{
			setAudioHapticsEnabled(client, YES);
		}
		break;
	}
	case HAPTIC_CMDID_AUDIOHAPTIC_DISABLE:
	{
		if (pDrv2605data->audio_haptics_enabled)
		{
			if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)
				setAudioHapticsEnabled(client, NO);
			drv260x_change_mode(client, MODE_STANDBY);
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);
		}
		break;
	}
	case HAPTIC_CMDID_AUDIOHAPTIC_GETSTATUS:
	{
		if ((drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK) == MODE_AUDIOHAPTIC)
		{
			pDrv2605data->read_val = 1;
		}	else {
			pDrv2605data->read_val = 0;
		}
		break;
	}
	case HAPTIC_CMDID_REG_READ:
	{
		int i=1;
		if(pDrv2605data->pReadValue != NULL){
			printk("%s, ERROR, pReadValue should be NULL\n",__FUNCTION__);
		} else {
			pDrv2605data->pReadValue = (char *)kzalloc(len-1, GFP_KERNEL);
			if(pDrv2605data->pReadValue == NULL){
				printk("%s, ERROR, pReadValue alloc fail\n",__FUNCTION__);					
			} else {
				pDrv2605data->ReadLen = len -1;
				for(i=0;i<(len-1);i++){
					pDrv2605data->pReadValue[i] = drv260x_read_reg(client, buff[i+1]);	
				}
			}
		}
		break;
	}
	case HAPTIC_CMDID_REG_WRITE:
	{
		drv260x_write_reg_val(client, &buff[1], len-1);
		break;
	}
	case HAPTIC_CMDID_REG_SETBIT:
	{
		int i=1;
		for(i=1; i< len; ){
			drv260x_setbit_reg(client, buff[i], buff[i+1], buff[i+2]);
			i += 3;
		}
		break;
	}
	default:
		printk("%s, unknown HAPTIC cmd\n", __FUNCTION__);
		break;
	}

	mutex_unlock(&vibdata.lock);

	return len;
}


static struct file_operations fops =
{
	.read = drv260x_read,
	.write = drv260x_write
};

static int Haptics_init(struct drv2605_data *pDrv2605Data)
{
	int reval = -ENOMEM;

	pDrv2605Data->version = MKDEV(0,0);
	reval = alloc_chrdev_region(&pDrv2605Data->version, 0, 1, HAPTICS_DEVICE_NAME);
	if (reval < 0)
	{
		printk(KERN_ALERT"drv260x: error getting major number %d\n", reval);
		goto fail0;
	}

	pDrv2605Data->class = class_create(THIS_MODULE, HAPTICS_DEVICE_NAME);
	if (!pDrv2605Data->class)
	{
		printk(KERN_ALERT"drv260x: error creating class\n");
		goto fail1;
	}

	pDrv2605Data->device = device_create(pDrv2605Data->class, NULL, pDrv2605Data->version, NULL, HAPTICS_DEVICE_NAME);
	if (!pDrv2605Data->device)
	{
		printk(KERN_ALERT"drv260x: error creating device 2605\n");
		goto fail2;
	}

	cdev_init(&pDrv2605Data->cdev, &fops);
	pDrv2605Data->cdev.owner = THIS_MODULE;
	pDrv2605Data->cdev.ops = &fops;
	reval = cdev_add(&pDrv2605Data->cdev, pDrv2605Data->version, 1);

	if (reval)
	{
		printk(KERN_ALERT"drv260x: fail to add cdev\n");
		goto fail3;
	}

	pDrv2605Data->sw_dev.name = "haptics";
	reval = switch_dev_register(&pDrv2605Data->sw_dev);
	if (reval < 0) {
		printk(KERN_ALERT"drv260x: fail to register switch\n");
		goto fail4;
	}

	vibdata.to_dev.name = "vibrator";
	vibdata.to_dev.get_time = vibrator_get_time;
	vibdata.to_dev.enable = vibrator_enable;

	if (timed_output_dev_register(&(vibdata.to_dev)) < 0)
	{
		printk(KERN_ALERT"drv260x: fail to create timed output dev\n");
		goto fail3;
	}

#ifdef VIBRATOR_MULTI_USERMODE
	if(vibrator_create_sysfs(vibdata.to_dev.dev) < 0)
	{
		printk(KERN_ALERT"drv260x: fail to create sysfs\n");
		goto fail3;
	}
#endif

	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = vibrator_timer_func;
	INIT_WORK(&vibdata.work, vibrator_work);
	INIT_WORK(&vibdata.work_play_eff, play_effect);

	wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);

	printk(KERN_ALERT"drv260x: initialized\n");
	return 0;

fail4:
	switch_dev_unregister(&pDrv2605Data->sw_dev);
fail3:
	device_destroy(pDrv2605Data->class, pDrv2605Data->version);
fail2:
	class_destroy(pDrv2605Data->class);
fail1:
	unregister_chrdev_region(pDrv2605Data->version, 1);
fail0:
	return reval;
}

static void dev_init_platform_data(struct i2c_client* client, struct drv2605_data *pDrv2605data)
{
	struct drv2605_platform_data *pDrv2605Platdata = &pDrv2605data->PlatData;
	struct actuator_data actuator = pDrv2605Platdata->actuator;
	struct audio2haptics_data a2h = pDrv2605Platdata->a2h;
	unsigned char loop = 0;
	unsigned char tmp[8] = {0};

	drv2605_select_library(client, actuator.g_effect_bank);

	//OTP memory saves data from 0x16 to 0x1a
	if(pDrv2605data->OTP == 0) {
		if(actuator.rated_vol != 0){
			tmp[0] = RATED_VOLTAGE_REG;
			tmp[1] = actuator.rated_vol;
			printk("%s, RatedVol = 0x%x\n", __FUNCTION__, actuator.rated_vol);
			drv260x_write_reg_val(client, tmp, 2);
		} else {
			printk("%s, ERROR Rated ZERO\n", __FUNCTION__);
		}

		if(actuator.over_drive_vol != 0){
			tmp[0] = OVERDRIVE_CLAMP_VOLTAGE_REG;
			tmp[1] = actuator.over_drive_vol;
			printk("%s, OverDriveVol = 0x%x\n", __FUNCTION__, actuator.over_drive_vol);
			drv260x_write_reg_val(client, tmp, 2);
		} else {
			printk("%s, ERROR OverDriveVol ZERO\n", __FUNCTION__);
		}

		drv260x_setbit_reg(client, FEEDBACK_CONTROL_REG,
			FEEDBACK_CONTROL_DEVICE_TYPE_MASK,
			(actuator.device_type == LRA)?FEEDBACK_CONTROL_MODE_LRA:FEEDBACK_CONTROL_MODE_ERM);
	} else {
		printk("%s, OTP programmed\n", __FUNCTION__);
	}

	if(actuator.loop == OPEN_LOOP) {
		if(actuator.device_type == LRA)
			loop = 0x01;
		else if(actuator.device_type == ERM)
			loop = ERM_OpenLoop_Enabled;
	}

	drv260x_setbit_reg(client, Control3_REG, Control3_REG_LOOP_MASK, loop);

	//for audio to haptics
	if(pDrv2605Platdata->GpioTrigger == 0)	//not used as external trigger
	{
		tmp[0] = AUDIO_HAPTICS_MIN_INPUT_REG;
		tmp[1] = a2h.a2h_min_input;
		tmp[2] = AUDIO_HAPTICS_MAX_INPUT_REG;
		tmp[3] = a2h.a2h_max_input;
		tmp[4] = AUDIO_HAPTICS_MIN_OUTPUT_REG;
		tmp[5] = a2h.a2h_min_output;
		tmp[6] = AUDIO_HAPTICS_MAX_OUTPUT_REG;
		tmp[7] = a2h.a2h_max_output;
		drv260x_write_reg_val(client, tmp, sizeof(tmp));
	}
}

#ifdef VIBRATOR_AUTO_CALIBRATE
static int dev_auto_calibrate(struct i2c_client* client, struct drv2605_data *pDrv2605data)
{
	int err = 0, status=0;

	err = drv260x_write_reg_val(client, autocal_sequence, sizeof(autocal_sequence));
	pDrv2605data->mode = AUTO_CALIBRATION;

	/* Wait until the procedure is done */
	drv2605_poll_go_bit(client);
	/* Read status */
	status = drv260x_read_reg(client, STATUS_REG);

	if(pDrv2605data->device_id != (status & DEV_ID_MASK)) {
		printk("%s, ERROR after calibration status =0x%x\n", __FUNCTION__, status);
		return -ENODEV;
	}

	/* Check result */
	if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
	{
		printk(KERN_ALERT"drv260x auto-cal failed.\n");
		drv260x_write_reg_val(client, autocal_sequence, sizeof(autocal_sequence));

		drv2605_poll_go_bit(client);
		status = drv260x_read_reg(client, STATUS_REG);
		if ((status & DIAG_RESULT_MASK) == AUTO_CAL_FAILED)
		{
			printk(KERN_ALERT"drv260x auto-cal retry failed.\n");
			// return -ENODEV;
		}
	}

	/* Read calibration results */
	drv260x_read_reg(client, AUTO_CALI_RESULT_REG);
	drv260x_read_reg(client, AUTO_CALI_BACK_EMF_RESULT_REG);
	drv260x_read_reg(client, FEEDBACK_CONTROL_REG);

	return err;
}
#else
#ifdef CONFIG_ZTEMT_HAPTICS_DRV2605_DRIVE_ERM
static int dev_manual_calibrate(struct i2c_client* client, struct drv2605_data *pDrv2605data)
{
	return 0;
}
#else
static int dev_manual_calibrate(struct i2c_client* client, struct drv2605_data *pDrv2605data)
{
	unsigned char manual_seq[] = {
		RATED_VOLTAGE_REG,              NUBIA_LRA_RATED_VOLTAGE,
		OVERDRIVE_CLAMP_VOLTAGE_REG,    NUBIA_LRA_OVERDRIVE_CLAMP_VOLTAGE,
		AUTO_CALI_RESULT_REG,           NUBIA_LRA_AUTOCAL_COMPENSATION,
		AUTO_CALI_BACK_EMF_RESULT_REG,  NUBIA_LRA_AUTOCAL_BACKEMF,
	};

	drv260x_write_reg_val(client, manual_seq, sizeof(manual_seq));

	drv260x_setbit_reg(client, FEEDBACK_CONTROL_REG,
		FEEDBACK_CONTROL_BEMF_GAIN_MASK, FEEDBACK_CONTROL_BEMF_LRA_GAIN2);

	return 0;
}
#endif
#endif

#ifdef CONFIG_OF
static int drv260x_parse_dt(struct device *dev,
			struct drv2605_platform_data *pdata)
{
	struct device_node *temp = NULL, *np = dev->of_node;
	enum of_gpio_flags gpio_enable_flags = OF_GPIO_ACTIVE_LOW;
	int rc = 0;
	u32 temp_val = 0;

	pdata->GpioTrigger = of_property_read_bool(np, "immersion,external-trigger");
	pdata->GpioEnable = of_get_named_gpio_flags(np,
				"immersion,gpio-enable", 0, &gpio_enable_flags);

	temp = of_find_node_by_name(np, "immersion,actuator");
	if(!temp) {
		dev_err(dev, "Unable to find actuator data\n");
		return -1;
	}
	rc = of_property_read_u32(temp, "actuator,type", &pdata->actuator.device_type);
	if (rc) {
		dev_err(dev, "Unable to read actuator type\n");
		return rc;
	}
	rc = of_property_read_u32(temp, "actuator,effect-bank", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read actuator effect bank\n");
		return rc;
	} else {
		pdata->actuator.g_effect_bank = temp_val;
	}
	rc = of_property_read_u32(temp, "actuator,loop", &pdata->actuator.loop);
	if (rc) {
		dev_err(dev, "Unable to read actuator loop type\n");
		return rc;
	}
	rc = of_property_read_u32(temp, "actuator,rated-vol", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read actuator rated voltage\n");
		return rc;
	} else {
		pdata->actuator.rated_vol = temp_val;
	}
	rc = of_property_read_u32(temp, "actuator,over-drive-vol", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read actuator over driver voltage\n");
		return rc;
	} else {
		pdata->actuator.over_drive_vol = temp_val;
	}

	temp = of_find_node_by_name(np, "immersion,audio2haptics");
	if(!temp) {
		dev_err(dev, "Unable to find audio2haptics data\n");
		return -1;
	}
	rc = of_property_read_u32(temp, "a2h,min-input", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read a2h min input\n");
		return rc;
	} else {
		pdata->a2h.a2h_min_input = temp_val;
	}
	rc = of_property_read_u32(temp, "a2h,max-input", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read a2h max-input\n");
		return rc;
	} else {
		pdata->a2h.a2h_max_input = temp_val;
	}
	rc = of_property_read_u32(temp, "a2h,min-output", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read a2h min-output\n");
		return rc;
	} else {
		pdata->a2h.a2h_min_output = temp_val;
	}
	rc = of_property_read_u32(temp, "a2h,max-output", &temp_val);
	if (rc) {
		dev_err(dev, "Unable to read actuator a2h max-output\n");
		return rc;
	} else {
		pdata->a2h.a2h_max_output = temp_val;
	}

	vibrator_debug("%s:GpioTrigger=%d, GpioEnable=%d\n", __func__,
		pdata->GpioTrigger, pdata->GpioEnable);
	vibrator_debug("%s:actuator:0x%x,0x%x,0x%x,0x%x,0x%x\n", __func__,
		pdata->actuator.device_type, pdata->actuator.g_effect_bank,
		pdata->actuator.loop, pdata->actuator.rated_vol,
		pdata->actuator.over_drive_vol);
	vibrator_debug("%s:a2h:0x%x,0x%x,0x%x,0x%x\n", __func__,
		pdata->a2h.a2h_min_input,	pdata->a2h.a2h_max_input,
		pdata->a2h.a2h_min_output, pdata->a2h.a2h_max_output);

	return 0;
}
#else
static int drv260x_parse_dt(struct device *dev,
		struct drv2605_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int drv260x_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct drv2605_data *pDrv2605data;
	struct drv2605_platform_data *pDrv2605Platdata;
	int err = 0;
	int status = 0;

	if (client->dev.of_node) {
		pDrv2605Platdata = devm_kzalloc(&client->dev,
			sizeof(struct drv2605_platform_data), GFP_KERNEL);
		if (!pDrv2605Platdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = drv260x_parse_dt(&client->dev, pDrv2605Platdata);
		if (err) {
			dev_err(&client->dev, "Parsing DT failed(%d)", err);
			return err;
		}
	} else {
		pDrv2605Platdata = client->dev.platform_data;
	}

	if (!pDrv2605Platdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR"%s:I2C check failed\n", __FUNCTION__);
		return -ENODEV;
	}

	pDrv2605data = kzalloc(sizeof(struct drv2605_data),GFP_KERNEL);
	if(!pDrv2605data){
		err = -ENOMEM;
		printk(KERN_ERR"%s: -ENOMEM error\n", __FUNCTION__);
		goto exit_alloc_data_failed;
	}

	pDrv2605data->client = client;
	this_client = client;

	memcpy(&pDrv2605data->PlatData, pDrv2605Platdata, sizeof(struct drv2605_platform_data));
	i2c_set_clientdata(client,pDrv2605data);

	if(pDrv2605data->PlatData.GpioTrigger){
		err = gpio_request(pDrv2605data->PlatData.GpioTrigger, HAPTICS_DEVICE_NAME"Trigger");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request Trigger error\n", __FUNCTION__);				
			goto exit_gpio_request_failed1;
		}
	}

	if(pDrv2605data->PlatData.GpioEnable){
		err = gpio_request(pDrv2605data->PlatData.GpioEnable, HAPTICS_DEVICE_NAME"Enable");
		if(err < 0){
			printk(KERN_ERR"%s: GPIO request enable error\n", __FUNCTION__);
			goto exit_gpio_request_failed2;
		}

		/* Enable power to the chip */
		gpio_direction_output(pDrv2605data->PlatData.GpioEnable, GPIO_LEVEL_HIGH);

		/* Wait 30 us */
		udelay(30);
	}

	status = drv260x_read_reg(pDrv2605data->client, STATUS_REG);
	/* Read device ID */
	pDrv2605data->device_id = (status & DEV_ID_MASK);
	switch (pDrv2605data->device_id)
	{
	case DRV2605_VER_1DOT1:
		printk("drv260x driver found: drv2605 v1.1.\n");
		break;
	case DRV2605_VER_1DOT0:
		printk("drv260x driver found: drv2605 v1.0.\n");
		break;
	case DRV2604:
		printk(KERN_ALERT"drv260x driver found: drv2604.\n");
		break;
	default:
		printk(KERN_ERR"drv260x driver found: unknown.\n");
		break;
	}

	if((pDrv2605data->device_id != DRV2605_VER_1DOT1)
		&& (pDrv2605data->device_id != DRV2605_VER_1DOT0)) {
		printk("%s, status(0x%x),device_id(%d) fail\n",
			__FUNCTION__, status, pDrv2605data->device_id);
		err = -ENODEV;
		goto exit_device_id_failed;
	}

	pDrv2605data->mode = MODE_STANDBY;

	drv260x_change_mode(pDrv2605data->client, MODE_INTERNAL_TRIGGER);
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

	pDrv2605data->OTP = drv260x_read_reg(pDrv2605data->client, AUTOCAL_MEM_INTERFACE_REG) & AUTOCAL_MEM_INTERFACE_REG_OTP_MASK;

	dev_init_platform_data(pDrv2605data->client, pDrv2605data);

#ifdef VIBRATOR_AUTO_CALIBRATE
	if(pDrv2605data->OTP == 0) {
		err = dev_auto_calibrate(pDrv2605data->client, pDrv2605data);
		if(err < 0){
			printk("%s, ERROR, calibration fail\n",	__FUNCTION__);
			goto exit_device_id_failed;
		}
	}
#else
	if(pDrv2605data->OTP == 0) {
		dev_manual_calibrate(pDrv2605data->client, pDrv2605data);
	}
#endif

	/* Put hardware in standby */
	drv260x_change_mode(pDrv2605data->client, MODE_STANDBY);

	Haptics_init(pDrv2605data);

	printk("drv260x probe succeeded\n");

	return 0;

	exit_device_id_failed:
	if(pDrv2605data->PlatData.GpioEnable){
		gpio_set_value(pDrv2605data->PlatData.GpioEnable, GPIO_LEVEL_LOW);
		gpio_free(pDrv2605data->PlatData.GpioEnable);
	}

	exit_gpio_request_failed2:
	if(pDrv2605data->PlatData.GpioTrigger){
		gpio_free(pDrv2605data->PlatData.GpioTrigger);
	}

	exit_gpio_request_failed1:
	if(pDrv2605data){
		kfree(pDrv2605data);
	}
	exit_alloc_data_failed:
	printk(KERN_ERR"%s failed, err=%d\n",__FUNCTION__, err);
	return err;
}

static int drv260x_remove(struct i2c_client* client)
{
	struct drv2605_data *pDrv2605Data = i2c_get_clientdata(client);

	device_destroy(pDrv2605Data->class, pDrv2605Data->version);
	class_destroy(pDrv2605Data->class);
	unregister_chrdev_region(pDrv2605Data->version, 1);

	if(pDrv2605Data->PlatData.GpioTrigger)
	gpio_free(pDrv2605Data->PlatData.GpioTrigger);

	if(pDrv2605Data->PlatData.GpioEnable)
	gpio_free(pDrv2605Data->PlatData.GpioEnable);

	kfree(pDrv2605Data);

	i2c_set_clientdata(client,NULL);

	printk(KERN_ALERT"drv260x remove");

	return 0;
}


static struct i2c_device_id drv260x_id_table[] =
{
	{ HAPTICS_DEVICE_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, drv260x_id_table);

#ifdef CONFIG_OF
static struct of_device_id drv2605_match_table[] = {
	{ .compatible = "immersion,drv2605",},
	{ },
};
#else
#define drv2605_match_table NULL
#endif

static struct i2c_driver drv260x_driver =
{
	.driver = {
		.name = HAPTICS_DEVICE_NAME,
		.of_match_table = drv2605_match_table,
	},
	.id_table = drv260x_id_table,
	.probe = drv260x_probe,
	.remove = drv260x_remove,
	.suspend = drv260x_suspend,
	.resume = drv260x_resume
};

static int __init drv260x_init(void)
{
	return i2c_add_driver(&drv260x_driver);
}

static void __exit drv260x_exit(void)
{
	i2c_del_driver(&drv260x_driver);
}

module_init(drv260x_init);
module_exit(drv260x_exit);

MODULE_AUTHOR("Texas Instrument Inc.");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);
