/*
 ** =============================================================================
 ** Copyright (c) 2012  Immersion Corporation.
 **
 ** This program is free software; you can redistribute it and/or
 ** modify it under the terms of the GNU General Public License
 ** as published by the Free Software Foundation; either version 2
 ** of the License, or (at your option) any later version.
 **
 ** This program is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ** GNU General Public License for more details.
 **
 ** You should have received a copy of the GNU General Public License
 ** along with this program; if not, write to the Free Software
 ** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **
 ** File:
 **     drv2605.c
 **
 ** Description:
 **     DRV2605 chip driver
 **
 ** =============================================================================
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


#include <asm/uaccess.h>
#include <linux/gpio.h>

#include <linux/of_gpio.h>
//#include <mach/gpiomux.h>

//#include <linux/gpio.h>
//#include <mach/gpio-herring.h>

#include <linux/sched.h>

#include "drv2605.h"

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


/*  Current code version: 182 */
MODULE_AUTHOR("Immersion Corp.");
MODULE_DESCRIPTION("Driver for "HAPTICS_DEVICE_NAME);

//#define NEED_DO_AUTO_CAL_FOR_LRA_VIB 1

static const unsigned char autocal_sequence[] = {
	MODE_REG,                       AUTO_CALIBRATION,
	REAL_TIME_PLAYBACK_REG,         REAL_TIME_PLAYBACK_STRENGTH,	
	GO_REG,                         GO,
};


static struct drv2605_platform_data  drv2605_plat_data = {
	//.GpioEnable = GPIO8,			//enable the chip
	.GpioTrigger = 0,						//external trigger pin, (0: internal trigger)

	//rated = 1.5Vrms, ov=2.1Vrms, f=204hz
	.actuator = {
		.device_type = LRA,
		.g_effect_bank = LIBRARY_F,
		.loop = CLOSE_LOOP,
		.rated_vol = 0x50,
		.over_drive_vol = 0x80,
		.drive_time = DEFAULT_DRIVE_TIME,
	},
	.a2h = {
		.a2h_min_input = AUDIO_HAPTICS_MIN_INPUT_VOLTAGE,
		.a2h_max_input = AUDIO_HAPTICS_MAX_INPUT_VOLTAGE,
		.a2h_min_output = AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE,
		.a2h_max_output = AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE,		
	},


};
#if 0

static struct i2c_board_info __initdata drv2605_i2c_info = {
	I2C_BOARD_INFO(HAPTICS_DEVICE_NAME, 0x5A),
	.platform_data = &drv2605_plat_data,
};
#endif
static int drv260x_write_reg_val(struct i2c_client *client,const unsigned char* data, unsigned int size)
{
	int i = 0;
	int err = 0;

	if (size % 2 != 0)
		return -EINVAL;

	while (i < size)
	{
		err = i2c_smbus_write_byte_data(client, data[i], data[i+1]);
		if(err < 0){
			printk(KERN_ERR"%s, err=%d\n", __FUNCTION__, err);
			break;
		}	
		i+=2;
	}

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

	if(mode == MODE_PATTERN_RTP_ON)
		tmp[1] = MODE_REAL_TIME_PLAYBACK;
	else if(mode == MODE_PATTERN_RTP_OFF)
		tmp[1] = MODE_STANDBY;

	if(((mode == MODE_STANDBY) || (mode == MODE_PATTERN_RTP_OFF))
			&&((pDrv2605data->mode == MODE_PATTERN_RTP_OFF) || (pDrv2605data->mode == MODE_STANDBY))){
	}else if(mode != pDrv2605data->mode){
		drv260x_write_reg_val(client, tmp, sizeof(tmp));
		if(tmp[1] == MODE_STANDBY){
			schedule_timeout_interruptible(msecs_to_jiffies(10));
		}else if((pDrv2605data->mode == MODE_STANDBY)||(pDrv2605data->mode == MODE_PATTERN_RTP_OFF)){
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
	//	struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);

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
		} else
		{
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);		
			drv260x_change_mode(client, MODE_STANDBY);
		}
	}

	wake_unlock(&vibdata.wklock);
}

static void vibrator_enable( struct timed_output_dev *dev, int value)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);
	//	struct Haptics	*pvibdata = container_of(dev, struct Haptics, to_dev);
	char mode;

	mutex_lock(&vibdata.lock);
	hrtimer_cancel(&vibdata.timer);
	cancel_work_sync(&vibdata.work);
	pr_debug("vibrator_enable: %d\n", value);
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

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	schedule_work(&vibdata.work);
	return HRTIMER_NORESTART;
}

static void vibrator_work(struct work_struct *work)
{
	struct i2c_client *client = this_client;
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	if(pDrv2605data->mode == MODE_PATTERN_RTP_ON){
		drv260x_change_mode(client, MODE_PATTERN_RTP_OFF);
		if(pDrv2605data->repeat_times == 0){
			drv260x_change_mode(client, MODE_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);	
		}else{
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)pDrv2605data->silience_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}
	}else if(pDrv2605data->mode == MODE_PATTERN_RTP_OFF){
		if(pDrv2605data->repeat_times > 0){
			pDrv2605data->repeat_times--;
			drv260x_change_mode(client, MODE_PATTERN_RTP_ON);
			hrtimer_start(&vibdata.timer, ns_to_ktime((u64)pDrv2605data->vibration_time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		}else{
			drv260x_change_mode(client, MODE_STANDBY);
			pDrv2605data->vibrator_is_playing = NO;
			switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);	
		}
	}else{
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
	} else
	{
		drv260x_change_mode(client, MODE_STANDBY);
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_IDLE);		
	}
}

static void setAudioHapticsEnabled(struct i2c_client *client, int enable)
{
	struct drv2605_data *pDrv2605data = i2c_get_clientdata(client);

	if (enable)
	{
		drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

		drv260x_setbit_reg(client, 
				Control1_REG, 
				Control1_REG_AC_COUPLE_MASK, 
				AC_COUPLE_ENABLED );

		drv260x_setbit_reg(client, 
				Control3_REG, 
				Control3_REG_PWMANALOG_MASK, 
				INPUT_ANALOG);	

		drv260x_change_mode(client, MODE_AUDIOHAPTIC);
		pDrv2605data->audio_haptics_enabled = YES;
		switch_set_state(&pDrv2605data->sw_dev, SW_STATE_AUDIO2HAPTIC);
	} else
	{
		drv260x_change_mode(client, MODE_STANDBY); // Disable audio-to-haptics
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

		// Chip needs to be brought out of standby to change the registers
		drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
		schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

		drv260x_setbit_reg(client, 
				Control1_REG, 
				Control1_REG_AC_COUPLE_MASK, 
				AC_COUPLE_DISABLED );

		drv260x_setbit_reg(client, 
				Control3_REG, 
				Control3_REG_PWMANALOG_MASK, 
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
		}else{
			ret = pDrv2605data->ReadLen;
		}
		pDrv2605data->ReadLen = 0;
		kfree(pDrv2605data->pReadValue);
		pDrv2605data->pReadValue = NULL;

	}else{

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

	mutex_lock(&vibdata.lock);

	if(isforDebug(buff[0])){
	}else{
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

				if(pDrv2605data->vibration_time > 0){
					mode = drv260x_read_reg(client, MODE_REG) & DRV260X_MODE_MASK;
					if (mode != MODE_REAL_TIME_PLAYBACK){
						if (pDrv2605data->audio_haptics_enabled && mode == MODE_AUDIOHAPTIC){
							setAudioHapticsEnabled(client, NO);
						}else if(mode == MODE_STANDBY){
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
		case HAPTIC_CMDID_STOP:
			{
				if (pDrv2605data->vibrator_is_playing)
				{
					pDrv2605data->vibrator_is_playing = NO;
					if (pDrv2605data->audio_haptics_enabled)
					{
						setAudioHapticsEnabled(client, YES);
					} else
					{
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
				}
				else
				{
					pDrv2605data->read_val = 0;
				}
				break;
			}
		case HAPTIC_CMDID_REG_READ:
			{
				int i=1;
				if(pDrv2605data->pReadValue != NULL){
					printk("%s, ERROR, pReadValue should be NULL\n",__FUNCTION__);
				}else{
					pDrv2605data->pReadValue = (char *)kzalloc(len-1, GFP_KERNEL);
					if(pDrv2605data->pReadValue == NULL){
						printk("%s, ERROR, pReadValue alloc fail\n",__FUNCTION__);					
					}else{
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
		}else{
			printk("%s, ERROR Rated ZERO\n", __FUNCTION__);
		}

		if(actuator.over_drive_vol != 0){
			tmp[0] = OVERDRIVE_CLAMP_VOLTAGE_REG;
			tmp[1] = actuator.over_drive_vol;
			printk("%s, OverDriveVol = 0x%x\n", __FUNCTION__, actuator.over_drive_vol);
			drv260x_write_reg_val(client, tmp, 2);
		}else{
			printk("%s, ERROR OverDriveVol ZERO\n", __FUNCTION__);
		}

		drv260x_setbit_reg(client, 
				FEEDBACK_CONTROL_REG, 
				FEEDBACK_CONTROL_DEVICE_TYPE_MASK, 
				(actuator.device_type == LRA)?FEEDBACK_CONTROL_MODE_LRA:FEEDBACK_CONTROL_MODE_ERM);
	}else{
		printk("%s, OTP programmed\n", __FUNCTION__);
	}

	if(actuator.drive_time != DEFAULT_DRIVE_TIME){
		drv260x_setbit_reg(client, 
				Control1_REG, 
				Control1_REG_DRIVE_TIME_MASK, 
				actuator.drive_time);
	}

	if(actuator.loop == OPEN_LOOP){
		if(actuator.device_type == LRA)
			loop = 0x01;
		else if(actuator.device_type == ERM)
			loop = ERM_OpenLoop_Enabled;
	}

	drv260x_setbit_reg(client, 
			Control3_REG, 
			Control3_REG_LOOP_MASK, 
			loop);	

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

#ifdef NEED_DO_AUTO_CAL_FOR_LRA_VIB
static int dev_auto_calibrate(struct i2c_client* client, struct drv2605_data *pDrv2605data)
{
	int err = 0, status=0;
	int auto_cali_rst, auto_cali_bak_efm_rst, fbc_ctrl;

	err = drv260x_write_reg_val(client, autocal_sequence, sizeof(autocal_sequence));
	pDrv2605data->mode = AUTO_CALIBRATION;		

	/* Wait until the procedure is done */
	drv2605_poll_go_bit(client);
	/* Read status */
	status = drv260x_read_reg(client, STATUS_REG);
	pr_info("drv260x_read_reg device id is 0x%x\n", status);

	if(pDrv2605data->device_id != (status & DEV_ID_MASK)){
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
	auto_cali_rst = drv260x_read_reg(client, AUTO_CALI_RESULT_REG);
	auto_cali_bak_efm_rst = drv260x_read_reg(client, AUTO_CALI_BACK_EMF_RESULT_REG);
	fbc_ctrl = drv260x_read_reg(client, FEEDBACK_CONTROL_REG);
	pr_info("Read calibration results is 0x%x, 0x%x, 0x%x\n", auto_cali_rst, auto_cali_bak_efm_rst, fbc_ctrl);

	return err;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND

void drv260x_early_suspend(struct early_suspend *h)
{
	struct drv2605_data *pDrv2605data = container_of(h, struct drv2605_data,
			early_suspend);
	struct i2c_client *client = pDrv2605data->client;

#if 1
	printk("%s\n", __FUNCTION__);
#else	
	drv260x_change_mode(client, MODE_STANDBY); 
	vibdata.repeat_times = 0;
	vibdata.timed_effect = 0;
	vibdata.should_stop = YES;
	pDrv2605Data->vibrator_is_playing = NO;
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
#endif

	return ;
}

void drv260x_late_resume(struct early_suspend *h)
{
	struct drv2605_data *pDrv2605data = container_of(h, struct drv2605_data,
			early_suspend);
	struct i2c_client *client = pDrv2605data->client;

#if 1
	printk("%s\n", __FUNCTION__);
#else	
	drv260x_change_mode(client, MODE_INTERNAL_TRIGGER);
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));
#endif
	return ;
}
#endif
static int drv260x_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct drv2605_data *pDrv2605data;
	struct drv2605_platform_data *pDrv2605Platdata = &drv2605_plat_data;

	int err = 0;
	int status = 0, auto_cali_rst, auto_cali_bak_efm_rst, fbc_ctrl;
	int en_gpio = 0;
	
	printk(KERN_ERR"%s: \n", __FUNCTION__);

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

	if (client->dev.of_node) {
		en_gpio = of_get_named_gpio(client->dev.of_node, "drv2605,en", 0);
		if(en_gpio < 0) {
			pr_err("parse enable gpio fail: %d\n", en_gpio);
		}else {
			if (gpio_is_valid(en_gpio)) {
				err = gpio_request(en_gpio, "drv2605_vib_en_gpio");
				if (err) {
					pr_err("%s: gpio %d request failed\n", __func__, en_gpio);
				}
			} else {
					pr_err( "%s: Invalid gpio %d\n", __func__, en_gpio);
			}
			gpio_direction_output(en_gpio, 1);
			gpio_set_value(en_gpio, 1);

			if (gpio_is_valid(en_gpio))
				gpio_free(en_gpio);
		}
	}

	status = drv260x_read_reg(pDrv2605data->client, STATUS_REG);
	/* Read device ID */
	pDrv2605data->device_id = (status & DEV_ID_MASK);
	switch (pDrv2605data->device_id)
	{
		case DRV2605_VER_1DOT1:
			pr_info("drv260x driver found: drv2605 v1.1.\n");
			break;
		case DRV2605_VER_1DOT0:
			pr_info("drv260x driver found: drv2605 v1.0.\n");
			break;
		case DRV2604:
			pr_info(KERN_ALERT"drv260x driver found: drv2604.\n");
			break;
		default:
			pr_info(KERN_ERR"drv260x driver found: unknown.\n");
			break;
	}

#if 0
	if((pDrv2605data->device_id != DRV2605_VER_1DOT1)
			&&(pDrv2605data->device_id != DRV2605_VER_1DOT0)){
		printk("%s, status(0x%x),device_id(%d) fail\n",
				__FUNCTION__, status, pDrv2605data->device_id);
		goto exit_gpio_request_failed2;
	}
#endif

	pDrv2605data->mode = MODE_STANDBY;

	drv260x_change_mode(pDrv2605data->client, MODE_INTERNAL_TRIGGER);
	schedule_timeout_interruptible(msecs_to_jiffies(STANDBY_WAKE_DELAY));

	pDrv2605data->OTP = drv260x_read_reg(pDrv2605data->client, AUTOCAL_MEM_INTERFACE_REG) & AUTOCAL_MEM_INTERFACE_REG_OTP_MASK;

	dev_init_platform_data(pDrv2605data->client, pDrv2605data);

	if(pDrv2605data->OTP == 0){
		/* Read calibration results */
		auto_cali_rst = drv260x_read_reg(client, AUTO_CALI_RESULT_REG);
		auto_cali_bak_efm_rst = drv260x_read_reg(client, AUTO_CALI_BACK_EMF_RESULT_REG);
		fbc_ctrl = drv260x_read_reg(client, FEEDBACK_CONTROL_REG);
		pr_info("Read calibration results is 0x%x, 0x%x, 0x%x\n", auto_cali_rst, auto_cali_bak_efm_rst, fbc_ctrl);
#ifdef NEED_DO_AUTO_CAL_FOR_LRA_VIB
		err = dev_auto_calibrate(pDrv2605data->client, pDrv2605data);
		if(err < 0){
			printk("%s, ERROR, calibration fail\n",	__FUNCTION__);
			//goto exit_gpio_request_failed2;
		}
#endif
	}

	/* Put hardware in standby */
	drv260x_change_mode(pDrv2605data->client, MODE_STANDBY);

	Haptics_init(pDrv2605data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	pDrv2605data->early_suspend.suspend = drv260x_early_suspend;
	pDrv2605data->early_suspend.resume = drv260x_late_resume;
	pDrv2605data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&pDrv2605data->early_suspend);
#endif	

	printk("drv260x probe succeeded\n");

	return 0;


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

static const struct of_device_id drv2605_of_match[] = {
	{ .compatible = "ti,drv2605", },
	{ },
};
MODULE_DEVICE_TABLE(of, drv2605_of_match);

static struct i2c_driver drv260x_driver = {
	.driver = {
		.name	= HAPTICS_DEVICE_NAME,
		.owner	= THIS_MODULE,
		//.pm = &mpu6050_pm,
		.of_match_table = drv2605_of_match,
	},
	.probe		= drv260x_probe,
	.remove 	= drv260x_remove,
	.id_table	= drv260x_id_table,
};

module_i2c_driver(drv260x_driver);
MODULE_DESCRIPTION("TI drv2605 ic driver");
MODULE_LICENSE("GPL v1");

