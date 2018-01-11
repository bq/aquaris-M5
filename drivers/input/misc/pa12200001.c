/*
 * This file is part of the PA12200001 sensor driver.
 * PA12200001 is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: Alan Hsiao  <alanhsiao@txc.com.tw>
 *	    Chester Hsu <chesterhsu@txc.com.tw>
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *Reversion
 *1.06 : fix ioctl bugs
 *1.07 : fix interrupt bugs

*/
//#define DEBUG
#define pr_fmt(fmt)      "%s: " fmt, __func__
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <asm/atomic.h>
#include "pa12200001.h"

#define PA12200001_DRV_NAME	"pa12200001"
#define DRIVER_VERSION		"1.06"
#define PA1220_PICMT_CAL_MULT	25
#define PA1220_PICMT_CAL_DIV	10

#define MISC_DEV_NAME		"alsps_dev"
#define PS_CAL_FILE_PATH	"/persist/xtalk_cal"
#define THRD_CAL_FILE_PATH	"/data/thrd_cal" //Threshold Calbration file path

#define ABS_LIGHT			0x28
#define PS_POLLING_RATE		100

static int pa12200001_open(struct inode *inode, struct file *file);
static int pa12200001_release(struct inode *inode, struct file *file);
static long pa12200001_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int pa12200001_run_fast_calibration(struct i2c_client *client);

struct pa12200001_data {
    struct 		i2c_client *client;
    struct 		mutex lock;
    struct 		pa12200001_platform_data *pdata;

    struct 		workqueue_struct *wq;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif

#ifdef USE_LIGHT_FEATURE
    struct 		input_dev *light_input_dev;
#endif
    struct 		work_struct work_light;
#ifdef USE_LIGHT_FEATURE
    struct 		hrtimer light_timer;
    ktime_t 	light_poll_delay;
#endif
	struct		wake_lock ps_wakelock;
    struct 		input_dev *proximity_input_dev;
    struct 		work_struct work_proximity;
    struct 		work_struct work_irq;
    struct 		hrtimer proximity_timer;
    ktime_t 	proximity_poll_delay;
    ulong 		enable;

    int          ps_enable;
    int          als_enable;
	int pre_lux;
    /* PS Calibration */
    u8 		crosstalk;
    u8 		crosstalk_base;
    int      cal_result;

    /* threshold */
    u8		ps_thrd_low;
    u8		ps_thrd_high;
    u8		irq_enabled;
    int irq_gpio;
    int irq;
    struct regulator *vdd;
    unsigned int irq_gpio_flags;
	struct sensors_classdev ps_cdev;
	int ps_stat;
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "pa12200001",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000,	/* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

#ifdef USE_LIGHT_FEATURE
/* pa12200001 range*/
static int pa12200001_range[4] = {149,1363,2833,13897};
#endif

/* Object status, near=0, far=1 */
static int intr_flag = 1;

/* Global Variant */
static bool pa122_has_load_param = false;
static int bCal_flag=0;
#ifdef USE_LIGHT_FEATURE
static int ALSTemp[10];
static int ALSTempIndex=0;
static int ALSAVGStart=0;
#endif
//static u8  PS_PRAM[4]={0,0,0,0}; // 0: Xtalk_offset 1:xtalk_value 2: PS_Threshold_low 3: PS_Threshold_high

struct i2c_client *pa12_i2c_client = NULL;
static int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *buf)
{
    int ret = 0;
    int i = 0;
	struct pa12200001_data *data = i2c_get_clientdata(client);

    for (i = 0; i < I2C_RETRY_TIMES;i++){
		mutex_lock(&data->lock);
        ret = i2c_smbus_read_byte_data(client, reg);
		mutex_unlock(&data->lock);
        if (ret < 0) {
            pr_err("failed to read 0x%x, slave_addr=0x%x\n", reg, client->addr);
            msleep(I2C_RETRY_DELAY);
        } else {
            *buf = (u8) ret;
            return 0;	//success
        }
    }

    return ret;
}

static int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value)
{
    int ret = 0;
    int i = 0;
	struct pa12200001_data *data = i2c_get_clientdata(client);

    for (i = 0;i < I2C_RETRY_TIMES; i++){
		mutex_lock(&data->lock);
        ret = i2c_smbus_write_byte_data(client, reg, value);
		mutex_unlock(&data->lock);
        if (ret < 0) {
            pr_err("failed to write 0x%x, slave_addr=0x%x\n", reg, client->addr);
            msleep(I2C_RETRY_DELAY);
        } else {
            return 0;	//success
        }
    }
    return ret;
}

static int pa12200001_read_file(char *filename,u8* param)
{
    struct file *fop;
    mm_segment_t old_fs;
	loff_t pos = 0;
	char buf[8];
	int ret;
	unsigned int a, b;

    fop = filp_open(filename,O_RDONLY,0);
    if (IS_ERR(fop)){
        pr_err("open file %s failed !\n",filename);
        return -1;
    }

    old_fs = get_fs();
    set_fs(get_ds()); //set_fs(KERNEL_DS);
    ret = vfs_read(fop, buf, 8, &pos);
	if (ret > 0) {
		sscanf(buf, "%u %u\n", &a, &b);
		param[0] = (u8)a;
		param[1] = (u8)b;
	}
    set_fs(old_fs);

    filp_close(fop,NULL);
    return 0;
}

static ssize_t pa12200001_write_file(char *filename, u8* param)
{
    struct file *fop;
    mm_segment_t old_fs;
	loff_t pos = 0;
	char buf[8];
	int ret;

    fop = filp_open(filename, O_CREAT|O_RDWR, 0644);
    if (IS_ERR(fop)){
        pr_err("create file %s failed !\n", filename);
        return -1;
    }
	snprintf(buf, sizeof(buf), "%03u %03u",
		     param[0], param[1]);

    old_fs = get_fs();
    set_fs(get_ds()); //set_fs(KERNEL_DS);
    ret = vfs_write(fop, buf, strlen(buf), &pos);
	if (ret != strlen(buf))
		pr_err("write file failed !\n");
    set_fs(old_fs);

    filp_close(fop,NULL);
    return 0;
}

static int pa12200001_get_mode(struct i2c_client *client)
{
    u8 data =0;
    i2c_read_reg(client, REG_CFG0, &data);
    return (data & 0x03);
}

static int pa12200001_set_mode(struct i2c_client *client, int mode)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int	ret;
	u8 buftemp[2]={0};
    u8 regdata = 0;
	int previous_mode = pa12200001_get_mode(client);
	int ps_already_enabled = (previous_mode & PS_ACTIVE) >> 1;

    pr_debug("mode = 0x%x\n", (u32)mode);

    data->ps_enable=((PS_ACTIVE & mode)>>1);
    data->als_enable=(ALS_ACTIVE & mode);

	if (!pa122_has_load_param) {
		pa122_has_load_param = true;
	    /* Check ps calibration file */
	    if(pa12200001_read_file(PS_CAL_FILE_PATH,buftemp)<0){
	        pr_warn("read file faild, use default offset\n");

	        buftemp[1] = data->crosstalk_base=10;
	        buftemp[0] = data->crosstalk=PA12_PS_OFFSET_DEFAULT+PA12_PS_OFFSET_EXTRA;
	        pa12200001_write_file(PS_CAL_FILE_PATH,buftemp); //Create x-tal_cal file
	    } else {
            if(buftemp[0] != 0) {
                pr_info("use cal file, x-talk=%d, base=%d\n",buftemp[0],buftemp[1]);
                data->crosstalk=buftemp[0];
                data->crosstalk_base=buftemp[1];
            } else {
                pr_info("xtalk read 0, crosstalk=%u\n", data->crosstalk);
            }

            data->ps_thrd_low = PA12_PS_TH_LOW2;
            data->ps_thrd_high = PA12_PS_TH_HIGH2;
	    }
	    i2c_write_reg(client,REG_PS_OFFSET,data->crosstalk); //X-talk Cancelling
	    data->cal_result = 0;
	}

    if(data->ps_enable && PA12_FAST_CAL && !ps_already_enabled)
        pa12200001_run_fast_calibration(client);

#ifdef USE_LIGHT_FEATURE
    if(data->als_enable & ALS_AVG_ENABLE)
        ALSAVGStart=1;
#endif

    if (mode != previous_mode) {
#ifdef USE_LIGHT_FEATURE
        if (ALS_POLLING) {
            /* Enable/Disable ALS */
            if (ALS_ACTIVE & mode) {
                hrtimer_start(&data->light_timer, data->light_poll_delay, HRTIMER_MODE_REL);
            } else {
                hrtimer_cancel(&data->light_timer);
                cancel_work_sync(&data->work_light);
            }
        }
#endif

        if (PS_POLLING) {
            /* Enable/Disable PS */
            if ((PS_ACTIVE & mode)>>1) {
                hrtimer_start(&data->proximity_timer, data->proximity_poll_delay, HRTIMER_MODE_REL);
            } else {
                hrtimer_cancel(&data->proximity_timer);
                cancel_work_sync(&data->work_proximity);
            }
        }

		if (data->ps_enable && !ps_already_enabled) {
			u8 buf;
			i2c_write_reg(client, REG_PS_TH, 0xFF);
			i2c_write_reg(client, REG_PS_TL, 0x00);
			i2c_read_reg(client, REG_CFG2, &buf);
			i2c_write_reg(client, REG_CFG2, buf & 0xFD);
		}

        ret = i2c_read_reg(client, REG_CFG0, &regdata);
        regdata = regdata & 0xFC;
        ret = i2c_write_reg(client, REG_CFG0, regdata | mode);

		if (data->ps_enable && !ps_already_enabled) {
			msleep(50);
            if(data->ps_thrd_low == PA12_PS_TH_LOW2&&data->ps_thrd_high == PA12_PS_TH_HIGH2) {
                i2c_write_reg(client, REG_PS_TH, PA12_PS_TH_HIGH2);
                i2c_write_reg(client, REG_PS_TL, PA12_PS_TH_LOW2);
            } else {
                i2c_write_reg(client, REG_PS_TH, PA12_PS_TH_HIGH);
                i2c_write_reg(client, REG_PS_TL, PA12_PS_TH_LOW);
            }
		}
    }
    return 0;
}

#ifdef USE_LIGHT_FEATURE
/* range */
static int pa12200001_get_range(struct i2c_client *client)
{
    u8 data = 0;
    i2c_read_reg(client, REG_CFG0, &data);
    return ((data >> 4) & 0x03);
}
#endif

#if 0
static int pa12200001_set_range(struct i2c_client *client, int range)
{
    int ret;
    u8 data =0;

    ret = i2c_read_reg(client, REG_CFG0, &data);
    data = data & 0x03;
    data = data | (range << 4);
    ret = i2c_write_reg(client, REG_CFG0, data);

    return ret;
}
#endif

static int pa12200001_get_pscrosstalk(struct i2c_client *client)
{
    int ret;
    u8 data = 0;
    ret = i2c_read_reg(client, REG_PS_OFFSET, &data);
    return data;
}

void pa12_swap(u8 *x, u8 *y)
{
    u8 temp = *x;
    *x = *y;
    *y = temp;
}

static int pa12200001_thrd_calibration(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int i,j,temp_diff;
    u16 sum_of_pdata = 0;
    u8 temp_pdata[10],buftemp[2],cfg0data=0,temp_thrd;
    unsigned int ArySize = 10;

    pr_debug("START threshold calibration\n");

    i2c_read_reg(client, REG_CFG0, &cfg0data);
    i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); /*PS On*/

    for(i = 0; i < 10; i++){
        mdelay(50);
        i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
        pr_debug("ps temp_data = %d\n", temp_pdata[i]);
    }

    /* pdata sorting */
    for (i = 0; i < ArySize - 1; i++) {
        for (j = i+1; j < ArySize; j++) {
            if (temp_pdata[i] > temp_pdata[j])
                pa12_swap(temp_pdata + i, temp_pdata + j);
        }
    }

    /* calculate the cross-talk using central 5 data */
    for (i = 3; i < 8; i++) {
        pr_debug("temp_pdata = %d\n", temp_pdata[i]);
        sum_of_pdata = sum_of_pdata + temp_pdata[i];
    }

    temp_thrd = sum_of_pdata/5;
    temp_diff = temp_thrd - (data->crosstalk_base+PA12_PS_TH_BASE_HIGH);
    if(temp_diff < 0){
        pr_debug("Threshold Cal fail \n");
        return -1 ;

    }else{
		buftemp[1]=temp_thrd; // High Treshold

        if (data->crosstalk > 50) {
            if((temp_thrd -80) > data->crosstalk_base)
                buftemp[0]=temp_thrd - 80 ;
            else
                buftemp[0]=(data->crosstalk_base+PA12_PS_TH_BASE_LOW);
        } else {
            if((temp_thrd -40) > data->crosstalk_base)
                buftemp[0]=temp_thrd - 40 ;
            else
                buftemp[0]=(data->crosstalk_base+PA12_PS_TH_BASE_LOW);
        }

        data->ps_thrd_low=buftemp[0];
        data->ps_thrd_high=buftemp[1];

        if(pa12200001_write_file(THRD_CAL_FILE_PATH,buftemp)<0)  //Write to file
            pr_err("Create PS Thredhold calibration file error!!");
        else
            pr_debug("Create PS Thredhold calibration file Success!!");
    }
    return 0;
}


static DEFINE_MUTEX(calibrate_lock);

static int pa12200001_run_calibration(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int i, j;
    int ret;
    u16 sum_of_pdata = 0;
    u8 temp_pdata[20],buftemp[2],cfg0data=0,cfg2data=0;
    unsigned int ArySize = 20;
    unsigned int cal_check_flag = 0;

    pr_debug("START proximity sensor calibration\n");
	
	mutex_lock(&calibrate_lock);
RECALIBRATION:
	/* Prevent Interrupt */
	ret = i2c_write_reg(client, REG_PS_TH, 0xFF);
	ret = i2c_write_reg(client, REG_PS_TL, 0x00);

    ret = i2c_read_reg(client, REG_CFG2, &cfg2data);
    ret = i2c_write_reg(client, REG_CFG2, cfg2data & 0x33); /*Offset mode & disable intr from ps*/
    ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00); /*Set crosstalk = 0*/

    ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
    ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); /*PS On*/

    for(i = 0; i < 20; i++){
        mdelay(50);
        ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
        pr_debug("temp_data = %d\n", temp_pdata[i]);
    }

    /* pdata sorting */
    for (i = 0; i < ArySize - 1; i++)
        for (j = i+1; j < ArySize; j++)
            if (temp_pdata[i] > temp_pdata[j])
                pa12_swap(temp_pdata + i, temp_pdata + j);

    /* calculate the cross-talk using central 10 data */
    for (i = 5; i < 15; i++) {
        pr_debug("temp_pdata = %d\n", temp_pdata[i]);
        sum_of_pdata = sum_of_pdata + temp_pdata[i];
    }

    data->crosstalk = sum_of_pdata/10;
    pr_debug("sum_of_pdata = %d, cross_talk = %d\n",
            sum_of_pdata, data->crosstalk);

    /* Restore CFG2 (Normal mode) and Measure base x-talk */
    ret = i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0); //make sure return normal mode

    if (data->crosstalk > 50){
        pr_debug("invalid calibrated data\n");

        if(cal_check_flag == 0){
            pr_debug("RECALIBRATION start\n");
            cal_check_flag = 1;
            goto RECALIBRATION;
        }else{
            pr_debug("CALIBRATION FAIL -> cross_talk is set to DEFAULT\n");
            data->cal_result = -1;
            data->crosstalk = PA12_PS_OFFSET_DEFAULT + PA12_PS_OFFSET_EXTRA;
			ret = i2c_write_reg(client, REG_PS_OFFSET, data->crosstalk);
			ret = i2c_write_reg(client, REG_PS_TH, PA12_PS_TH_HIGH);
			ret = i2c_write_reg(client, REG_PS_TL, PA12_PS_TH_LOW);
			mutex_unlock(&calibrate_lock);
            return -EINVAL;
        }
    }

	data->crosstalk += PA12_PS_OFFSET_EXTRA;

CROSSTALK_BASE_RECALIBRATION:
    /*Write offset value to 0x10*/
    ret = i2c_write_reg(client, REG_PS_OFFSET, data->crosstalk);
    sum_of_pdata = 0;

    for(i = 0; i < 10; i++){
        mdelay(50);
        ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
        pr_debug("temp_data = %d\n", temp_pdata[i]);
    }

    /* calculate the cross-talk_base using central 5 data */
    for (i = 3; i < 8; i++) {
        pr_debug("temp_pdata = %d\n", temp_pdata[i]);
        sum_of_pdata = sum_of_pdata + temp_pdata[i];
    }

    data->crosstalk_base = sum_of_pdata/5;
    pr_debug("sum_of_pdata = %d, cross_talk_base = %d\n",
             sum_of_pdata, data->crosstalk_base);

	if (data->crosstalk_base > 0) {
		data->crosstalk += 1;
		goto CROSSTALK_BASE_RECALIBRATION;
	}

    /* Restore CFG0  */
	ret = i2c_write_reg(client, REG_PS_TH, PA12_PS_TH_HIGH);
	ret = i2c_write_reg(client, REG_PS_TL, PA12_PS_TH_LOW);
    ret = i2c_write_reg(client, REG_CFG0, cfg0data);

    pr_debug("FINISH proximity sensor calibration\n");
    /*Write x-talk info to file*/
    buftemp[0]=data->crosstalk;//-PA12_PS_OFFSET_EXTRA;
    buftemp[1]=data->crosstalk_base;

	pr_info("write buftemp[0]=%u, buftemp[1]=%u\n", buftemp[0], buftemp[1]);
    if(pa12200001_write_file(PS_CAL_FILE_PATH,buftemp)<0) {
        pr_err("Open PS x-talk calibration file error!!");
        data->cal_result = -2;
		mutex_unlock(&calibrate_lock);
		return -EFAULT;
    } else {
        pr_debug("Open PS x-talk calibration file Success!!");
    }

    data->cal_result = 0;
	mutex_unlock(&calibrate_lock);
    return data->crosstalk;
}

static int pa12200001_run_fast_calibration(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int i=0;
    int j=0;
    u16 sum_of_pdata = 0;
    u8  xtalk_temp=0;
    u8 temp_pdata[4], cfg0data=0,cfg2data=0;//,cfg3data=0;
    unsigned int ArySize = 4;

    if (bCal_flag & PA12_FAST_CAL_ONCE){
        pr_debug("Ignore Fast Calibration\n");
        return data->crosstalk;
    }

    pr_debug("START proximity sensor fast calibration\n");
	/* Prevent Interrupt */
	i2c_write_reg(client, REG_PS_TH, 0xFF);
	i2c_write_reg(client, REG_PS_TL, 0x00);

    i2c_read_reg(client, REG_CFG2, &cfg2data);
    i2c_write_reg(client, REG_CFG2, cfg2data & 0x33); /*Offset mode & disable intr from ps*/

    i2c_write_reg(client, REG_PS_OFFSET, 0x00); /*Set crosstalk = 0*/

    i2c_read_reg(client, REG_CFG0, &cfg0data);
    i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); /*PS On*/

    for(i = 0; i < 4; i++){
        mdelay(50);
        i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
        pr_debug("temp_data = %d\n", temp_pdata[i]);
    }

    /* pdata sorting */
    for (i = 0; i < ArySize - 1; i++)
        for (j = i+1; j < ArySize; j++)
            if (temp_pdata[i] > temp_pdata[j])
                pa12_swap(temp_pdata + i, temp_pdata + j);

    /* calculate the cross-talk using central 10 data */
    for (i = 1; i < 3; i++) {
        pr_debug("temp_pdata = %d\n", temp_pdata[i]);
        sum_of_pdata = sum_of_pdata + temp_pdata[i];
    }

    xtalk_temp = sum_of_pdata/2;
	xtalk_temp += PA12_PS_OFFSET_EXTRA;
    pr_debug("sum_of_pdata = %d   cross_talk = %d\n",
             sum_of_pdata, data->crosstalk);

    //Restore Data
    i2c_write_reg(client, REG_CFG0, cfg0data);
    i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0); //make sure return normal mode

    if ((sum_of_pdata<510-PA12_PS_OFFSET_EXTRA)&&(xtalk_temp < PA12_PS_OFFSET_MAX)
		&& (data->crosstalk < xtalk_temp)) {
        pr_debug("Fast calibrated data=%d\n",xtalk_temp);
        bCal_flag = 1;
        //Write offset value to 0x10
        i2c_write_reg(client,REG_PS_OFFSET,xtalk_temp);
    } else {
        pr_debug("Fast calibration fail\n");
        i2c_write_reg(client,REG_PS_OFFSET,data->crosstalk);
    }
    pr_debug("FINISH PS calibration\n");

    return xtalk_temp;
}

#ifdef USE_LIGHT_FEATURE
void pa12200001_adjust_als_threshold(struct i2c_client *client,int count)
{
    int newTH,newTL;
    u8 regdata=0;

    newTH=count+PA12_ALS_TH_RANGE;
    if(newTH>65535)
        newTH=65535;

    newTL=count-PA12_ALS_TH_RANGE;
    if(newTL<0)
        newTL=0;

    /* Reset ALS Threshold */
    i2c_write_reg(client,REG_ALS_TH_LSB,newTH & 0xFF); //set TH threshold
    i2c_write_reg(client,REG_ALS_TH_MSB,(newTH >>8) & 0xFF); //set TH threshold
    i2c_write_reg(client,REG_ALS_TL_LSB,newTL & 0xFF); //set TL threshold
    i2c_write_reg(client,REG_ALS_TL_MSB,(newTL >>8) & 0xFF); //set TL threshold

    i2c_read_reg(client, REG_CFG2, &regdata);
    i2c_write_reg(client, REG_CFG2, regdata & 0xFC); // clear INT Flag
    return ;
}

static int pa12200001_get_als_value(struct i2c_client *client)
{
    u8 lsb=0, msb=0;
    int als_adc=0;
    int i=0;
    int sum=0;

    i2c_read_reg(client, REG_ALS_DATA_LSB, &lsb);
    i2c_read_reg(client, REG_ALS_DATA_MSB, &msb);
    als_adc=((msb << 8) | lsb);
    //pr_debug("ALS ADC Count=%d\n",als_adc);

    if(ALS_AVG_ENABLE){ //AVG Function
        if(ALSAVGStart){
            ALSAVGStart=0;
            ALSTempIndex=0;

            for(i=0;i<10;i++)
				ALSTemp[i]=als_adc;
        } else {
            if(ALSTempIndex<=9)
                ALSTempIndex++;
            else
                ALSTempIndex=0;
            ALSTemp[ALSTempIndex]=als_adc;

            for(i=0;i<10;i++)
                sum+=ALSTemp[i];

            return (sum/10);
        }
    }
    return als_adc;
}

static int pa12200001_get_lux_value(struct i2c_client *client)
{
    int als_adc = pa12200001_get_als_value(client);
    int lux;
    int range = pa12200001_get_range(client);

    if(!ALS_POLLING)
        pa12200001_adjust_als_threshold(client,als_adc); //Dynamic Threshold

    lux = (als_adc * pa12200001_range[range]) >> 10;
	lux = lux * PA1220_PICMT_CAL_MULT / PA1220_PICMT_CAL_DIV; // Calibration for PICMT 
    return lux;
}
#endif
static u8 pa12200001_get_ps_value(struct i2c_client *client)
{
    u8 data = 0;
    i2c_read_reg(client, REG_PS_DATA, &data);
    return data;
}
static int pa12200001_power_init(struct pa12200001_data *data, bool on)
{
    int rc;

    if (!on)
        goto pwr_deinit;

    data->vdd = regulator_get(&data->client->dev, "vdd");
    if (IS_ERR(data->vdd)) {
        rc = PTR_ERR(data->vdd);
        dev_err(&data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(data->vdd) > 0) {
        rc = regulator_set_voltage(data->vdd, PAl2_VDD_MIN_UV,
                PAl2_VDD_MAX_UV);
        if (rc) {
            dev_err(&data->client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_put;
        }
    }
    return 0;

reg_vdd_put:
    regulator_put(data->vdd);
    return rc;

pwr_deinit:
    if (regulator_count_voltages(data->vdd) > 0)
        regulator_set_voltage(data->vdd, 0, PAl2_VDD_MAX_UV);
    regulator_put(data->vdd);
    return 0;
}

static int pa12200001_init_client(struct i2c_client *client)
{
    int ret;
    u8 int_set = 0;

    struct pa12200001_data *data = i2c_get_clientdata(client);

    pa12200001_power_init(data,true);

    /* Initialize Sensor */
    ret=i2c_write_reg(client,REG_CFG0,
            (PA12_ALS_GAIN << 4));

    ret=i2c_write_reg(client,REG_CFG1,
            ((PA12_LED_CURR	<< 4)| (PA12_PS_PRST << 2)| (PA12_ALS_PRST)));

    ret=i2c_write_reg(client,REG_CFG3,
            ((PA12_INT_TYPE	<< 6)| (PA12_PS_PERIOD << 3)| (PA12_ALS_PERIOD)));

    ret=i2c_write_reg(client,REG_PS_SET,0x03); //PSET

#if 0
    /* Check threshold calibration file */
    if(pa12200001_read_file(THRD_CAL_FILE_PATH,buftemp)<0){
        pr_debug("Use Default threhold , Low = %d , High = %d\n",PA12_PS_TH_LOW,PA12_PS_TH_HIGH);

        buftemp[0]=data->ps_thrd_low=PA12_PS_TH_LOW;
        buftemp[1]=data->ps_thrd_high=PA12_PS_TH_HIGH;
        pa12200001_write_file(THRD_CAL_FILE_PATH,buftemp); //Create thrd_cal file

    }else{
        pr_debug("Use Threshold Cal file , Low = %d , High = %d\n",buftemp[0],buftemp[1]);
        data->ps_thrd_low=buftemp[0];
        data->ps_thrd_high=buftemp[1];
    }
#endif

    data->ps_thrd_low = PA12_PS_TH_LOW;
    data->ps_thrd_high = PA12_PS_TH_HIGH;

    if(!PS_POLLING)
    {
        /* Set PS threshold */
        i2c_write_reg(client,REG_PS_TH,data->ps_thrd_high); //set TH threshold
        i2c_write_reg(client,REG_PS_TL,data->ps_thrd_low); //set TL threshold
        int_set=1; //set PS INT
    }

#ifdef USE_LIGHT_FEATURE
    if(!ALS_POLLING)
    {
        /* Set ALS threshold */
        i2c_write_reg(client,REG_ALS_TH_LSB,PA12_ALS_TH_HIGH & 0xFF); //set TH threshold
        i2c_write_reg(client,REG_ALS_TH_MSB,(PA12_ALS_TH_HIGH >>8) & 0xFF); //set TH threshold
        i2c_write_reg(client,REG_ALS_TL_LSB,PA12_ALS_TH_LOW & 0xFF); //set TL threshold
        i2c_write_reg(client,REG_ALS_TL_MSB,(PA12_ALS_TH_LOW >>8) & 0xFF); //set TL threshold
        if(int_set)
            int_set=3; // if PS INT enable set 3 (both enable)

    }
#endif

    ret=i2c_write_reg(client,REG_CFG2,
            ((PA12_PS_MODE	<< 6)|(int_set << 2)));
    if(ret < 0) {
        pr_err("i2c_send function err\n");
        goto EXIT_ERR;
    }

    return 0;

EXIT_ERR:
    pr_err("pa12200001 init dev fail %d\n", ret);
    return ret;
}

static int pa12200001_get_object(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    u8 psdata = pa12200001_get_ps_value(client);
    u8 regdata=0;

    if (PS_POLLING) {
        switch (intr_flag) {
            case 0:
                if(psdata < data->ps_thrd_low)
                    intr_flag = 1;
                break;
            case 1:
                if(psdata > data->ps_thrd_high)
                    intr_flag = 0;
                break;
        }
    } else {
        switch (PA12_INT_TYPE) {
            case 0: /* Window Type */
                if(intr_flag == 1){
                    if(psdata > data->ps_thrd_high){
                        intr_flag = 0;
                        i2c_write_reg(client,REG_PS_TL,data->ps_thrd_low);
                        i2c_write_reg(client,REG_PS_TH, PA12_PS_TH_MAX);
                    }
                } else if(intr_flag == 0) {
                    if (psdata < data->ps_thrd_low) {
                        intr_flag = 1;
                        i2c_write_reg(client,REG_PS_TL,PA12_PS_TH_MIN);
                        i2c_write_reg(client,REG_PS_TH,data->ps_thrd_high);
                    }
                }
                i2c_read_reg(client, REG_CFG2, &regdata);
                i2c_write_reg(client, REG_CFG2, regdata & 0xFC); // clear PS INT Flag
                break;

            case 1: /* Hysteresis Type */
                if(psdata > data->ps_thrd_high){
					i2c_write_reg(client,REG_CFG1,
						((PA12_LED_CURR	<< 4)| (1 << 2)| (PA12_ALS_PRST)));
                    intr_flag = 0;
                } else if (psdata < data->ps_thrd_low) {
                    intr_flag = 1;
					i2c_write_reg(client,REG_CFG1,
						((PA12_LED_CURR	<< 4)| (PA12_PS_PRST << 2)| (PA12_ALS_PRST)));
                }
                break;
        }
    }
	pr_info("ps_thrd_low=%u, ps_thrd_high=%u, psdata=%u, report %s\n",
			data->ps_thrd_low, data->ps_thrd_high, psdata, intr_flag ? "Far" : "Near");

    return intr_flag ? PA12_PS_FAR_DISTANCE : PA12_PS_NEAR_DISTANCE;
}

static int pa12200001_get_intstat(struct i2c_client *client)
{
    u8 data =0;
    i2c_read_reg(client, REG_CFG2, &data);
    return (data & 0x03);
}

/* X-talk Calibration file */
static ssize_t pa12200001_show_calibration_file(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
    u8 buftemp[2];
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    /* Check ps calibration file */
    if (pa12200001_read_file(PS_CAL_FILE_PATH,buftemp) < 0){
        pr_debug("Use Default ps offset , x-talk = %d\n",PA12_PS_OFFSET_DEFAULT);

        buftemp[1] = data->crosstalk_base=10;
        buftemp[0] = data->crosstalk = PA12_PS_OFFSET_DEFAULT;
        buftemp[0] = data->crosstalk - PA12_PS_OFFSET_EXTRA;
        pa12200001_write_file(PS_CAL_FILE_PATH, buftemp); //Create x-tal_cal file

    } else {
        pr_debug("Use PS Cal file , x-talk = %d base = %d\n",buftemp[0],buftemp[1]);
        data->crosstalk=buftemp[0]+PA12_PS_OFFSET_EXTRA  ;
        data->crosstalk_base=buftemp[1];
    }

    i2c_write_reg(data->client,REG_PS_OFFSET,data->crosstalk); //X-talk Cancelling

    return sprintf(buf, "X-talk data= %d\n",buftemp[0]);
}
static ssize_t pa12200001_store_calibration_file(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct file  *fop;
    mm_segment_t old_fs;
    u8 tempbuf[2];
    int temp;
    sscanf(buf, "%d", &temp);
    tempbuf[0]=(u8)temp;

    fop = filp_open(PS_CAL_FILE_PATH,O_CREAT | O_RDWR,0644);
    if(IS_ERR(fop)){
        pr_err("open file %s failed !", PS_CAL_FILE_PATH);
        return -1;
    }

    old_fs = get_fs();
    set_fs(get_ds()); //set_fs(KERNEL_DS);
    fop->f_op->write(fop, (char *)tempbuf, sizeof(tempbuf), &fop->f_pos);
    set_fs(old_fs);

    filp_close(fop,NULL);
    return count;
}
static DEVICE_ATTR(xtalk_param, S_IWUGO | S_IRUGO,
        pa12200001_show_calibration_file, pa12200001_store_calibration_file);

/* Threshold Calibration file */
static ssize_t pa12200001_show_pthrd_file(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    u8 readtemp[2];
    if (pa12200001_read_file(THRD_CAL_FILE_PATH,readtemp) < 0)
        return sprintf(buf, "Open File Error\n");

    return sprintf(buf, "Low thrd = %d, High thrd = %d\n",readtemp[0],readtemp[1]);
}

static ssize_t pa12200001_store_pthrd_file(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    u8 buftemp[2];
    int temp[2];


    if (2 != sscanf(buf, "%d %d", &temp[0], &temp[1])) {
        pr_err("invalid format: '%s'\n", buf);
        return -1;
    }
    buftemp[0]=(u8)temp[0];
    buftemp[1]=(u8)temp[1];

    if(pa12200001_write_file(THRD_CAL_FILE_PATH,buftemp)<0)
        pr_err("Create PS Thredhold calibration file error!!");
    else
        pr_debug("Create PS Thredhold calibration file Success!!");

    return count;
}

static int pa12200001_ps_set_enable(struct sensors_classdev *sensors_cdev,
			      unsigned int enable)
{
	struct pa12200001_data *pa1 =
	    container_of(sensors_cdev, struct pa12200001_data, ps_cdev);
    int mode = 0;

	if ((enable != 0) && (enable != 1)) {
		pr_err("invalid argument %d\n", enable);
		return -EINVAL;
	}

    mode = pa12200001_get_mode(pa1->client);
    if(enable == 1) {
        //turn on ps  sensor
        int Pval;
        mode |= PS_ACTIVE;
        set_bit(PS_ACTIVE, &pa1->enable);
		pa12200001_set_mode(pa1->client, mode);

        Pval=pa12200001_get_object(pa1->client);
        pr_debug("PS value: %d\n", Pval);

        input_report_abs(pa1->proximity_input_dev, ABS_DISTANCE, Pval);
        input_sync(pa1->proximity_input_dev);
		pa1->ps_stat = Pval;
		irq_set_irq_wake(pa1->irq, 1);
		wake_lock(&pa1->ps_wakelock);
		if (!pa1->irq_enabled) {
			enable_irq(pa1->irq);
			pa1->irq_enabled = 1;
		}
    } else {
		if (pa1->irq_enabled) {
			disable_irq(pa1->irq);
			pa1->irq_enabled = 0;
		}
        mode &= ~PS_ACTIVE;
		pa12200001_set_mode(pa1->client, mode);
        clear_bit(PS_ACTIVE, &pa1->enable);
		irq_set_irq_wake(pa1->irq, 0);
		wake_unlock(&pa1->ps_wakelock);
    }
    pa1->ps_enable=((PS_ACTIVE & mode)>>1);
    return 0;
}

/*at least 500ms, "echo A > poll_delay" means poll A msec! */
//static ssize_t pa12200001_ps_poll_delay(struct sensors_classdev *sensors_cdev,
//				  unsigned int delay_msec)
//{
//	struct pa12200001_data *pa1 =
//	    container_of(sensors_cdev, struct pa12200001_data, ps_cdev);

//	if (delay_msec < PS_POLLING_RATE)	/* at least 100 ms */
//		delay_msec = PS_POLLING_RATE;

//    pa1->proximity_poll_delay = ns_to_ktime(delay_msec * NSEC_PER_MSEC);
//    if (PS_POLLING)
//    {
            /* Enable/Disable PS */
//            if (pa1->ps_enable)
//            {
//               hrtimer_start(&pa1->proximity_timer, pa1->proximity_poll_delay, HRTIMER_MODE_REL);
//            }
//    }
//
//	return 0;
//}


static DEVICE_ATTR(pthrd_param, S_IWUGO | S_IRUGO,
        pa12200001_show_pthrd_file, pa12200001_store_pthrd_file);

static ssize_t pa12200001_show_enable_ps_sensor(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);
    return sprintf(buf, "%d\n", (PS_ACTIVE & pa12200001_get_mode(data->client))?1:0);
}
static ssize_t pa12200001_store_enable_ps_sensor(struct device *pdev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);
    unsigned long val = simple_strtoul(buf, NULL, 10);

    if ((val != 0) && (val != 1)) {
        pr_err("invalid argument %ld\n", val);
        return count;
    }
	pr_info("enable = %ld\n", val);

	pa12200001_ps_set_enable(&data->ps_cdev, (unsigned int)val);

    return count;
}
static DEVICE_ATTR(ps_enable, S_IWUGO | S_IRUGO,
        pa12200001_show_enable_ps_sensor, pa12200001_store_enable_ps_sensor);

#ifdef USE_LIGHT_FEATURE
static ssize_t pa12200001_show_enable_als_sensor(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    return sprintf(buf, "%d\n", (ALS_ACTIVE & pa12200001_get_mode(data->client))?1:0);
}
static ssize_t pa12200001_store_enable_als_sensor(struct device *pdev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    unsigned long val = simple_strtoul(buf, NULL, 10);
    int mode=0;

    if ((val != 0) && (val != 1))
    {
        pr_err("invalid argument %ld\n", val);
        return count;
    }
	pr_info("enable = %ld\n", val);

    mode = pa12200001_get_mode(data->client);
    if (val == 1) {
        //turn on light  sensor
        if (data->pre_lux == 0){
            // in total darkness, force a first report
            input_report_abs(data->light_input_dev, ABS_LIGHT, 1);
            input_sync(data->light_input_dev);
            data->pre_lux = 1;
        }
        mode |= ALS_ACTIVE;
        set_bit(ALS_ACTIVE,&data->enable);
    } else {
        mode &= ~ALS_ACTIVE;
        clear_bit(ALS_ACTIVE, &data->enable);
    }
    pa12200001_set_mode(data->client, mode);

    return count;
}
static DEVICE_ATTR(als_enable, S_IWUGO | S_IRUGO,
        pa12200001_show_enable_als_sensor, pa12200001_store_enable_als_sensor);

static ssize_t pa12200001_show_als_poll_delay(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", 0);	// return in micro-second
}

static ssize_t pa12200001_store_als_poll_delay(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned long val = simple_strtoul(buf, NULL, 10);

    if (val < 5000)
        val = 5000;	// minimum 5ms

    //data->light_poll_delay=ns_to_ktime(val * NSEC_PER_MSEC);
    return count;
}

static DEVICE_ATTR(als_poll_delay, S_IWUSR | S_IRUGO,
        pa12200001_show_als_poll_delay, pa12200001_store_als_poll_delay);
#endif

static ssize_t pa12200001_show_ps(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    return sprintf(buf, "%d\n", pa12200001_get_ps_value(data->client));
}
static DEVICE_ATTR(ps, S_IRUGO, pa12200001_show_ps, NULL);

#ifdef USE_LIGHT_FEATURE
static ssize_t pa12200001_show_als(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    return sprintf(buf, "%d\n", pa12200001_get_als_value(data->client));
}
static DEVICE_ATTR(als, S_IRUGO, pa12200001_show_als, NULL);
#endif

static ssize_t pa12200001_show_reg(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
    int ret;
    int i = 0;
    int count = 0;
    u8 regdata = 0;
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    for (i = 0; i < 17; i++) {
        ret = i2c_read_reg(data->client, 0x00+i, &regdata);
        if (ret < 0)
            break;
        else
            count += sprintf(buf+count,"[%x]: %x\n", 0x00+i, regdata);
    }
    return count;
}
static ssize_t pa12200001_store_reg(struct device *pdev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int addr, cmd, ret;
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    if(2 != sscanf(buf, "%x %x", &addr, &cmd)) {
        pr_err("invalid format: '%s'\n", buf);
        return count;
    }

    ret = i2c_write_reg(data->client, addr, cmd);
    return count;
}
static DEVICE_ATTR(reg, S_IWUGO | S_IRUGO, pa12200001_show_reg, pa12200001_store_reg);

/* PS Calibration */
static ssize_t pa12200001_store_ps_calibration(struct device *pdev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val != 1) {
		pr_err("invalid argument %lu\n", val);
		return -EINVAL;
	}

    ret = pa12200001_run_calibration(data->client);
	if (ret < 0)
		return ret;

	pr_debug("ret = %d, count = %d\n", ret, count);
    return count;
}
static ssize_t pa12200001_show_ps_calibration(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);
    if(data->cal_result<0)
        return sprintf(buf, "%d\n",-1);

    return sprintf(buf, "%d\n", pa12200001_get_pscrosstalk(data->client));

}
static DEVICE_ATTR(pscalibration, S_IWUGO | S_IRUGO | S_IXUGO,
        pa12200001_show_ps_calibration, pa12200001_store_ps_calibration);

/* PS Threshold Calibration */
static ssize_t pa12200001_store_pthreshold_calibration(struct device *pdev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);
    pa12200001_thrd_calibration(data->client);
    return count;
}
static ssize_t pa12200001_show_pthreshold_calibration(struct device *pdev,
        struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);

    return sprintf(buf, "Low threshold = %d , High threshold = %d\n",
		data->ps_thrd_low,data->ps_thrd_high);

}
static DEVICE_ATTR(pthredcalibration, S_IWUGO | S_IRUGO,
        pa12200001_show_pthreshold_calibration, pa12200001_store_pthreshold_calibration);

static ssize_t pa12200001_store_dev_init(struct device *pdev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_dev = container_of(pdev, struct input_dev, dev);
	struct pa12200001_data *data = input_get_drvdata(input_dev);
    int ret;
    ret = pa12200001_init_client(data->client);
    return count;
}
static DEVICE_ATTR(dev_init, S_IWUGO | S_IRUGO, NULL, pa12200001_store_dev_init);

static void pa12200001_work_func_proximity(struct work_struct *work)
{
    struct pa12200001_data *data = container_of(work,
            struct pa12200001_data, work_proximity);
    int pval;

    pval = pa12200001_get_object(data->client);
	if (pval == data->ps_stat) {
		pr_info("ignore repeated event\n");
		return;
	}

	data->ps_stat = pval;

    input_report_abs(data->proximity_input_dev, ABS_DISTANCE, pval);
    input_sync(data->proximity_input_dev);

#ifdef USE_LIGHT_FEATURE
    if(ALS_POLLING & data->als_enable) //reschudule ALS polling work
    {
    }
#endif
}

#ifdef USE_LIGHT_FEATURE
static void pa12200001_work_func_light(struct work_struct *work)
{
    struct pa12200001_data *data = container_of(work, struct pa12200001_data, work_light);
    int aval;

    aval = pa12200001_get_lux_value(data->client);

	if (data->pre_lux == aval)
		return;
    //pr_debug("aval = %d\n", aval);

    input_report_abs(data->light_input_dev, ABS_LIGHT, aval);
    input_sync(data->light_input_dev);

	data->pre_lux = aval;

    if(PS_POLLING & data->ps_enable) //reschudule PS polling work
    {
    }
}
#endif

static void pa12200001_work_func_irq(struct work_struct *work)
{
    struct pa12200001_data *data = container_of(work, struct pa12200001_data, work_irq);
    u8 int_status;

    int_status = pa12200001_get_intstat(data->client);  //Get INT Status and
    pr_debug("IRQ Work INT status: %d\n", int_status);

#ifdef USE_LIGHT_FEATURE
    // Read ALS and Report
    if ((int_status & ALS_INT_ACTIVE) && !ALS_POLLING) {
        queue_work(data->wq, &data->work_light);
    }
#endif
    // Read PS and Report
    if (!PS_POLLING) {
        queue_work(data->wq, &data->work_proximity);
    }
}

/* assume this is ISR */
static irqreturn_t pa12200001_irq(int irq, void *info)
{
    struct i2c_client *client=(struct i2c_client *)info;
    struct pa12200001_data *data = i2c_get_clientdata(client);
    schedule_work(&data->work_irq); //pa12200001_work_func_irq()
    return IRQ_HANDLED;
}

#ifdef USE_LIGHT_FEATURE
/*assume this is timer*/
static enum hrtimer_restart pa12200001_light_timer_func(struct hrtimer *timer)
{
    struct pa12200001_data *data = container_of(timer, struct pa12200001_data, light_timer);

    queue_work(data->wq, &data->work_light);
    hrtimer_forward_now(&data->light_timer, data->light_poll_delay);
    return HRTIMER_RESTART;
}
#endif

static enum hrtimer_restart pa12200001_pxy_timer_func(struct hrtimer *timer)
{
    struct pa12200001_data *data = container_of(timer, struct pa12200001_data, proximity_timer);

    queue_work(data->wq, &data->work_proximity);
    hrtimer_forward_now(&data->proximity_timer, data->proximity_poll_delay);
    return HRTIMER_RESTART;
}
static int sensor_parse_dt(struct device *dev, struct pa12200001_data *data)
{
    struct device_node *np = dev->of_node;
    struct i2c_client *client;
    unsigned int tmp;
    int rc = 0;

    client = data->client;
    data->irq_gpio = of_get_named_gpio_flags(np, "pa12200001,irq-gpio",
            0, &data->irq_gpio_flags);
    if (data->irq_gpio < 0) {
        pr_err("invalid irq gpio\n");
        return -EINVAL;
    }

    rc = of_property_read_u32(np, "pa12200001,prox_th_min", &tmp);
    if (rc) {
        dev_err(dev, "Unable to read prox_th_min\n");
        return rc;
    }
    data->ps_thrd_low = tmp;

    rc = of_property_read_u32(np, "pa12200001,prox_th_max", &tmp);
    if (rc) {
        dev_err(dev, "Unable to read prox_th_max\n");
        return rc;
    }
    data->ps_thrd_high = tmp;

    return 0;
}
static void pa12200001_timer_init(struct pa12200001_data *data)
{
    if (PS_POLLING) {
        hrtimer_init(&data->proximity_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        data->proximity_poll_delay = ns_to_ktime(100 * NSEC_PER_MSEC);
        data->proximity_timer.function = pa12200001_pxy_timer_func;
    }

#ifdef USE_LIGHT_FEATURE
    if (ALS_POLLING) {
        hrtimer_init(&data->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        data->light_poll_delay = ns_to_ktime(100 * NSEC_PER_MSEC);
        data->light_timer.function = pa12200001_light_timer_func;
    }
#endif
}

static struct attribute *pa12200001_als_attributes[] = {
#ifdef USE_LIGHT_FEATURE
    &dev_attr_als_enable.attr,
    &dev_attr_als_poll_delay.attr,
    &dev_attr_als.attr,
#endif
    &dev_attr_reg.attr,
    &dev_attr_dev_init.attr,
    NULL
};
static const struct attribute_group pa12200001_als_attr_group = {
    .attrs = pa12200001_als_attributes,
};

static struct attribute *pa12200001_ps_attributes[] = {
    &dev_attr_ps_enable.attr,
    &dev_attr_ps.attr,
    &dev_attr_reg.attr,
    &dev_attr_pscalibration.attr,
    &dev_attr_pthredcalibration.attr,
    &dev_attr_dev_init.attr,
    &dev_attr_xtalk_param.attr,
    &dev_attr_pthrd_param.attr,
    NULL
};
static const struct attribute_group pa12200001_ps_attr_group = {
    .attrs = pa12200001_ps_attributes,
};

static int pa12200001_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int pa12200001_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long pa12200001_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int enable;
    int alspsdata;
    int mode;
    int ret = -1;

    if (pa12_i2c_client == NULL) {
        pr_err("i2c driver not installed\n");
        return -ENODEV;
    }

	pr_info("cmd = %u\n", _IOC_NR(cmd));
    switch (cmd) {
        case PA12_IOCTL_PS_ENABLE:
            ret = copy_from_user(&enable,(void __user *)arg, sizeof(enable));
            if (ret) {
                pr_err("PS enable copy data failed\n");
                return -EFAULT;
            }

            mode=pa12200001_get_mode(pa12_i2c_client);
            mode=((mode & 0x01) | (enable << 1));  //clear bit 1 then set enable status
            pa12200001_set_mode(pa12_i2c_client,mode);

            break;

        case PA12_IOCTL_PS_GET_DATA:
            alspsdata=pa12200001_get_ps_value(pa12_i2c_client);
            ret = copy_to_user((void __user *)arg,&alspsdata,sizeof(alspsdata));
            if (ret) {
                pr_err("PS Read data copy data failed\n");
                return -EFAULT;
            }
            break;

        case PA12_IOCTL_PS_CALIBRATION:
            alspsdata=pa12200001_run_calibration(pa12_i2c_client);
            ret = copy_to_user((void __user *)arg,&alspsdata,sizeof(alspsdata));
            if (ret) {
                pr_err("PS Calibration copy data failed\n");
                return -EFAULT;
            }
            break;

#ifdef USE_LIGHT_FEATURE
        case PA12_IOCTL_ALS_ENABLE:
            ret = copy_from_user(&enable,(void __user *)arg, sizeof(enable));
            if (ret) {
                pr_err("ALS enable copy data failed\n");
                return -EFAULT;
            }

            mode=pa12200001_get_mode(pa12_i2c_client);
            mode=((mode & 0x02) | enable);  //clear bit 0 then set enable status
            pa12200001_set_mode(pa12_i2c_client,mode);
            break;

        case PA12_IOCTL_ALS_GET_DATA:
            alspsdata=pa12200001_get_lux_value(pa12_i2c_client);
            ret = copy_to_user(&alspsdata,(void __user *)arg, sizeof(alspsdata));
            if (ret) {
                pr_err("ALS Read copy data failed\n");
                return -EFAULT;
            }
            break;
#endif

        default:
            break;
    }
    return 0;
}

static const struct file_operations pa12200001_fops = {
    .owner = THIS_MODULE,
    .open = pa12200001_open,
    .release = pa12200001_release,
    .unlocked_ioctl = pa12200001_ioctl,
};
static struct miscdevice pa12200001_alsps_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = MISC_DEV_NAME,
    .fops = &pa12200001_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/*Suspend/Resume*/
static void pa12200001_early_suspend(struct early_suspend *h)
{
}
static void pa12200001_late_resume(struct early_suspend *h)
{
}
static struct early_suspend pa12_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = pa12200001_early_suspend,
    .resume = pa12200001_late_resume,
};
#endif

int pa12200001_ps_state(void)
{
	int mode = 0;
	int pval = -1;
	struct pa12200001_data *data;
	u8 psdata;

	if (pa12_i2c_client)
		data = i2c_get_clientdata(pa12_i2c_client);
	else
		return -ENODEV;

	if (data && !data->ps_enable) {
		mode = pa12200001_get_mode(data->client);
		/* turn on ps */
		mode |= PS_ACTIVE;
		pa12200001_set_mode(data->client, mode);
		msleep(20);
		psdata = pa12200001_get_ps_value(data->client);
		if (psdata > data->ps_thrd_high)
			pval = 1;
		else if (psdata < data->ps_thrd_low)
			pval = 0;
		/* turn off ps */
		mode &= ~PS_ACTIVE;
		pa12200001_set_mode(data->client, mode);
	}
	pr_info("pval = %d\n", pval);
	return pval;
}
EXPORT_SYMBOL(pa12200001_ps_state);

static int pa12200001_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct pa12200001_data *data;
    struct pa12200001_platform_data *pdata=client->dev.platform_data;
    int err = 0;
    int ret = 0;

	pr_debug("enter, addr = 0x%x\n", client->addr);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
        pr_err("i2c check functionality error");
		return -ENODEV;
    }

    data = kzalloc(sizeof(struct pa12200001_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->client = client;
    data->pdata  = pdata;
    pa12_i2c_client = client;
    i2c_set_clientdata(client, data);

    err = sensor_parse_dt(&client->dev, data);
    if (err) {
		kfree(data);
		data = NULL;
        pr_err("parse dt failed\n");
        return err;
    }
    mutex_init(&data->lock);

    INIT_WORK(&data->work_proximity, pa12200001_work_func_proximity);	//PS  polling work regist
#ifdef USE_LIGHT_FEATURE
    INIT_WORK(&data->work_light, pa12200001_work_func_light);		//ALS  polling work regist
#endif
    INIT_WORK(&data->work_irq, pa12200001_work_func_irq);			//IRQ Work func

    pa12200001_timer_init(data);

#ifdef USE_LIGHT_FEATURE
    data->light_input_dev = input_allocate_device();
    if (!data->light_input_dev) {
        err = -ENOMEM;
        pr_err("allocate light input dev failed\n");
    }
    set_bit(EV_ABS, data->light_input_dev->evbit);
    input_set_drvdata(data->light_input_dev, data);
    data->light_input_dev->name = "light";

    input_set_capability(data->light_input_dev, EV_ABS, ABS_LIGHT);
    input_set_abs_params(data->light_input_dev, ABS_LIGHT, 0, 1, 0, 0);

    ret = input_register_device(data->light_input_dev);			//ALS INPUT regist
    if (ret < 0) {
        err = -ENOMEM;
        pr_err("register light input dev failed\n");
    }
#endif
    data->proximity_input_dev = input_allocate_device();
    if (!data->proximity_input_dev) {
        err = -ENOMEM;
        pr_err("alloc proximity input dev failed\n");
    }
    set_bit(EV_ABS, data->proximity_input_dev->evbit);
    input_set_drvdata(data->proximity_input_dev, data);
    data->proximity_input_dev->name = "proximity";
    //input_set_capability(data->proximity_input_dev, EV_ABS, ABS_DISTANCE);
    input_set_abs_params(data->proximity_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    ret = input_register_device(data->proximity_input_dev);			//PS INPUT regist
    if (ret < 0) {
        err = -ENOMEM;
        pr_err("register proximity input dev failed\n");
    }
	pr_debug("register input device done\n");

#ifdef USE_LIGHT_FEATURE
    if (!PS_POLLING || !ALS_POLLING)
#else
    if (!PS_POLLING)
#endif
    {
        if (gpio_is_valid(data->irq_gpio)) {
            err = gpio_request(data->irq_gpio, "pa12200001_irq_gpio");
            if (err) {
                pr_err("request irq gpio request failed\n");
                return -EINTR;
            }

            err = gpio_direction_input(data->irq_gpio);
            if (err) {
                pr_err("set irq gpio dir failed\n");
                return -EIO;
            }
        }

        data->irq = data->client->irq= gpio_to_irq(data->irq_gpio);

        err = request_irq(data->irq,pa12200001_irq,
                (PA12_INT_TYPE == 1) ? (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING) : IRQF_TRIGGER_FALLING,
                PA12200001_DRV_NAME,(void *)client);
        if (err)
            pr_err("request irq %d failed\n", data->irq);
        else {
			disable_irq(data->irq);
			data->irq_enabled = 0;
			pr_debug("register proximity irq done\n");
        }
    }

    data->wq = create_singlethread_workqueue("pa12200001_wq");
    if (!data->wq)
        pr_err("could not create workqueue\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
    /*register suspend/resume*/
    register_early_suspend(&pa12_early_suspend_desc);
#endif

    err = misc_register(&pa12200001_alsps_device);
    if (err) {
        pr_err("miscdev regist error\n");
    }

    /*Device Initialize*/
    err = pa12200001_init_client(client);
	if (err < 0) {
		pr_err("init device failed\n");
		goto dev_init_err;
	}

#ifdef USE_LIGHT_FEATURE
    ret = sysfs_create_group(&data->light_input_dev->dev.kobj,
			&pa12200001_als_attr_group);
    if (ret) {
        pr_err("could not create als sysfs group\n");
    }
#endif
    ret = sysfs_create_group(&data->proximity_input_dev->dev.kobj,
			&pa12200001_ps_attr_group);
    if (ret) {
        pr_err("could not create ps sysfs group\n");
    }

    /*Fast PS Calibration*/
    //if(PA12_FAST_CAL)
        //pa12200001_run_fast_calibration(client);

    data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = pa12200001_ps_set_enable;

//	if (PS_POLLING)
//	data->ps_cdev.sensors_poll_delay = pa12200001_ps_poll_delay;

    err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err)
		pr_err("register sensors class failed %d\n", err);

	input_report_abs(data->proximity_input_dev, ABS_DISTANCE, PA12_PS_FAR_DISTANCE);
	input_sync(data->proximity_input_dev);
	data->ps_stat = PA12_PS_FAR_DISTANCE;
	pr_notice("probe success\n");
    return 0;

dev_init_err:
#ifdef USE_LIGHT_FEATURE
    input_unregister_device(data->light_input_dev);
#endif
    input_unregister_device(data->proximity_input_dev);

#ifdef USE_LIGHT_FEATURE
    input_free_device(data->light_input_dev);
#endif
    input_free_device(data->proximity_input_dev);

	misc_deregister(&pa12200001_alsps_device);
	wake_lock_init(&data->ps_wakelock, WAKE_LOCK_SUSPEND, "pa12200001-ps-wakelock");

#ifdef USE_LIGHT_FEATURE
    if (!PS_POLLING || !ALS_POLLING)
#else
    if (!PS_POLLING)
#endif
    {
        free_irq(data->irq, client);
        gpio_free(data->irq_gpio);
    }

    destroy_workqueue(data->wq);
    mutex_destroy(&data->lock);

    kfree(data);
	data = NULL;
	return -ENODEV;
}

static int pa12200001_remove(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);

    if (pa12200001_get_mode(client)& PS_ACTIVE) {
        if (PS_POLLING)
            hrtimer_cancel(&data->proximity_timer);
        cancel_work_sync(&data->work_proximity);
    }

#ifdef USE_LIGHT_FEATURE
    if (pa12200001_get_mode(client) & ALS_ACTIVE) {
        if (ALS_POLLING)
		hrtimer_cancel(&data->light_timer);
        cancel_work_sync(&data->work_light);
    }

    input_unregister_device(data->light_input_dev);
#endif
    input_unregister_device(data->proximity_input_dev);

#ifdef USE_LIGHT_FEATURE
    input_free_device(data->light_input_dev);
#endif
    input_free_device(data->proximity_input_dev);

    misc_deregister(&pa12200001_alsps_device);

#ifdef USE_LIGHT_FEATURE
    if (!PS_POLLING || !ALS_POLLING)
#else
    if (!PS_POLLING)
#endif
    {
        free_irq(data->irq, client);
        gpio_free(data->irq_gpio);
    }

#ifdef USE_LIGHT_FEATURE
	sysfs_remove_group(&data->light_input_dev->dev.kobj, &pa12200001_als_attr_group);
#endif
	sysfs_remove_group(&data->proximity_input_dev->dev.kobj, &pa12200001_ps_attr_group);

    /* Power down the device */
    pa12200001_set_mode(client, 0);

    destroy_workqueue(data->wq);
    mutex_destroy(&data->lock);
	wake_lock_destroy(&data->ps_wakelock);

    kfree(data);
    return 0;
}

static const struct i2c_device_id pa12200001_id[] = {
    { PA12200001_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, pa12200001_id);

static struct of_device_id pa12200001_match_table[] = {
    {.compatible = "pa12200001",},
    {},
};

#if CONFIG_PM
static int pa12200001_pm_suspend(struct device *dev)
{
    return 0;
}
static int pa12200001_pm_resume(struct device *dev)
{
    return 0;
}
static const struct dev_pm_ops pa12200001_dev_pm_ops = {
	.suspend = pa12200001_pm_suspend,
	.resume = pa12200001_pm_resume,
};
#endif
static struct i2c_driver pa12200001_driver = {
    .driver = {
        .name	= PA12200001_DRV_NAME,
        .owner	= THIS_MODULE,
        .of_match_table = pa12200001_match_table,
#if CONFIG_PM
		.pm = &pa12200001_dev_pm_ops,
#endif
    },
    .probe	= pa12200001_probe,
    .remove	= pa12200001_remove,
    .id_table = pa12200001_id,
};

static int __init pa12200001_init(void)
{
    return i2c_add_driver(&pa12200001_driver);
}

static void __exit pa12200001_exit(void)
{
    i2c_del_driver(&pa12200001_driver);
}

MODULE_AUTHOR("Sensor Team, TXC");
MODULE_DESCRIPTION("PA12 ambient light + proximity sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(pa12200001_init);
module_exit(pa12200001_exit);
