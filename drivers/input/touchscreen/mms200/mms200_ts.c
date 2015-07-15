/*
 * MELFAS MMS200 Touchscreen Driver
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

#include "mms200_ts.h"
#include "mms200_l8720_firmware.h"
#include "../lct_tp_fm_info.h"
#include "../lct_ctp_upgrade.h"
#include "../lct_ctp_selftest.h"
#define __TP_X_Y_SWAP__		// xuke @ 20141031

#define COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4
#define VTG_MIN_UV		2850000
#define VTG_MAX_UV		2850000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000
#define __MMS_LCT_TEST  
static int esd_cnt;
static struct mms_ts_info *ts_info;
static unsigned char tp_fw[64*1024];

#if defined(CONFIG_FB)
static int mms_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#else
static void mms_ts_early_suspend(struct early_suspend *h);
static void mms_ts_late_resume(struct early_suspend *h);
#endif
/************************MMS log info**********************/
#define MMS_LOG_INFO     0
#if MMS_LOG_INFO
#define mmsprk(x,...)    printk("[winson] %d  %s "x,__LINE__,__func__,##__VA_ARGS__)
#else
#define mmsprk(x,...)    do {} while (0);
#endif
/**********************************************************/
static int mms_ts_config(struct mms_ts_info *info);

extern int is_tp_driver_loaded;
extern int get_fw_version(struct i2c_client *client, u8 *buf);

/* mms_ts_enable - wake-up func (VDD on)  */
static void mms_ts_enable(struct mms_ts_info *info)
{
	if (info->enabled)
		return;
	
	mutex_lock(&info->lock);
    mmsprk("mutex lock \n");
	gpio_direction_output(info->pdata->reset_gpio, 1);
	msleep(50);
	
	info->enabled = true;
	enable_irq(info->irq);

	mutex_unlock(&info->lock);
    mmsprk("mutext unlock\n");
}

/* mms_ts_disable - sleep func (VDD off) */
static void mms_ts_disable(struct mms_ts_info *info)
{
	if (!info->enabled)
		return;

	mutex_lock(&info->lock);
    mmsprk("mutext lock .\n");
	disable_irq(info->irq);

	msleep(50);
	gpio_direction_output(info->pdata->reset_gpio, 0);
	msleep(100);

	info->enabled = false;

	mutex_unlock(&info->lock);
    mmsprk("mutex unlock \n");
}

/* mms_reboot - IC reset */ 
//static void mms_reboot(struct mms_ts_info *info)
void mms_reboot(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_lock_adapter(adapter);

	msleep(20);
	gpio_direction_output(info->pdata->reset_gpio, 0);
	msleep(50);

	gpio_direction_output(info->pdata->reset_gpio, 1);
	msleep(20);

	i2c_unlock_adapter(adapter);
}

/*
 * mms_clear_input_data - all finger point release
 */
void mms_clear_input_data(struct mms_ts_info *info)
{
	int i;

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(info->input_dev);

	return;
}

/* mms_report_input_data - The position of a touch send to platfrom  */
void mms_report_input_data(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	int i;
	struct i2c_client *client = info->client;
	int id;
	int x;
	int y;
	int touch_major;
	int pressure;
	int key_code;
	int key_state;
	u8 *tmp;

	dev_dbg(&client->dev, "%s\n", __func__);
	if (buf[0] == MMS_NOTIFY_EVENT) {
		dev_info(&client->dev, "TSP mode changed (%d)\n", buf[1]);
		goto out;
	} else if (buf[0] == MMS_ERROR_EVENT) {
		dev_info(&client->dev, "Error detected, restarting TSP\n");
		mms_clear_input_data(info);
		mms_reboot(info);
		esd_cnt++;
		if (esd_cnt>= ESD_DETECT_COUNT)
		{
			i2c_smbus_write_byte_data(info->client, MMS_MODE_CONTROL, 0x04);
			esd_cnt =0;
		}
		goto out;
	}

	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = buf + i;
		dev_dbg(&client->dev, "%s, sz=%d, tmp=(0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n", 
			__func__, sz, tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
		esd_cnt =0;
		if (tmp[0] & MMS_TOUCH_KEY_EVENT) {
			dev_info(&client->dev, "%s, key\n", __func__);
			switch (tmp[0] & 0xf) {
			case 1:
				key_code = KEY_MENU;
				break;
			case 2:
				key_code = KEY_BACK;
				break;
			default:
				dev_err(&client->dev, "unknown key type\n");
				goto out;
				break;
			}

			key_state = (tmp[0] & 0x80) ? 1 : 0;
			input_report_key(info->input_dev, key_code, key_state);

		} else {
			dev_dbg(&client->dev, "%s, point\n", __func__);
			id = (tmp[0] & 0xf) -1;

			x = tmp[2] | ((tmp[1] & 0xf) << 8);
			y = tmp[3] | (((tmp[1] >> 4 ) & 0xf) << 8);
#if defined(__TP_X_Y_SWAP__)
			x = info->pdata->x_max - x;
			y = info->pdata->y_max - y;
#endif
			touch_major = tmp[4];
//			pressure = tmp[5];
			pressure = (tmp[0] & 0x80) ? 1 : 0;

			input_mt_slot(info->input_dev, id);
			
			if (!(tmp[0] & 0x80)) {
				dev_dbg(&client->dev, "%s, continue\n", __func__);
				input_report_key(info->input_dev, BTN_TOUCH, 0);
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
				pressure = 0;
                continue;
			} else {
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
				input_report_key(info->input_dev, BTN_TOUCH, 1);
				pressure = (tmp[5]>>1)+10;
			}
			
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
			dev_dbg(&client->dev, "%s, x=%d, y=%d, pressure=%d\n", __func__, x, y, pressure);
		}
	}

	dev_dbg(&client->dev, "%s, input_sync\n", __func__);
    input_mt_report_pointer_emulation(info->input_dev,false);
	input_sync(info->input_dev);

out:
	return;

}

/* mms_ts_interrupt - interrupt thread */
static irqreturn_t mms_ts_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[MAX_FINGER_NUM * FINGER_EVENT_SZ] = { 0, };
	int ret = 0;
	int error = 0;
	int sz;
	u8 reg = MMS_INPUT_EVENT;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
		},
	};
    mmsprk("start\n");
	dev_dbg(&client->dev, "%s\n", __func__);
 	//Read event packet size
	sz = i2c_smbus_read_byte_data(client, MMS_EVENT_PKT_SZ);
	
	//Check read error
	if(sz < 0){
		dev_err(&client->dev, "ERROR : failed to read event packet size - error code [%d]\n", sz);
		error = sz;
		sz = FINGER_EVENT_SZ;
	}
	//Check packet size
	if (sz > sizeof(buf)) {
		dev_err(&client->dev, "ERROR : event buffer overflow - buffer [%ld], packet [%d]\n", sizeof(buf), sz);
		error = -1;
		sz = FINGER_EVENT_SZ;
	}
	
	//Read event packet
	msg[1].len = sz;
	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	//Check event packet size
	if(error < 0){
		//skip
	}
	else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "ERROR : failed to read event packet - size [%d], error code [%d])\n", sz, ret);
	} 
	else {
		mms_report_input_data(info, sz, buf);
	}
	
	return IRQ_HANDLED;
}

#if MMS_USE_INIT_DONE
/*
 * mms_ts_input_open - Register input device after call this function 
 * this function is wait firmware flash wait
 */ 
static int mms_ts_input_open(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);
	int ret;

#if MMS_USE_INIT_DONE
	ret = wait_for_completion_interruptible_timeout(&info->init_done,
			msecs_to_jiffies(90 * MSEC_PER_SEC));
#endif

	if (ret > 0) {
		if (info->irq != -1) {
			mms_ts_enable(info);
			ret = 0;
		} else {
			ret = -ENXIO;
		}
	} else {
		dev_err(&dev->dev, "error while waiting for device to init\n");
		ret = -ENXIO;
	}

	return ret;
}

/*
 * mms_ts_input_close -If device power off state call this function
 */
static void mms_ts_input_close(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	mms_ts_disable(info);
}
#endif

int mms_fw_update_by_array_data(char *data, int size, void * context, bool force)
{
	struct mms_ts_info *info = context;
	char *pdata = data;
	int retires = 3, fw_size = size;
	int ret;
	bool force_update = force;

	printk("%s, fw_size=%d\n", __func__, fw_size);
	if (!pdata) {
		dev_err(&info->client->dev, "failed to read firmware\n");
		return -1;
	}

	do {
		ret = mms_flash_fw(info, pdata, fw_size, force_update);
		printk("%s, mms_flash_fw ret=%d, retires=%d\n", __func__, ret, retires);
	} while (ret && --retires);

	if (!retires) {
		dev_err(&info->client->dev, "failed to flash firmware after retires\n");
	}

	return ret;
}

void mms_fw_update_controller(const struct firmware *fw, void * context)
{
	struct mms_ts_info *info = context;
	int retires = 3;
	int ret;

	printk("%s, fw size=%ld\n", __func__, fw->size);
	if (!fw) {
		dev_err(&info->client->dev, "failed to read firmware\n");
#if MMS_USE_INIT_DONE
		complete_all(&info->init_done);
#endif
		return;
	}

	do {
		ret = mms_flash_fw(info,fw->data,fw->size,false);
	} while (ret && --retires);

	if (!retires) {
		dev_err(&info->client->dev, "failed to flash firmware after retires\n");
	}
	release_firmware(fw);
}
/*
 * mms_ts_config - f/w check download & irq thread register
 */
static int mms_ts_config(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int ret;
	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"mms_ts", info);

	if (ret) {
		dev_err(&client->dev, "failed to register irq\n");
		goto out;
	}
	disable_irq(client->irq);
	info->irq = client->irq;
	barrier();

  	info->tx_num = i2c_smbus_read_byte_data(client, MMS_TX_NUM);
	info->rx_num = i2c_smbus_read_byte_data(client, MMS_RX_NUM);
	info->key_num = i2c_smbus_read_byte_data(client, MMS_KEY_NUM);

	dev_info(&client->dev, "Melfas touch controller initialized\n");
	printk("%s, Melfas touch controller initialized\n", __func__);
	mms_reboot(info);
#if MMS_USE_INIT_DONE
	complete_all(&info->init_done);
#else
	enable_irq(client->irq);
#endif

out:
	return ret;
}

//melfas test mode


static ssize_t mms_force_update(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
        struct i2c_client *client = to_i2c_client(dev);
	struct file *fp; 
	mm_segment_t old_fs;
	size_t fw_size, nread;
	int error = 0;
	int result = 0;
 	disable_irq(client->irq);
	old_fs = get_fs();
	set_fs(KERNEL_DS);  
 
	fp = filp_open(EXTRA_FW_PATH, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		printk("%s: failed to open %s.\n", __func__, EXTRA_FW_PATH);
		error = -ENOENT;
		goto open_err;
	}
 	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (0 < fw_size) {
		unsigned char *fw_data;
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data,fw_size, &fp->f_pos);
		printk("%s: start, file path %s, size %ld Bytes\n", __func__,EXTRA_FW_PATH, fw_size);
		if (nread != fw_size) {
			    printk("%s: failed to read firmware file, nread %ld Bytes\n", __func__, nread);
		    error = -EIO;
		} else{
			result=mms_flash_fw(info,fw_data,fw_size, true); // last argument is full update
		}
		kfree(fw_data);
	}
 	filp_close(fp, current->files);
open_err:
	enable_irq(client->irq);
	set_fs(old_fs);
	return result;
}
static DEVICE_ATTR(fw_update, 0644, mms_force_update, NULL);

static struct attribute *mms_attrs[] = {
	&dev_attr_fw_update.attr,
	NULL,
};

static const struct attribute_group mms_attr_group = {
	.attrs = mms_attrs,
};

static int mms_ts_get_ts_version(struct mms_ts_info *info)
{
	struct mms_ts_info *pinfo = info;

	pinfo->fw_version[0] = i2c_smbus_read_byte_data(pinfo->client, MMS_FIRM_INFO);
	if (pinfo->fw_version[0] < 0)
		return -1;
	
	pinfo->fw_version[1] = i2c_smbus_read_byte_data(pinfo->client, MMS_MANUFACTURE_INFO);
	pinfo->fw_version[2] = i2c_smbus_read_byte_data(pinfo->client, MMS_CHIP_INFO);

	return 0;
}

static int ctp_upgrade_func(void)
{
	struct file* filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char *fileName = "/mnt/sdcard/CTP_FW.bin";
	char *fileName1 = "/storage/sdcard1/CTP_FW.bin";
	struct inode *inode;
	int fsize = 0;

	if(1)
	{	
		printk(KERN_INFO "[TP] %s: upgrade from file(%s) start!\n", __func__, fileName);
		// open file
		filp = filp_open(fileName, O_RDONLY, 0);
		if(IS_ERR(filp)) 
		{
			filp = filp_open(fileName1, O_RDONLY, 0);
			if(IS_ERR(filp)) 
			{
				//strcpy(ft_ctp_upgrade_status,"File no exist");
				lct_set_ctp_upgrade_status("File no exist");
				printk(KERN_ERR "[TP] %s: open firmware file failed\n", __func__);
				return -1;
			}
		}

		inode = filp->f_dentry->d_inode;
		fsize = inode->i_size;
		
		if(fsize > sizeof(tp_fw))
		{
			printk(KERN_ERR "[TP] %s: firmware size %d is too big\n", __func__,fsize);
			return -1;
		}
		
		oldfs = get_fs();
		set_fs(get_ds());
		
		// read the latest firmware binary file
		result=filp->f_op->read(filp,tp_fw,fsize, &filp->f_pos);
		if(result < 0) 
		{
			//strcpy(ft_ctp_upgrade_status,"File read err");
			lct_set_ctp_upgrade_status("File read err");
			printk(KERN_ERR "[TP] %s: read firmware file failed\n", __func__);
			return -1;
		}
			
		set_fs(oldfs);
		filp_close(filp, NULL);
			
		printk(KERN_INFO "[TP] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, tp_fw[0], tp_fw[1], tp_fw[2], tp_fw[3]);
			
	//	if(result > 0)
		{
			result = mms_fw_update_by_array_data(tp_fw, fsize, ts_info, true);
			if( result == 0)
			{
				lct_set_ctp_upgrade_status("Success");
				return 0;
			}
			else
			{
				lct_set_ctp_upgrade_status("failed");
			}
		}
	}
	lct_set_ctp_upgrade_status("failed");
	return -1;
}

static void ctp_upgrade_read_ver_func(char *ver)
{
	int cnt= 0;

	if(ver == NULL)
	{
		return;
	}

	mms_ts_get_ts_version(ts_info);

	cnt = sprintf(ver, "vid:0x%03x,fw:0x%03x,ic:%s\n",

		ts_info->fw_version[1], ts_info->fw_version[0], "melfas");
	return ;
}

static int touch_driver_power_on(struct i2c_client *client, bool on)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);
	struct mms_ts_platform_data *data = info->pdata;
	int rc = 0;

	if (!on)
		goto power_off;

	if (gpio_is_valid(data->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		gpio_set_value(data->power_ldo_gpio, 1);
	}
	else
	{
		printk("%s, regulator\n", __func__);
		rc = regulator_enable(data->avdd);
		if (rc)
		{
			dev_err(&client->dev, "Regulator avdd enable failed rc=%d\n", rc);
			return rc;
		}
	}

	if (data->i2c_pull_up)
	{
		rc = regulator_enable(data->vcc_i2c);
		if (rc)
		{
			dev_err(&client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
//			regulator_disable(data->avdd);
		}
	}
	return rc;

power_off:
	if (gpio_is_valid(data->power_ldo_gpio))
	{
		gpio_set_value(data->power_ldo_gpio, 0);
	}
	else
	{
		rc = regulator_disable(data->avdd);
		if (rc)
		{
			dev_err(&client->dev, "Regulator avdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	rc = regulator_disable(data->vcc_i2c);
	if (rc)
	{
		dev_err(&client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}

static int touch_driver_power_init(struct i2c_client *client, bool on)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);
	struct mms_ts_platform_data *data = info->pdata;
	int rc = 0;

	if (!on)
		goto pwr_deinit;
	
	if (gpio_is_valid(data->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		rc = gpio_request(data->power_ldo_gpio, "mms_ldo_gpio");
		if (rc)
		{
			printk("irq gpio request failed\n");
			goto Err_gpio_request;
		}
		
		rc = gpio_direction_output(data->power_ldo_gpio, 1);
		if (rc)
		{
			printk("set_direction for irq gpio failed\n");
			goto free_ldo_gpio;
		}
	}
	else
	{
		printk("%s, regulator\n", __func__);
		data->avdd = regulator_get(&client->dev, "avdd");
		if (IS_ERR(data->avdd))
		{
			rc = PTR_ERR(data->avdd);
			dev_err(&client->dev, "Regulator get failed avdd rc=%d\n", rc);
			goto Err_regulator_get;
		}

		if (regulator_count_voltages(data->avdd) > 0)
		{
			rc = regulator_set_voltage(data->avdd, VTG_MIN_UV, VTG_MAX_UV);
			if (rc)
			{
				dev_err(&client->dev, "Regulator set_vtg failed avdd rc=%d\n", rc);
				goto reg_avdd_put;
			}
		}
	}

	if (data->i2c_pull_up)
	{
		data->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(data->vcc_i2c))
		{
			rc = PTR_ERR(data->vcc_i2c);
			dev_err(&client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
			goto reg_avdd_set_vtg;
		}

		if (regulator_count_voltages(data->vcc_i2c) > 0)
		{
			rc = regulator_set_voltage(data->vcc_i2c, I2C_VTG_MIN_UV, I2C_VTG_MAX_UV);
			if (rc)
			{
				dev_err(&client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
				goto reg_vcc_i2c_put;
			}
		}
	}
	return 0;

reg_vcc_i2c_put:
	if (data->i2c_pull_up)
		regulator_put(data->vcc_i2c);
reg_avdd_set_vtg:
	if (!gpio_is_valid(data->power_ldo_gpio))
		if (regulator_count_voltages(data->avdd) > 0)
			regulator_set_voltage(data->avdd, 0, VTG_MAX_UV);
reg_avdd_put:
free_ldo_gpio:	
	if (gpio_is_valid(data->power_ldo_gpio))
		gpio_free(data->power_ldo_gpio);
	else
		regulator_put(data->avdd);
Err_regulator_get:
Err_gpio_request:
	return rc;
pwr_deinit:
	if (gpio_is_valid(data->power_ldo_gpio))
		gpio_free(data->power_ldo_gpio);
	else
	{
		if (regulator_count_voltages(data->avdd) > 0)
			regulator_set_voltage(data->avdd, 0, VTG_MAX_UV);
		regulator_put(data->avdd);
	}

	if (data->i2c_pull_up)
	{
		if (regulator_count_voltages(data->vcc_i2c) > 0)
			regulator_set_voltage(data->vcc_i2c, 0, I2C_VTG_MAX_UV);
		regulator_put(data->vcc_i2c);
	}
	return 0;
}

static int touch_driver_pinctrl_init(struct i2c_client *client)
{
	int retval;
	struct mms_ts_info *info = i2c_get_clientdata(client);
	struct mms_ts_platform_data *data = info->pdata;
	
	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->ts_pinctrl))
	{
		dev_dbg(&client->dev, "Target does not use pinctrl\n");
		retval = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return retval;
	}
    
	data->gpio_state_active	= pinctrl_lookup_state(data->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active))
	{
		printk("%s Can not get ts default pinstate\n", __func__);
		retval = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend = pinctrl_lookup_state(data->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend))
	{
		dev_err(&client->dev,	"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int touch_driver_pinctrl_select(struct i2c_client *client, bool on)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);
	struct mms_ts_platform_data *data = info->pdata;
	struct pinctrl_state *pins_state;
	int ret;
	
	pins_state = on ? data->gpio_state_active : data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state))
	{
		ret = pinctrl_select_state(data->ts_pinctrl, pins_state);
		if (ret)
		{
			dev_err(&client->dev, "can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	}
	else
	{
		dev_err(&client->dev,	"not a valid '%s' pinstate\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}

#ifdef CONFIG_OF
static int touch_driver_get_dt_coords(struct device *dev, char *name, struct mms_ts_info *data)
{
	u32 coords[COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	struct mms_ts_platform_data *pdata = data->pdata;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != COORDS_ARR_SIZE)
	{
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL))
	{
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "melfas,panel-coords"))
	{
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	}
	else if (!strcmp(name, "melfas,display-coords"))
	{
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}
	else
	{
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int touch_driver_parse_dt(struct device *dev, struct mms_ts_info *data)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	struct mms_ts_platform_data *pdata = data->pdata;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	printk("%s\n", __func__);
	rc = touch_driver_get_dt_coords(dev, "melfas,panel-coords", data);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = touch_driver_get_dt_coords(dev, "melfas,display-coords", data);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np, "melfas,i2c-pull-up");
	pdata->fw_auto_update = of_property_read_bool(np, "melfas,fw-auto-update");
	pdata->no_force_update = of_property_read_bool(np, "melfas,no-force-update");
	
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "melfas,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	else
		printk("%s, reset_gpio=%d\n", __func__, pdata->reset_gpio);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "melfas,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	else
		printk("%s, irq_gpio=%d\n", __func__, pdata->irq_gpio);

	/* power ldo gpio info*/
	pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "melfas,power_ldo-gpio", 0, &pdata->power_ldo_gpio_flags);
	if (pdata->power_ldo_gpio < 0)
#if 0
		return pdata->power_ldo_gpio;
#else
		printk("%s, power_ldo_gpio=%d\n", __func__, pdata->power_ldo_gpio);
#endif

	prop = of_find_property(np, "melfas,button-map", NULL);
	if (prop)
	{
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np, "melfas,button-map", button_map, num_buttons);
		if (rc)
		{
			dev_err(dev, "Unable to read key codes\n");
			return -EINVAL;
		}
	}

	return 0;
}
#else
static int touch_driver_parse_dt(struct device *dev, struct mms_ts_info *pdata)
{
	return -ENODEV;
}
#endif
int mms200_self_test(void){
    struct mms_ts_info *info = ts_info;
    struct i2c_client *client = info->client;
    dev_info(&info->client->dev, "cm delta Test\n");

    info->tx_num = i2c_smbus_read_byte_data(client, MMS_TX_NUM);
    info->rx_num = i2c_smbus_read_byte_data(client, MMS_RX_NUM);
    info->key_num = i2c_smbus_read_byte_data(client, MMS_KEY_NUM);
    info->self_test_val=0;
    if(get_cm_test_init(info)){
        dev_info(&info->client->dev, "Failed\n");
        return -EAGAIN;
    }
    if(get_cm_delta(info)){
        return -EAGAIN;
    }
    if(get_cm_test_exit(info)){
        return -EAGAIN;
    }

    //ret = snprintf(buf,PAGE_SIZE,"%s\n",info->get_data);
    memset(info->get_data,0,4096);
    printk("mms self test val : %d .\n",info->self_test_val);
    return info->self_test_val;
}
static int mms_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
	const char *fw_name = FW_NAME;
	char fw_version[32];

	printk("%s, dev=%s, is_tp_driver_loaded=%d\n", __func__, dev_name(&client->dev), is_tp_driver_loaded);

	if(is_tp_driver_loaded == 1)
	{
		printk("%s, other driver has been loaded\n", __func__);
		return -ENODEV;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

    if (client->dev.of_node)
	{
		info = devm_kzalloc(&client->dev, sizeof(struct mms_ts_info), GFP_KERNEL);
		if (!info) {
			dev_err(&client->dev, "Failed to allocated mms_ts_info\n");
			return -ENOMEM;
		}
		
		info->pdata = devm_kzalloc(&client->dev, sizeof(struct mms_ts_platform_data), GFP_KERNEL);
		if (!info) {
			dev_err(&client->dev, "Failed to allocated mms_ts_platform_data\n");
			goto Err_kzalloc_pdata;
		}
		
		ret = touch_driver_parse_dt(&client->dev, info);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			goto Err_parse_dt;
		}
    }

	i2c_set_clientdata(client, info);
	ts_info = info;
	
	ret = touch_driver_pinctrl_init(client);
	if (!ret && info->pdata->ts_pinctrl)
	{
		ret = touch_driver_pinctrl_select(client, true);
		if (ret < 0)
			goto Err_pinctrl_init;
	}
	
	ret = touch_driver_power_init(client, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto Err_power_init;
	}

	ret = touch_driver_power_on(client, true);
	if (ret) {
		dev_err(&client->dev, "power on failed");
		goto Err_power_on;
	}

	if (gpio_is_valid(info->pdata->reset_gpio))
	{
	    ret = gpio_request(info->pdata->reset_gpio, "mms_reset");
	    if (ret)
	    {
	        pr_err("*** Failed to request GPIO %d, error %d ***\n", info->pdata->reset_gpio, ret);
	        goto Err_gpio_reset;
	    }

	    // power on TP
	    ret = gpio_direction_output(info->pdata->reset_gpio, 1);
		if (ret)
		{
			dev_err(&client->dev, "set_direction for reset gpio failed\n");
			goto Err_reset_output;
		}
	}
	
	if (gpio_is_valid(info->pdata->irq_gpio))
	{
	    ret = gpio_request(info->pdata->irq_gpio, "mms_interrupt");
	    if (ret < 0)
	    {
	        pr_err("*** Failed to request GPIO %d, error %d ***\n", info->pdata->irq_gpio, ret);
	        goto Err_gpio_irq;
	    }
		
	    ret = gpio_direction_input(info->pdata->irq_gpio);
		if (ret)
		{
			dev_err(&client->dev, "set_direction for irq gpio failed\n");
			goto Err_irq_input;
		}
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocated memory\n");
		goto Err_input_allocate;
	}

	client->irq = gpio_to_irq(info->pdata->irq_gpio);
	printk("%s, irq=%d\n", __func__, client->irq);
	info->client = client;
	info->input_dev = input_dev;
//	info->pdata = client->dev.platform_data;
#if MMS_USE_INIT_DONE
	init_completion(&info->init_done);
#endif
	info->irq = -1;

	mutex_init(&info->lock);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);

	snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));

	input_dev->name = "Melfas MMS200 Touch Screen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
#if MMS_USE_INIT_DONE
	input_dev->open = mms_ts_input_open;
	input_dev->close = mms_ts_input_close;
#endif
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
    __set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->pdata->y_max, 0, 0);

	input_set_drvdata(input_dev, info);

#if MMS_HAS_TOUCH_KEY
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
#endif

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "failed to register input dev\n");
		goto Err_input_register;
	}
	
	input_set_drvdata(input_dev, info);

	mms_reboot(info);

#if 1
	ret = mms_ts_get_ts_version(info);
	if (ret < 0)
//		goto Err_get_version;
		return -ENODEV;
	printk("%s, fw_version=0x%02x, manufacture=0x%02x, chip=0x%02x\n", __func__, 
		info->fw_version[0], info->fw_version[1], info->fw_version[2]);
#endif

	info->fw_name = kstrdup(fw_name, GFP_KERNEL);

#if 1		// xuke @ 20141124
	printk("%s, fw_auto_update=%d, no_force_update=%d\n", __func__, 
		info->pdata->fw_auto_update, info->pdata->no_force_update);
	if (info->pdata->fw_auto_update)
	{
		ret= mms_fw_update_by_array_data(MELFAS_MFSB, MELFAS_MFSB_Size, 
					info, !info->pdata->no_force_update);
		if (ret) {
			dev_err(&client->dev, "failed to schedule firmware update\n");
			goto Err_request_firmware;
		}
	}
#else
	ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &client->dev,
					GFP_KERNEL, info, mms_fw_update_controller);
	printk("%s, request_firmware_nowait ret=%d\n", __func__, ret);
	if (ret) {
		dev_err(&client->dev, "failed to schedule firmware update\n");
		goto Err_request_firmware;
	}
#endif

#ifdef SUPPORT_READ_TP_VERSION
	memset(fw_version, 0, sizeof(fw_version));
	sprintf(fw_version, "fw:v%02d, ic:%s\n", info->fw_version[0], "MMS252");
	init_tp_fm_info(info->fw_version[0], fw_version, "Melfas");
#endif

	lct_ctp_upgrade_int(ctp_upgrade_func, ctp_upgrade_read_ver_func);

	mms_ts_config(info);

	esd_cnt = 0;
#if defined(CONFIG_FB)
	info->mms_fb_notif.notifier_call = mms_fb_notifier_callback;
	ret = fb_register_client(&info->mms_fb_notif);
	if (ret)
	{
		printk("Unable to register fb_notifier: %d\n", ret);
		goto Err_fb_register;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = mms_ts_early_suspend;
	info->early_suspend.resume = mms_ts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif
#ifdef __MMS_LCT_TEST
        //mutex_init(&data->selftest_lock);
    lct_ctp_selftest_int(mms200_self_test);
#endif

#ifdef __MMS_LOG_MODE__
	if(mms_ts_log(info)){
		dev_err(&client->dev, "failed to create Log mode.\n");
		goto Err_ts_log;
	}

//	info->class = class_create(THIS_MODULE, "mms_ts");
//	device_create(info->class, NULL, info->mms_dev, NULL, "mms_ts");
#endif

#ifdef __MMS_TEST_MODE__
	if (mms_sysfs_test_mode(info)){
		dev_err(&client->dev, "failed to create sysfs test mode\n");
		goto Err_test_mode;
	}

#endif
	if (sysfs_create_group(&client->dev.kobj, &mms_attr_group)) {
		dev_err(&client->dev, "failed to create sysfs group\n");
		goto Err_create_group;
	}

	if (sysfs_create_link(NULL, &client->dev.kobj, "mms_ts")) {
		dev_err(&client->dev, "failed to create sysfs symlink\n");
		goto Err_create_link;
	}

	dev_notice(&client->dev, "mms dev initialized\n");
	printk("%s done\n", __func__);

	return 0;

Err_create_link:
	sysfs_remove_group(&client->dev.kobj, &mms_attr_group);
Err_create_group:
#ifdef __MMS_TEST_MODE__
Err_test_mode:
#endif
#ifdef __MMS_LOG_MODE__
Err_ts_log:
#endif
#if defined(CONFIG_FB)
	fb_unregister_client(&info->mms_fb_notif);
#endif
Err_fb_register:
Err_request_firmware:
//Err_get_version:
	kfree(info->fw_name);
Err_input_register:
	input_mt_destroy_slots(input_dev);
	mutex_destroy(&info->lock);
	input_free_device(input_dev);
Err_input_allocate:
Err_irq_input:
	if (gpio_is_valid(info->pdata->irq_gpio))
		gpio_free(info->pdata->irq_gpio);
Err_gpio_irq:
Err_reset_output:
	if (gpio_is_valid(info->pdata->reset_gpio))
		gpio_free(info->pdata->reset_gpio);
Err_gpio_reset:
	touch_driver_power_on(client, false);
Err_power_on:
	touch_driver_power_init(client, false);
Err_power_init:
	if (info->pdata->ts_pinctrl)
		if (touch_driver_pinctrl_select(client, false) < 0)
			pr_err("Cannot get idle pinctrl state\n");
Err_pinctrl_init:
	if (info->pdata->ts_pinctrl)
		pinctrl_put(info->pdata->ts_pinctrl);
	i2c_set_clientdata(client, NULL);
Err_parse_dt:
	devm_kfree(&client->dev, info->pdata);
Err_kzalloc_pdata:
	devm_kfree(&client->dev, info);
	
	printk("%s err, ret=%d\n", __func__, ret);
	return ret;
}

static int mms_ts_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0)
		free_irq(info->irq, info);
#ifdef __MMS_TEST_MODE__
	sysfs_remove_group(&info->client->dev.kobj, &mms_attr_group);
	mms_sysfs_remove(info);
	sysfs_remove_link(NULL, "mms_ts");
	kfree(info->get_data);
#endif

#ifdef __MMS_LOG_MODE__
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
#endif

	input_unregister_device(info->input_dev);
#if defined(CONFIG_FB)
	fb_unregister_client(&info->mms_fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&info->early_suspend);
#endif
	kfree(info->fw_name);
	devm_kfree(&client->dev, info);

	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int mms_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	printk("%s\n", __func__);
	mutex_lock(&info->input_dev->mutex);
    mmsprk("mutex locked \n");
	if (info->input_dev->users) {
		mms_ts_disable(info);
		mms_clear_input_data(info);
	}

	mutex_unlock(&info->input_dev->mutex);
    mmsprk("mutex unlocked \n");
	return 0;

}

static int mms_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	printk("%s\n", __func__);
	mutex_lock(&info->input_dev->mutex);
    mmsprk("mutext lock\n");
	if (info->input_dev->users)
		mms_ts_enable(info);

	mutex_unlock(&info->input_dev->mutex);
    mmsprk("mutex unlock \n");
	return 0;
}
#endif

#if defined(CONFIG_FB)
static int mms_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
	struct mms_ts_info *info = container_of(self, struct mms_ts_info, mms_fb_notif);
    int *blank;

    printk("%s\n", __func__);
    if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK )
    {
        blank = evdata->data;
        printk("blank = %d\n",*blank);
		
        if (*blank == FB_BLANK_UNBLANK)
        {
            mms_ts_resume(&info->client->dev);
        }
        else if (*blank == FB_BLANK_POWERDOWN)
        {
            mms_ts_suspend(&info->client->dev);
        }
    }

    return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_ts_early_suspend(struct early_suspend *h)
{
	struct mms_ts_info *info;
	info = container_of(h, struct mms_ts_info, early_suspend);
	mms_ts_suspend(&info->client->dev);
}

static void mms_ts_late_resume(struct early_suspend *h)
{
	struct mms_ts_info *info;
	info = container_of(h, struct mms_ts_info, early_suspend);
	mms_ts_resume(&info->client->dev);
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops mms_ts_pm_ops = {
	.suspend	= mms_ts_suspend,
	.resume		= mms_ts_resume,
};
#endif

static const struct i2c_device_id mms_ts_id[] = {
	{"mms_ts", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mms_ts_id);

#ifdef CONFIG_OF
static struct of_device_id mms_match_table[] = {
	{ .compatible = "melfas,mms_ts",},
	{ },
};
#else
#define mms_match_table NULL
#endif

static struct i2c_driver mms_ts_driver = {
	.probe		= mms_ts_probe,
	.remove		= mms_ts_remove,
	.driver		= {
				.name	= "mms_ts",
				.owner = THIS_MODULE,
				.of_match_table = mms_match_table,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
				.pm	= &mms_ts_pm_ops,
#endif
	},
	.id_table	= mms_ts_id,
};

static int __init mms_ts_init(void)
{
	printk("%s\n", __func__);
	return i2c_add_driver(&mms_ts_driver);
}

static void __exit mms_ts_exit(void)
{
	return i2c_del_driver(&mms_ts_driver);
}

module_init(mms_ts_init);
module_exit(mms_ts_exit);

MODULE_VERSION("2014.08.06");
MODULE_DESCRIPTION("MELFAS MMS200 Touchscreen Driver");
MODULE_LICENSE("GPL");

