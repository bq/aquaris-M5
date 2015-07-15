#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <asm/div64.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include "it7236_touchkey.h"
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#define IT7280_FW_AUTO_UPGRADE    1

#if IT7280_FW_AUTO_UPGRADE
#include "IT7236_upgrade.h"
#endif



#define IT7280_I2C_NAME "it7236_touchkey"

#define ITE_VTG_MIN_UV		2700000
#define ITE_VTG_MAX_UV		3300000
#define ITE_I2C_VTG_MIN_UV	1800000
#define ITE_I2C_VTG_MAX_UV	1800000

#define TOUCHKEY_LEFT		195
#define TOUCHKEY_RIGHT		196

static int ite7280_major = 0;	// dynamic major by default
static int ite7280_minor = 0;
static struct cdev ite7280_cdev;
static struct class *ite7280_class = NULL;
static dev_t ite7280_dev;
static struct input_dev *input_dev;
static int fw_upgrade_success = 0;
static u8 wTemp[128] = {0x00};
static char config_id[10];
static char key_down = 0;
//static char fih_touch_buildid[20];
static char it7236_fw_name[256] = "it7236.fw";

static int get_config_ver(void);
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ts_early_suspend(struct early_suspend *es);
static void ts_late_resume(struct early_suspend *es);
#endif

#if 0
static int fih_touch_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len;

	snprintf(fih_touch_buildid, PAGE_SIZE, "ITE-V%d.%d.%d%d\n", config_id[0], config_id[1],config_id[2], config_id[3]);
	len = snprintf(page, PAGE_SIZE, "%s", fih_touch_buildid);
	return len;
}
#endif
#if 0
static int fih_init(void)
{
	if (create_proc_read_entry("AllHWList/TouchKey", 0, NULL, fih_touch_read_proc, NULL) == NULL) {
		proc_mkdir("AllHWList", NULL);
		if (create_proc_read_entry("AllHWList/TouchKey", 0, NULL, fih_touch_read_proc, NULL) == NULL) {
			pr_err("fail to create proc/AllHWList/TouchKey\n");
			return -1;
		}
	}
	return 0;
}
#endif
static int i2cReadFromIt7280(struct i2c_client *client, unsigned char bufferIndex, unsigned char dataBuffer[], unsigned short dataLength)
{
	int ret, retry = 3;
	struct i2c_msg msgs[2] = { {
		.addr = client->addr,
		.flags = 0,
		.len = 1,
		.buf = &bufferIndex
		}, {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = dataLength,
		.buf = dataBuffer
		}
	};
	memset(dataBuffer, 0x56, dataLength);

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
		retry--;
	} while((ret != 2) && (retry > 0));

	if(ret != 2) {
		pr_err("%s : i2c_transfer error\n", __func__);
		return -1;
	}
	return 0;
}

static int i2cWriteToIt7280(struct i2c_client *client, unsigned char bufferIndex, unsigned char const dataBuffer[], unsigned short dataLength)
{
	unsigned char buffer4Write[256];
	int ret, retry = 3;
	struct i2c_msg msgs[1] = { {
		.addr = client->addr,
		.flags = 0,
		.len = dataLength + 1,
		.buf = buffer4Write
		}
	};

	if(dataLength < 256) {
		buffer4Write[0] = bufferIndex;
		memcpy(&(buffer4Write[1]), dataBuffer, dataLength);

		do {
			ret = i2c_transfer(client->adapter, msgs, 1);
			retry--;
		} while((ret != 1) && (retry > 0));

		if(ret != 1)
			pr_err("%s : i2c_transfer error\n", __func__);
		return ret;
	}
	else {
		pr_err("%s : i2c_transfer error , out of size\n", __func__);
		return -1;
	}
}

static bool waitCommandDone(void)
{
	unsigned char ucQuery = 0x00;
	unsigned int count = 0;

	do {
		if(!i2cReadFromIt7280(gl_ts->client, 0xFA, &ucQuery, 1))
			ucQuery = 0x00;
		count++;
	} while((ucQuery != 0x80) && (count < 10));

	if( ucQuery == 0x80)
		return  true;
	else
		return  false;
}

static bool fnFirmwareReinitialize(void)
{
	int count = 0;
	u8 data[1];
	char pucBuffer[1];

	pucBuffer[0] = 0xFF;
	i2cWriteToIt7280(gl_ts->client, 0xF6, pucBuffer, 1);

	pucBuffer[0] = 0x64;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	pucBuffer[0] = 0x3F;
	i2cWriteToIt7280(gl_ts->client, 0x00, pucBuffer, 1);
	pucBuffer[0] = 0x7C;
	i2cWriteToIt7280(gl_ts->client, 0x01, pucBuffer, 1);
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0x00, pucBuffer, 1);
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0x01, pucBuffer, 1);

	do{
		i2cReadFromIt7280(gl_ts->client, 0xFA, data, 1);
		count++;
	} while( (data[0] != 0x80) && (count < 20));

	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);

	return true;
}

static ssize_t IT7280_upgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s not ready\n", __func__);
}

static int it7236_upgrade(u8* InputBuffer, int fileSize)
{
	int i, j, k;
	int StartPage = 0;
	int registerI, registerJ;
	int page, pageres;
	int Addr;
	int nErasePage;
	u8 result = 1;
	u8 DATABuffer1[128] = {0x00};
	u8 DATABuffer2[128] = {0x00};
	u8 OutputDataBuffer[8192] = {0x00};
	u8 WriteDATABuffer[128] = {0x00};
	char pucBuffer[1];
	int retry = 0;
	int ret1, ret2;

	//Request Full Authority of All Registers
	pucBuffer[0] = 0x01;
	i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);

	// 1. Assert Reset of MCU
	pucBuffer[0] = 0x64;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	pucBuffer[0] = 0x04;
	i2cWriteToIt7280(gl_ts->client, 0x01, pucBuffer, 1);

	// 2. Assert EF enable & reset
	pucBuffer[0] = 0x10;
	i2cWriteToIt7280(gl_ts->client, 0x2B, pucBuffer, 1);
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);

	// 3. Test Mode Enable
	pucBuffer[0] = 0x07;
	gl_ts->client->addr = 0x7F;
	i2cWriteToIt7280(gl_ts->client, 0xF4, pucBuffer, 1);
	gl_ts->client->addr = 0x46;

	nErasePage = fileSize/256;
	if(fileSize % 256 == 0)
		nErasePage -= 1;

	// 4. EF HVC Flow (Erase Flash)
	for( i = 0 ; i < nErasePage + 1 ; i++ ){
		// EF HVC Flow
		pucBuffer[0] = i+StartPage;
		i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);	//	Select axa of EF E/P Mode

		pucBuffer[0] = 0x00;
		i2cWriteToIt7280(gl_ts->client, 0xF9, pucBuffer, 1);	// Select axa of EF E/P Mode Fail
		pucBuffer[0] = 0xB2;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Select efmode of EF E/P Mode(all erase)
		pucBuffer[0] = 0x80;
		i2cWriteToIt7280(gl_ts->client, 0xF9, pucBuffer, 1);	// Pump Enable
		mdelay(10);
		pucBuffer[0] = 0xB6;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Write EF CHVPL Mode Cmd
		pucBuffer[0] = 0x00;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Write EF Standby Mode Cmd
	}

	// 5. EFPL Flow - Write EF
	for (i = 0; i < fileSize; i += 256)
	{
		pucBuffer[0] = 0x05;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Write EF PL Mode Cmd

		//	Write EF Data - half page(128 bytes)
		for(registerI = 0 ; registerI < 128; registerI++)
		{
			if(( i + registerI ) < fileSize ) {
				DATABuffer1[registerI] = InputBuffer[i+registerI];
			}
			else {
				DATABuffer1[registerI] = 0x00;
			}
		}
		for(registerI = 128 ; registerI < 256; registerI++)
		{
			if(( i + registerI ) < fileSize ) {
				DATABuffer2[registerI - 128] = InputBuffer[i+registerI];
			}
			else {
				DATABuffer2[registerI - 128] = 0x00;
			}
		}
		registerJ = i & 0x00FF;
		page = ((i & 0x3F00)>>8) + StartPage;
		pageres = i % 256;
		retry = 0;
		do {
			pucBuffer[0] = page;
			i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
			i2cReadFromIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
			retry++;
		} while((pucBuffer[0] != page) && (retry < 10));

		/* write 256 bytes once */
		retry = 0;

		gl_ts->client->addr = 0x7F;
		do {
			ret1 = i2cWriteToIt7280(gl_ts->client, 0x00, DATABuffer1, 128);
			ret2 = i2cWriteToIt7280(gl_ts->client, 0x00 + 128, DATABuffer2, 128);
			retry++;
		} while(((ret1 * ret2) != 1) && (retry < 10));
		gl_ts->client->addr = 0x46;

		pucBuffer[0] = 0x00;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Write EF Standby Mode Cmd
		pucBuffer[0] = 0x00;
		i2cWriteToIt7280(gl_ts->client, 0xF9, pucBuffer, 1);	// Select axa of EF E/P Mode Fail
		pucBuffer[0] = 0xE2;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Select efmode of EF E/P Mode(all erase)
		pucBuffer[0] = 0x80;
		i2cWriteToIt7280(gl_ts->client, 0xF9, pucBuffer, 1);	// Pump Enable
		mdelay(10);
		pucBuffer[0] = 0xE6;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Write EF CHVPL Mode Cmd
		pucBuffer[0] = 0x00;
		i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);	// Write EF Standby Mode Cmd
	}

	// 6. Page switch to 0
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);

	// 7. Write EF Read Mode Cmd
	pucBuffer[0] = 0x01;
	i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);		// Write EF Standby Mode Cmd

	// 8. Read EF Data, Compare the firmware and input data. for j loop
	for ( j = 0; j < fileSize; j+=128)
	{
		page = ((j & 0x3F00)>>8) + StartPage;					// 3F = 0011 1111, at most 32 pages
		pageres = j % 256;
		pucBuffer[0] = page;
		i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);

		gl_ts->client->addr = 0x7F;
		i2cReadFromIt7280(gl_ts->client, 0x00 + pageres, wTemp, 128); // use 0x7f to read data
		gl_ts->client->addr = 0x46;
		// Compare Flash Data
		for( k = 0 ; k < 128 ; k++ )
		{
			if( j+k >= fileSize )
				break;

			pageres = (j + k) % 256;
			OutputDataBuffer[j+k] = wTemp[k];
			WriteDATABuffer[k] = InputBuffer[j+k];

			if(OutputDataBuffer[j+k] != WriteDATABuffer[k])
			{
				Addr = page << 8 | pageres;
				pr_err("Addr: %04x, Expected: %02x, Read: %02x\r\n", Addr, WriteDATABuffer[k], wTemp[k]);
				result = 0;
			}
		}
	}

	if(!result)
	{
		fnFirmwareReinitialize();
		pr_err("[IT7236] : failed to upgrade firmware\n\n");
		fw_upgrade_success = 1;
		return -1;
	}

	// 9. Write EF Standby Mode Cmd
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);
	// 10. Power On Reset
	fnFirmwareReinitialize();

	pr_info("[IT7236] : success to upgrade firmware\n\n");
	get_config_ver();
	fw_upgrade_success = 0;
	return 0;
}
#if 1
static ssize_t IT7280_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, retry = 5;
	const struct firmware *fw = NULL;
	int fileSize;
	u8* InputBuffer = kzalloc(0x8000, GFP_KERNEL);

	ret = request_firmware( &fw, it7236_fw_name, dev);
	if (ret < 0) {
		pr_info("[IT7236] : open %s failed\n", it7236_fw_name);
		return -EINVAL;
	}
	else
		memcpy(InputBuffer ,fw->data, fw->size);

	fileSize = fw->size;

	pr_info("[IT7236]: fw_ver : %c, %c, %c, %c\n",*(fw->data + 0x0406), *(fw->data + 0x0407), *(fw->data + 0x0408), *(fw->data + 0x0409));
	pr_info("[IT7236]: --------------------- fw_size = %ld\n", fw->size);

	while(((it7236_upgrade(InputBuffer, fileSize)) != 0) && (retry > 0))
		retry--;

	//ret = it7236_upgrade(InputBuffer, fileSize);

	return count;
}
#endif
#if IT7280_FW_AUTO_UPGRADE
static int get_firmware_ver(void)
{
	#if 0
	char pucBuffer[1];
        u8  wTemp[4];
        u32  fw_version;
		int ret;

	waitCommandDone();
	pucBuffer[0] = 0x00;
	ret = i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	pucBuffer[0] = 0x80;
	ret = i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);
	pucBuffer[0] = 0x01;
	ret = i2cWriteToIt7280(gl_ts->client, 0x40, pucBuffer, 1);
	pucBuffer[0] = 0x01;
	ret = i2cWriteToIt7280(gl_ts->client, 0x41, pucBuffer, 1);
	pucBuffer[0] = 0x40;
	ret = i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);
	pucBuffer[0] = 0x00;
	ret = i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	pucBuffer[0] = 0x80;
	ret = i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);
	
	ret = i2cReadFromIt7280(gl_ts->client, 0x48, wTemp, 4);	//FIRMWARE_VERSION �?个字节，目前读出来应该为FF 00 04 00
	if(ret < 0)
	{
		return ret;
	}
	fw_version = (wTemp[0] << 24) | (wTemp[1] << 16) | (wTemp[2] << 8) | (wTemp[3]);
	//memcpy(fw_version, wTemp, 4);
	printk("IT7236 version:0X%02x.%02x.%02x.%02x\n", wTemp[0], wTemp[1], wTemp[2], wTemp[3]);

	return fw_version;
	#else
	char pucBuffer[1];
	int ret;
	u32  fw_version;
	u8  wTemp[10];
	waitCommandDone();
	//msleep(10);
	// 1. Request Full Authority of All Registers
	pucBuffer[0] = 0x01;
	if(i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1) < 0)
	{
		pr_err("get_firmware_ver failed \n ");
		return -1;
	}

	// 2. Assert Reset of MCU
	pucBuffer[0] = 0x64;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	pucBuffer[0] = 0x04;
	i2cWriteToIt7280(gl_ts->client, 0x01, pucBuffer, 1);

	// 3. Assert EF enable & reset
	pucBuffer[0] = 0x10;
	i2cWriteToIt7280(gl_ts->client, 0x2B, pucBuffer, 1);
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	//printk("get_config_ver 2222 \n ");
	// 4. Test Mode Enable
	pucBuffer[0] = 0x07;
	gl_ts->client->addr = 0x7F;
	i2cWriteToIt7280(gl_ts->client, 0xF4, pucBuffer, 1);
	gl_ts->client->addr = 0x46;
	//printk("get_config_ver 333 \n ");
	// 5. Page switch to 4
	pucBuffer[0] = 0x04;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	//printk("get_config_ver 444 \n ");
	// 6. Write EF Read Mode Cmd
	pucBuffer[0] = 0x01;
	i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);

	gl_ts->client->addr = 0x7F;
	ret = i2cReadFromIt7280(gl_ts->client, 0x00, wTemp, 10);
	//printk("get_config_ver 555 \n ");
	gl_ts->client->addr = 0x46;

	// 7. Write EF Standby Mode Cmd
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);
	//printk("get_config_ver 666 \n ");
	// 8. Power On Reset
	fnFirmwareReinitialize();
	//printk("get_config_ver end \n ");
	//printk("get_config_ver 777 \n ");
	fw_version = (wTemp[6] << 24) | (wTemp[7] << 16) | (wTemp[8] << 8) | (wTemp[9]);
	printk("IT7236 version:0X%02x.%02x.%02x.%02x\n", wTemp[6], wTemp[7], wTemp[8], wTemp[9]);

	return fw_version;
	#endif
}

static ssize_t IT7280_upgrade_auto(void)
{
	    int ret, retry = 5;
        int read_version ;

        read_version = get_firmware_ver();
		if(read_version == -1)
		{
			printk("IT7280_upgrade_auto i2c transfer error read_version =%x\n",read_version);
			return read_version;
		}
		ret = sizeof(rawData);
		printk("IT7280_upgrade_auto ret=%d \n",ret);
		printk("IT7280_upgrade_auto read_version = %x,raw_version=%x\n",read_version,raw_version);
		#if 1
        if(read_version != raw_version ){
			printk("IT7280_upgrade_auto start upgrade \n");
	   	 while(((it7236_upgrade((u8 *)rawData,sizeof(rawData))) != 0) && (retry > 0))
           // while(((it7236_upgrade(rawData,1024)) != 0) && (retry > 0))
		retry--;
	   //ret = it7236_upgrade(InputBuffer, fileSize);
        }
		#endif
	   return 0;//count;
}
#endif

static ssize_t IT7280_appfwupgrade_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s not ready\n", __func__);
}

static ssize_t IT7280_appfwupgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	pr_err("[IT7236]: %s not ready\n", __func__);
	return count;
}

static ssize_t IT7280_selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s not ready\n", __func__);
}

static ssize_t IT7280_selftest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	pr_err("[IT7236]: %s not ready\n", __func__);
	return count;
}

static ssize_t IT7280_tp_goldsample_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	pr_err("[IT7236]: %s not ready\n", __func__);
	return count;
}

static ssize_t IT7280_gpiocontrol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char pucBuffer[1];

	i2cReadFromIt7280(gl_ts->client, 0xFD, pucBuffer, 1);

	return sprintf(buf, "%d\n", pucBuffer[0]);
}

static ssize_t IT7280_gpiocontrol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int input;
	char pucBuffer[1];

	sscanf(buf, "%u", &input);

	pucBuffer[0] = input;
	i2cWriteToIt7280(gl_ts->client, 0xFD, pucBuffer, 1);

	return count;
}

static ssize_t IT7280_ledcontrol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char pucBuffer[1];

	i2cReadFromIt7280(gl_ts->client, 0xFC, pucBuffer, 1);

	return sprintf(buf, "%d\n", pucBuffer[0]);
}

static ssize_t IT7280_ledcontrol_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int input;
	char pucBuffer[1];

	sscanf(buf, "%u", &input);

	pucBuffer[0] = input;
	i2cWriteToIt7280(gl_ts->client, 0xFC, pucBuffer, 1);

	return count;
}

static ssize_t IT7280_touchcontrol_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char pucBuffer[1];

	i2cReadFromIt7280(gl_ts->client, 0xFF, pucBuffer, 1);

	return sprintf(buf, "%d\n", pucBuffer[0]);
}

static ssize_t firmware_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", it7236_fw_name);
}

static ssize_t firmware_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if(count > 256) {
		pr_err("%s : input string is too long, count = %ld\n", __func__, count);
		return -EINVAL;
	}

	sprintf(it7236_fw_name, "%s", buf);
	it7236_fw_name[count-1] = '\0';

	return count;
}

static DEVICE_ATTR(upgrade, 0644, IT7280_upgrade_show, IT7280_upgrade_store);
static DEVICE_ATTR(test, 0644, IT7280_selftest_show, IT7280_selftest_store);
static DEVICE_ATTR(goldsample, 0644, NULL, IT7280_tp_goldsample_store);
static DEVICE_ATTR(appfwupgrade, 0644, IT7280_appfwupgrade_show, IT7280_appfwupgrade_store);
static DEVICE_ATTR(gpiocontrol, 0644, IT7280_gpiocontrol_show, IT7280_gpiocontrol_store);
static DEVICE_ATTR(ledcontrol, 0644, IT7280_ledcontrol_show, IT7280_ledcontrol_store);
static DEVICE_ATTR(touchcontrol, 0644, IT7280_touchcontrol_show, NULL);
static DEVICE_ATTR(firmware_name, 0644, firmware_name_show, firmware_name_store);

static struct attribute *it7280_attributes[] = {
	&dev_attr_upgrade.attr,
	&dev_attr_test.attr,
	&dev_attr_goldsample.attr,
	&dev_attr_appfwupgrade.attr,
	&dev_attr_gpiocontrol.attr,
	&dev_attr_ledcontrol.attr,
	&dev_attr_touchcontrol.attr,
	&dev_attr_firmware_name.attr,
	NULL
};

static const struct attribute_group it7280_attr_group = {
	.attrs = it7280_attributes,
};
/**************************apk update start********************************/
struct ioctl_cmd168_apk {
	unsigned short slaveAddr;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

//====================================================================================
static int i2cAdvancedReadFromIt7236(struct i2c_client *client, unsigned char bufferIndex, unsigned char dataBuffer[], unsigned short dataLength)
{
	int ret;
	struct i2c_msg msgs[2] = { {
		.addr = client->addr,
		.flags = 0,//I2C_M_NOSTART,
		.len = 1,
		.buf = &bufferIndex
		}, {
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = dataLength,
		.buf = dataBuffer
		}
	};

	memset(dataBuffer, 0xFF, dataLength);
	  	ret = i2c_transfer(client->adapter, msgs, 2);

	return ret;
}

static int i2cAdvancedWriteToIt7236(struct i2c_client *client, unsigned char bufferIndex, unsigned char const dataBuffer[], unsigned short dataLength)
{
	unsigned char buffer4Write[256];
	int ret, retry = 3;
	struct i2c_msg msgs[1] = { {
		.addr = client->addr,
		.flags = 0,
		.len = dataLength + 1,
		.buf = buffer4Write
		}
	};

	if(dataLength < 256) {
		buffer4Write[0] = bufferIndex;
		memcpy(&(buffer4Write[1]), dataBuffer, dataLength);

		do {
			ret = i2c_transfer(client->adapter, msgs, 1);
			retry--;
		} while((ret != 1) && (retry > 0));
		if(ret != 1)
			pr_err("%s : i2c_transfer error\n", __func__);
		return ret;

		//return i2c_transfer(client->adapter, msgs, 1);
	}
	else
		return -1;
}

static long ite7280_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
//	struct ite7260_data *dev = filp->private_data;
	int retval = 0;
	int i;
	//unsigned char ucQuery;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168_apk data;
	struct i2c_client * i2c_client;
	//unsigned char datalen;
	//unsigned char ent[] = {0x60, 0x00, 0x49, 0x54, 0x37, 0x32};
//	unsigned char ext[] = {0x60, 0x80, 0x49, 0x54, 0x37, 0x32};

	//pr_info("=ite7236_ioctl=\n");
	memset(&data, 0, sizeof(struct ioctl_cmd168_apk));

	pr_info("=ite7280_ioctl cmd= %d\n",cmd);
	i2c_client = gl_ts->client;

	switch (cmd) {
	case IOCTL_SET:
		pr_info("=IOCTL_SET=\n");
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			return (retval);
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168_apk)) ) {
			retval = -EFAULT;
			return (retval);
		}
		
		for (i = 0; i < data.length; i++) {
			buffer[i] = (unsigned char) data.buffer[i];
		}
        
		i2c_client->addr = data.slaveAddr;
		//pr_info("=================================================\n");
		//pr_info("name[%s]---addr[%x]-flags[%d]=\n",gl_ts->client->name,gl_ts->client->addr,gl_ts->client->flags);
		//datalen = (unsigned char) (data.length + 1);
		//pr_info("datalen=%d\n", datalen);
		//write_lock(&dev->lock);
	retval = i2cAdvancedWriteToIt7236( i2c_client,
				(unsigned char) data.bufferIndex,
				buffer,
				(unsigned char)data.length );

		//write_unlock(&dev->lock);

		break;

	case IOCTL_GET:		
		pr_info("=IOCTL_GET=\n");
		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			return (retval);
		}

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			return (retval);
		}

		pr_info("sizeof(struct ioctl_cmd168)=%ld\n", sizeof(struct ioctl_cmd168_apk));
		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168_apk)) ) {
			retval = -EFAULT;
			return (retval);
		}
		
		i2c_client->addr = data.slaveAddr;
		retval = i2cAdvancedReadFromIt7236(i2c_client, 
				(unsigned char) data.bufferIndex,
				(unsigned char*) buffer,
				(unsigned char) data.length);
		//read_unlock(&dev->lock);
		//pr_info("GET:retval=%x\n", retval);
	
		for (i = 0; i < data.length; i++) {
			data.buffer[i] = (unsigned short) buffer[i];
		}
		//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, buffer[0], buffer[1], buffer[2], buffer[3]);
		//pr_info("GET:bufferIndex=%x, dataLength=%d, buffer[0]=%x, buffer[1]=%x, buffer[2]=%x, buffer[3]=%x\n", data.bufferIndex, data.length, data.buffer[0], data.buffer[1], data.buffer[2], data.buffer[3]);
		//if (data.bufferIndex == 0x80)
		//	data.buffer[0] = 0x00;
		if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168_apk)) ) {
			retval = -EFAULT;
			return (retval);
		}
		break;
		
	default:
		pr_info("default\n");
		retval = -ENOTTY;
		break;
	}

	pr_info("DONE! retval=%d\n", retval);
	return (retval);
}

static int ite7280_open(struct inode *inode, struct file *filp)
{
	int i;
	struct ioctl_cmd168_apk *dev;

	pr_info("=ite7236_open=\n");
	dev = kzalloc(sizeof(struct ioctl_cmd168_apk), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	/* initialize members */
	//rwlock_init(&dev->lock);
	for (i = 0; i < MAX_BUFFER_SIZE; i++) {
		dev->buffer[i] = 0xFF;
	}

	filp->private_data = dev;

	return 0; /* success */
}

static int ite7280_close(struct inode *inode, struct file *filp)
{
	struct ioctl_cmd168_apk *dev = filp->private_data;
	pr_info("=ite7280_close=\n");
	if (dev) {
		kfree(dev);
	}

	return 0; /* success */
}


struct file_operations ite7236_fops = {
	.owner = THIS_MODULE,
	.open = ite7280_open,
	.release = ite7280_close,
	.unlocked_ioctl = ite7280_ioctl,
};

static int get_config_ver(void)
{
	char pucBuffer[1];
	int ret;

	pr_err("get_config_ver start \n ");
	waitCommandDone();
	//msleep(10);
	// 1. Request Full Authority of All Registers
	pucBuffer[0] = 0x01;
	if(i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1) < 0)
	{
		pr_err("get_config_ver failed \n ");
		return -1;
	}

	// 2. Assert Reset of MCU
	pucBuffer[0] = 0x64;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	pucBuffer[0] = 0x04;
	i2cWriteToIt7280(gl_ts->client, 0x01, pucBuffer, 1);

	// 3. Assert EF enable & reset
	pucBuffer[0] = 0x10;
	i2cWriteToIt7280(gl_ts->client, 0x2B, pucBuffer, 1);
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	//printk("get_config_ver 2222 \n ");
	// 4. Test Mode Enable
	pucBuffer[0] = 0x07;
	gl_ts->client->addr = 0x7F;
	i2cWriteToIt7280(gl_ts->client, 0xF4, pucBuffer, 1);
	gl_ts->client->addr = 0x46;
	//printk("get_config_ver 333 \n ");
	// 5. Page switch to 4
	pucBuffer[0] = 0x04;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	//printk("get_config_ver 444 \n ");
	// 6. Write EF Read Mode Cmd
	pucBuffer[0] = 0x01;
	i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);

	gl_ts->client->addr = 0x7F;
	ret = i2cReadFromIt7280(gl_ts->client, 0x00, wTemp, 10);
	//printk("get_config_ver 555 \n ");
	gl_ts->client->addr = 0x46;

	// 7. Write EF Standby Mode Cmd
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF7, pucBuffer, 1);
	//printk("get_config_ver 666 \n ");
	// 8. Power On Reset
	fnFirmwareReinitialize();
	//printk("get_config_ver end \n ");
	//printk("get_config_ver 777 \n ");
	memcpy(config_id, wTemp+6, 4);
	pr_info("IT7236 version:%d.%d.%d.%d\n", config_id[0], config_id[1], config_id[2], config_id[3]);

	return ret;
}
#if 1

static void report_capkey_status(char key_code)
{
    char key_status;

    if(key_code == 0) {             
        key_status = 0;
    } else {
        key_status = 1;
    }
	//printk("report_capkey_status key_code =%d\n",key_code);
  
    if(1 == key_status){ //key_down
    	switch(key_code)
    	{
    		case 2 ://back
    		key_down = KEY_BACK;
    		break;
			case 4://menu
			key_down = KEY_HOMEPAGE;
			break;
			case 1://back
			key_down = KEY_MENU;
			break;
			default:
				break;
    	}
	   input_report_key(input_dev, key_down, 1);
	   
    }
    if(0 == key_status){ //key_up
    input_report_key(input_dev, key_down, 0);
   }
	input_sync(input_dev);
}
static void Read_Point(struct IT7236_ts_data *ts)
{
	u8 key_low;
    u8 key_high;
	unsigned char pucBuffer[1];
	pucBuffer[0] = 0x00;
	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);
	i2cReadFromIt7280(gl_ts->client, 0xFC, pucBuffer, 1);
	//printk("it7236 read data from 0xfc is 0x%x\n", pucBuffer[0]);
     key_low = pucBuffer[0] & 0x0f;
      key_high = (~((pucBuffer[0] & 0xf0) >> 4)) & 0x0f;
   // printk("it7236 key_low is 0x%x,key_high is 0x%x\n", key_low,key_high);
	if(key_low == key_high)
	{
		pucBuffer[0] = 0x00;
		i2cWriteToIt7280(gl_ts->client, 0xF3, pucBuffer, 1);
		report_capkey_status(key_low);
	}

	//input_report_key(input_dev, KEY_MENU, 1);
	//input_report_key(input_dev, KEY_MENU, 0);

	//input_sync(input_dev);
}
#endif
static irqreturn_t IT7280_ts_work_func(int irq, void *dev_id)
{
	disable_irq_nosync(gl_ts->client->irq);
	//printk("IT7280_ts_work_func enter \n");
	Read_Point(gl_ts);
	enable_irq(gl_ts->client->irq);
	return IRQ_HANDLED;
}


static int it7280_parse_dt(struct device *dev,
			struct IT7236_ts_data *pdata)
{
	int rc ,gpio;
	struct device_node *np = dev->of_node;

	/* get panel name info */
	pdata->name = "ite";
	rc = of_property_read_string(np, "ite,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	/* vdd, reset, irq gpio info */
	pdata->reg_enable = of_property_read_bool(np, "ite,regulator-enable");
	printk("%s, pdata->reg_enable=%d\n", __func__, pdata->reg_enable);
	if (!(pdata->reg_enable))
	{
		pdata->enable_gpio = of_get_named_gpio_flags(np, "vdd_enable",
					0, &pdata->enable_gpio_flags);
		if (pdata->enable_gpio < 0)
			return pdata->enable_gpio;
	}

	pdata->irq_gpio = of_get_named_gpio_flags(np, "ite,irq-gpio",
				0, &pdata->irq_gpio_flags);
	printk("%s, irq_gpio=%d\n", __func__, pdata->irq_gpio);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	gpio = of_get_named_gpio(np, "ite,reset-gpio", 0);
	printk("%s, reset-gpio=%d\n", __func__, gpio);
	pdata->reset_gpio = gpio;
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	#if 0
	pdata->gold_sample = of_get_property(np, "ite,golden-sample", &blen);
	if (!pdata->gold_sample) {
		pr_err("[IT7236]: Can not read the golden sample array\n");
	}
	pdata->gold_sample_size = blen;
	#endif
	return 0;
}

static int it7280_power_init(struct IT7236_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	if(data->reg_enable) {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if(regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd, ITE_VTG_MIN_UV, ITE_VTG_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}
	else {
		if (gpio_is_valid(data->enable_gpio)) {
			rc = gpio_request(data->enable_gpio, "it7280_enable_gpio");
			if (rc) {
				pr_err("enable gpio request failed\n");
				goto reg_vdd_set_vtg;
			}
		}
	}

	return 0;

reg_vdd_set_vtg:
	if ((regulator_count_voltages(data->vdd) > 0))
		regulator_set_voltage(data->vdd, 0, ITE_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
pwr_deinit:

	return 0;
}

static int it7280_power_on(struct IT7236_ts_data *data, bool on)
{
	int rc = 0;

	if (!on)
		goto power_off;

	if(data->reg_enable) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
		printk("it7280_power_on enable vreg ok \n");
	}
	else {
		if (gpio_is_valid(data->enable_gpio))
			gpio_set_value(data->enable_gpio, 1);
	}

	return rc;

power_off:
	if(data->reg_enable) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	else {
		if (gpio_is_valid(data->enable_gpio))
			gpio_set_value(data->enable_gpio, 0);
	}

	return rc;
}

static int ts_key_pinctrl_init(struct i2c_client *client)
{
	int retval;
	struct IT7236_ts_data *pdata = i2c_get_clientdata(client);
	
	/* Get pinctrl if target uses pinctrl */
	pdata->ts_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(pdata->ts_pinctrl))
	{
		dev_dbg(&client->dev, "Target does not use pinctrl\n");
		retval = PTR_ERR(pdata->ts_pinctrl);
		pdata->ts_pinctrl = NULL;
		return retval;
	}
    
	pdata->gpio_state_active	= pinctrl_lookup_state(pdata->ts_pinctrl, "ts_key_active");
	if (IS_ERR_OR_NULL(pdata->gpio_state_active))
	{
		printk("%s Can not get ts default pinstate\n", __func__);
		retval = PTR_ERR(pdata->gpio_state_active);
		pdata->ts_pinctrl = NULL;
		return retval;
	}

	pdata->gpio_state_suspend = pinctrl_lookup_state(pdata->ts_pinctrl, "ts_key_suspend");
	if (IS_ERR_OR_NULL(pdata->gpio_state_suspend))
	{
		dev_err(&client->dev,	"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(pdata->gpio_state_suspend);
		pdata->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int ts_key_pinctrl_select(struct i2c_client *client, bool on)
{
	struct IT7236_ts_data *pdata = i2c_get_clientdata(client);
	struct pinctrl_state *pins_state;
	int ret;
	
	pins_state = on ? pdata->gpio_state_active : pdata->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state))
	{
		ret = pinctrl_select_state(pdata->ts_pinctrl, pins_state);
		if (ret)
		{
			dev_err(&client->dev, "can not set %s pins\n",
				on ? "ts_key_active" : "ts_key_suspend");
			return ret;
		}
	}
	else
	{
		dev_err(&client->dev,	"not a valid '%s' pinstate\n",
			on ? "ts_key_active" : "ts_key_suspend");
	}

	return 0;
}



static int IT7280_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct IT7236_ts_data *ts;
	int ret = 0;
	int err;
	dev_t dev = MKDEV(ite7280_major, 0);
	struct device *class_dev = NULL;

	pr_info("[IT7236]: IT7236 enter probe\n");

	if (client->dev.of_node) {
		ts = devm_kzalloc(&client->dev,
			sizeof(struct IT7236_ts_data), GFP_KERNEL);
		if (!ts) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = it7280_parse_dt(&client->dev, ts);
		if (err) {
			dev_err(&client->dev, "Data parsing failed\n");
			goto err_parse;
		}
	}
	else
		ts = client->dev.platform_data;

	err = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (err) {
		pr_err("[IT7236]: IT7280 cdev can't get major number\n");
		goto err_alloc_dev;
	}
	ite7280_major = MAJOR(dev);

	/*allocate the character device*/
	cdev_init(&ite7280_cdev, &ite7236_fops);
	ite7280_cdev.owner = THIS_MODULE;
	ite7280_cdev.ops = &ite7236_fops;
	err = cdev_add(&ite7280_cdev, MKDEV(ite7280_major, ite7280_minor), 1);
	if(err) {
		goto err_add_dev;
	}

	/*register class*/
	ite7280_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ite7280_class)) {
		pr_err("[IT7236]: failed in creating class.\n");
		goto err_create_class;
	}

	ite7280_dev = MKDEV(ite7280_major, ite7280_minor);
	class_dev = device_create(ite7280_class, NULL, ite7280_dev, NULL, DEVICE_NAME);
	if(class_dev == NULL) {
		pr_err("[IT7236]: failed IT7280 in creating device.\n");
		goto err_create_dev;
	}

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("[IT7236]: failed to allocate input device\n");
		goto err_alloc_input_dev;
	}

	input_dev->name = IT7280_I2C_NAME;
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);
	input_set_capability(input_dev, EV_KEY,KEY_MENU);
	input_set_capability(input_dev, EV_KEY,KEY_HOMEPAGE);
	input_set_capability(input_dev, EV_KEY,KEY_BACK);
							 

	//set_bit(TOUCHKEY_LEFT, input_dev->keybit);
	//input_set_capability(input_dev, EV_KEY, TOUCHKEY_LEFT);
	//set_bit(TOUCHKEY_RIGHT, input_dev->keybit);
	//input_set_capability(input_dev, EV_KEY, TOUCHKEY_RIGHT);

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	err = input_register_device(input_dev);
	if(err) {
		pr_err("[IT7236]: device register error\n");
		goto dev_reg_err;
	}

	mutex_init(&ts->device_mode_mutex);
	client->irq = gpio_to_irq(ts->irq_gpio);
	ts->client = client;

	i2c_set_clientdata(client, ts);
	

	ret = ts_key_pinctrl_init(client);
	if (!ret && ts->ts_pinctrl)
	{
		ret = ts_key_pinctrl_select(client, true);
		if (ret < 0)
			goto Err_pinctrl_init;
	}

	/* power init */
	err = it7280_power_init(ts, true);
	if (err) {
		dev_err(&client->dev, "power init failed, %d", __LINE__);
		goto err_pwr_init;
	}

	/* power on sequence */
	err = it7280_power_on(ts, true);
	if (err) {
		dev_err(&client->dev, "power on failed, %d", __LINE__);
		goto err_pwr_on;
	}

	if (gpio_is_valid(ts->irq_gpio)) {
		err = gpio_request(ts->irq_gpio, "ITE_irq_gpio");
		if (err) {
			pr_err("[IT7236]: can not request irq gpio\n");
			goto free_irq_gpio;
		}

		err = gpio_direction_input(ts->irq_gpio);
		if (err) {
			pr_err("[IT7236]: irq_gpio direction_input set failed\n");
			goto err_irq_gpio_direction;
		}
	}
	else
		pr_err("[IT7236]: irq gpio error\n");

	if (gpio_is_valid(ts->reset_gpio)) {
		if (!gpio_request(ts->reset_gpio, "ITE_reset_gpio")) {
			gpio_direction_output(ts->reset_gpio, 1);
		}
		else
		{
			pr_err("[IT7236]: can not request reset gpio\n");
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("[IT7236] : IT7280_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts->ts_workqueue = create_singlethread_workqueue(IT7280_I2C_NAME);
	if (!ts->ts_workqueue)
		goto err_create_queue;

	ts->input_dev = input_dev;
	gl_ts = ts;
	ts->irqflags = IRQF_TRIGGER_LOW;
	if (ts->irq_gpio) {
		pr_info("[IT7236]: irq = %d , gpio = %d\n",gpio_to_irq(ts->irq_gpio), ts->irq_gpio);
		ret = request_threaded_irq(client->irq, NULL,
				   IT7280_ts_work_func, ts->irqflags|IRQF_ONESHOT,
				   IT7280_I2C_NAME, ts);
		if (ret == 0)
		{
			ts->use_irq = 1;
			printk("[IT7236] request irq success \n");
		}
		else {
			dev_err(&client->dev, "[IT7236] : request_irq failed\n");
			goto err_request_irq;
		}
	}

	ret = sysfs_create_group(&input_dev->dev.kobj, &it7280_attr_group);
	if (ret) {
		pr_err("[IT7236] : sysfs_create_group: Error to create calibration attribute\n");
		goto err_sysfs_create_group;
	}

	ret = sysfs_create_link(input_dev->dev.kobj.parent, &input_dev->dev.kobj, "touchkey");
	if(ret)
	{
		pr_err("[IT7236]: sysfs_create_link failed create [touchkey] link\n");
		goto err_sysfs_create_link;
	}

	/*to ensure the TP is connected to i2c*/
	ret = get_config_ver();
	//if(ret)
	//	goto err_get_ver;
	#if 0
	ret = fih_init();
	if(ret)
		goto err_create_allhwlist;
	#endif
	#if IT7280_FW_AUTO_UPGRADE
	IT7280_upgrade_auto();
	#endif

	#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret) {
		dev_err(&client->dev,
			"Unable to register fb_notifier: %d\n",
			ret);
	}
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ts_early_suspend;
	ts->early_suspend.resume = ts_late_resume;
	register_early_suspend(&ts->early_suspend);
	#endif
	pr_info("[IT7236]: gl_ts->client->irq = %d \n",gl_ts->client->irq);
	//enable_irq(gl_ts->client->irq);
	pr_info("[IT7236]: IT7236 probe end\n");
	return 0;

//err_create_allhwlist:
//err_get_ver:
//	sysfs_remove_link(input_dev->dev.kobj.parent, "touchkey");
err_sysfs_create_link:
	sysfs_remove_group(&input_dev->dev.kobj, &it7280_attr_group);
err_sysfs_create_group:
	free_irq(client->irq, ts);
err_request_irq:
	destroy_workqueue(ts->ts_workqueue);
err_create_queue:
err_check_functionality_failed:
	it7280_power_on(ts, false);
err_pwr_on:
	it7280_power_init(ts, false);
Err_pinctrl_init:
	if (ts->ts_pinctrl)
		pinctrl_put(ts->ts_pinctrl);
err_pwr_init:
err_irq_gpio_direction:
	gpio_free(ts->irq_gpio);
free_irq_gpio:
	input_unregister_device(input_dev);
dev_reg_err:
	input_free_device(input_dev);
err_alloc_input_dev:
	device_destroy(ite7280_class, ite7280_dev);
err_create_dev:
	class_destroy(ite7280_class);
err_create_class:
	cdev_del(&ite7280_cdev);
err_add_dev:
err_alloc_dev:
err_parse:
	devm_kfree(&client->dev, ts);
	return ret;

}

static int IT7280_ts_remove(struct i2c_client *client)
{
	sysfs_remove_group(&input_dev->dev.kobj, &it7280_attr_group);
	destroy_workqueue(gl_ts->ts_workqueue);
	#if defined(CONFIG_FB)
	fb_unregister_client(&gl_ts->fb_notif);
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&gl_ts->early_suspend);
	#endif
	it7280_power_on(gl_ts, false);
	it7280_power_init(gl_ts, false);
	input_unregister_device(input_dev);
	input_free_device(input_dev);
	device_destroy(ite7280_class, ite7280_dev);
	class_destroy(ite7280_class);
	cdev_del(&ite7280_cdev);
	devm_kfree(&client->dev, gl_ts);

	return 0;
}

static int IT7280_ts_suspend(struct device *dev)
{
		char pucBuffer[3],data[2];
	int count=0;
	printk("IT7280_ts_suspend enter \n");
      
	// disable_IRQ
    disable_irq_nosync(gl_ts->client->irq);
	
	  do{
	  	i2cReadFromIt7280(gl_ts->client, 0xFA, data, 2);
	  	pucBuffer[0] = 0x00;
	  	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);//set Pate to Page0
	  	
	  	  pucBuffer[0] = 0x80;
	      i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);//set 0XF1 bit7 t0 1
	      
	      
		  i2cReadFromIt7280(gl_ts->client, 0xFA, data, 2);
		count++;
	} while( ((data[0]&0X01) != 0x01) && (count < 5));//0xFA  bit0  =1?

	

        pucBuffer[0] = 0x01;//C/R
     	pucBuffer[1] = 0x30;//CMD=0x30,Set power mode
    	pucBuffer[2] = 0x02;//sub cmd =0x02,sleep mode
	    i2cWriteToIt7280(gl_ts->client, 0x40, pucBuffer, 3);//write commond buffer at 0X40
       
      
		pucBuffer[0] = 0x40;
	  	i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);//set Pate to Page0
	  	
	  	i2cReadFromIt7280(gl_ts->client, 0xF3, data, 2);
	  	
	  	 pucBuffer[0] = 0x00;
	  	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);//set Pate to Page0
	  
	  	
	    pucBuffer[0] = 0x80;
	    i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);//set 0XF1 bit7 t0 1
	   
	  	
	    i2cReadFromIt7280(gl_ts->client, 0xFA, data, 2);
		
		

		
	return 0;
}


static int IT7280_ts_resume(struct device *dev)
{
     char pucBuffer[2],data[2];
	 int count=0;

	 printk("IT7280_ts_resume enter \n");
	 
      pucBuffer[0] = 0x55; 
      i2cWriteToIt7280(gl_ts->client, 0xFB, pucBuffer, 1); 


 		   do{
	  	i2cReadFromIt7280(gl_ts->client, 0xFA, data, 2);
	  	
	  	pucBuffer[0] = 0x00;
	  	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);//set Pate to Page0
	  	
	  	  pucBuffer[0] = 0x80;
	      i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);//set 0XF1 bit7 t0 1
	      
	      
		  i2cReadFromIt7280(gl_ts->client, 0xFA, data, 2);
		count++;
	} while( ((data[0]&0X01) != 0x01) && (count < 5));//0xFA  bit0  =1?

    pucBuffer[0] = 0x01;//C/R
 	pucBuffer[1] = 0xF0;//CMD=0x30,Set power mode
    i2cWriteToIt7280(gl_ts->client, 0x40, pucBuffer,2);//write commond buffer at 0X40
   
  
	  pucBuffer[0] = 0x40;
  	i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);//set Pate to Page0
  	
  	
  	 pucBuffer[0] = 0x00;
  	i2cWriteToIt7280(gl_ts->client, 0xF0, pucBuffer, 1);//set Pate to Page0
  	
    pucBuffer[0] = 0x80;
    i2cWriteToIt7280(gl_ts->client, 0xF1, pucBuffer, 1);//set 0XF1 bit7 t0 1
  	
    i2cReadFromIt7280(gl_ts->client, 0xFA, data, 2);
      
        enable_irq(gl_ts->client->irq);

		
	return 0;
}


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct IT7236_ts_data *ts =
		container_of(self, struct IT7236_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
				ts && ts->client) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				if (IT7280_ts_resume(&ts->client->dev) != 0)
					dev_err(&ts->client->dev, "%s: failed\n", __func__);
			}else if (*blank == FB_BLANK_POWERDOWN) {
				if (IT7280_ts_suspend(&ts->client->dev) != 0)
					dev_err(&ts->client->dev, "%s: failed\n", __func__);
			}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ts_early_suspend(struct early_suspend *es)
{
	struct IT7236_ts_data *ts;
	ts = container_of(es, struct IT7236_ts_data, early_suspend);

	if (IT7280_ts_suspend(&ts->client->dev) != 0)
		dev_err(&ts->client->dev, "%s: failed\n", __func__);
}

static void ts_late_resume(struct early_suspend *es)
{
	struct IT7236_ts_data *ts;
	ts = container_of(es, struct IT7236_ts_data, early_suspend);

	if (IT7280_ts_resume(&ts->client->dev) != 0)
		dev_err(&ts->client->dev, "%s: failed\n", __func__);
}
#else
static SIMPLE_DEV_PM_OPS(ts_pm_ops, ts_suspend, ts_resume);
#endif





static const struct i2c_device_id IT7280_ts_id[] = {
	{ IT7280_I2C_NAME, 0 },
	{ }
};

static struct of_device_id it7236_match_table[] = {
	{ .compatible = "ite,7236",},
	{ },
};

#ifdef CONFIG_PM
static const struct dev_pm_ops it7236_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
		.suspend = IT7280_ts_suspend,
		.resume = IT7280_ts_resume,
#endif
};
#endif

static struct i2c_driver IT7280_ts_driver = {
		.class = I2C_CLASS_HWMON,
		.probe = IT7280_ts_probe,
		.remove = IT7280_ts_remove,
		.id_table = IT7280_ts_id,
		.driver = {
			.name = IT7280_I2C_NAME,
			.of_match_table = it7236_match_table,
#ifdef CONFIG_PM
			.pm = &it7236_pm_ops,
#endif
		},
};

static int __init IT7280_ts_init(void)
{
	printk("IT7280_ts_init enter \n");
	return i2c_add_driver(&IT7280_ts_driver);
}

static void __exit IT7280_ts_exit(void)
{
	i2c_del_driver(&IT7280_ts_driver);
}

module_init( IT7280_ts_init);
module_exit( IT7280_ts_exit);

MODULE_DESCRIPTION("IT7236 TouchKey Driver");
MODULE_LICENSE("GPL");
