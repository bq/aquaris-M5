/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#define pr_fmt(fmt) "PN547:%s: " fmt, __func__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "pn547.h"
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>


#define MAX_BUFFER_SIZE	512

#define PN547_DEBUG

struct pn547_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn547_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int 		pwr_en;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	unsigned int		irq_gpio;
	unsigned int 	irq_wakeup_state;
	/* CLK_REQ IRQ to signal the state has changed */
	unsigned int		irq_gpio_clk_req;	
	unsigned int		clkreq_gpio;
	/* CLK control */
	unsigned int		clk_src_gpio;
	const	char		*clk_src_name;
	struct	clk		*s_clk;
	bool			clk_run;
};

//kingsun, zhudm add
static struct of_device_id msm_match_table[] = {
	{.compatible = "nxp,pn547"},
	{}
};

unsigned char pn547_debug = 0;

MODULE_DEVICE_TABLE(of, msm_match_table);

/*
	Routine to Select clocks
*/
static int pn547_clock_select(struct pn547_dev *pn547_dev)
{
	int r = 0;

	if (!strcmp(pn547_dev->clk_src_name, "BBCLK2")) {
		pr_err("nfc clk_src is BBCLK2\n");
		pn547_dev->s_clk  =
			clk_get(&pn547_dev->client->dev, "ref_clk");
		if (pn547_dev->s_clk == NULL)
			goto err_invalid_dis_gpio;
	} else {
		pn547_dev->s_clk = NULL;
		goto err_invalid_dis_gpio;
	}

	if (pn547_dev->clk_run == false) {
		/* Set clock rate */
		pr_err("pn547 clk_run = true\n");
		r = clk_prepare_enable(pn547_dev->s_clk);
		if (r)
			goto err_invalid_clk;
		pn547_dev->clk_run = true;
	}
	r = 0;
	return r;

err_invalid_clk:
	r = -1;
	return r;
err_invalid_dis_gpio:
	r = -2;
	return r;
}

/*
	Routine to De-Select clocks
*/

static int pn547_clock_deselect(struct pn547_dev *pn547_dev)
{
	int r = -1;
	if (pn547_dev->s_clk != NULL) {
		if (pn547_dev->clk_run == true) {
			clk_disable_unprepare(pn547_dev->s_clk);
			pn547_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}

#if defined(PN547_DEBUG)
/* sysfs interface */
static ssize_t nfc_show(struct device *dev,struct device_attribute *attr, char *buf)
{

	return sprintf(buf,"debug=%d\n",pn547_debug);
}

/* debug fs, "echo @1 > /sys/bus/i2c/devices/xxx/nfc_debug" @1:debug_flag  */
static ssize_t nfc_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
	const char *p = buf;
	pn547_debug = *p;
	if (pn547_debug == 49)
		pn547_debug = 1;
	pr_err("Macle pn547_debug = %d\n", pn547_debug);
	return count;
}

static struct device_attribute pn547_dev_attr =
	__ATTR(nfc_debug, S_IRUGO | S_IWUGO, nfc_show, nfc_store);
#endif

static int nfc_parse_dt(struct device *dev, struct pn547_dev *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;

	pdata->ven_gpio = of_get_named_gpio(np, "qcom,nxp-ven", 0);
	if ((!gpio_is_valid(pdata->ven_gpio)))
		return -EINVAL;

	pdata->irq_gpio = of_get_named_gpio(np, "qcom,nxp-irq", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	pdata->firm_gpio = of_get_named_gpio(np, "qcom,nxp-dw-req", 0);
	if ((!gpio_is_valid(pdata->firm_gpio)))
		return -EINVAL;

	r = of_property_read_string(np, "qcom,clk-src", &pdata->clk_src_name);

	pdata->clkreq_gpio = of_get_named_gpio(np, "qcom,nxp-clkreq", 0);

         pr_err("GPIO ven,irq,firm,clk_req = %d, %d, %d",
		 	pdata->ven_gpio, pdata->irq_gpio, pdata->clkreq_gpio);

	if (r)
		return -EINVAL;
	return r;
}

static void pn547_disable_irq(struct pn547_dev *pn547_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn547_dev->irq_enabled_lock, flags);
	if (pn547_dev->irq_enabled) {
		disable_irq_nosync(pn547_dev->client->irq);
		pn547_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn547_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *pn547_dev = dev_id;


	
	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}

	pn547_disable_irq(pn547_dev);

	/* Wake up waiting readers */
	wake_up(&pn547_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev *pn547_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(tmp, 0, MAX_BUFFER_SIZE);
	if (pn547_debug == 1)
	pr_err("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn547_dev->read_mutex);

	if (!gpio_get_value(pn547_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn547_dev->irq_enabled = true;
		enable_irq(pn547_dev->client->irq);
		ret = wait_event_interruptible(pn547_dev->read_wq,
				gpio_get_value(pn547_dev->irq_gpio));

		pn547_disable_irq(pn547_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn547_dev->client, tmp, count);
	mutex_unlock(&pn547_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}

	if (copy_to_user(buf, tmp, ret)) {
		pr_err("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	if (pn547_debug == 1){
		pr_err("IFD->PC:");
		for(i = 0; i < ret; i++){
			pr_err(" %02X", tmp[i]);
		}
		pr_err("\n");
	}

	return ret;

fail:
	mutex_unlock(&pn547_dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn547_dev  *pn547_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	pn547_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (pn547_debug == 1)
		pr_err("pn547 i2c addr client->addr = 0x%x\n", pn547_dev->client->addr);

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	if (pn547_debug == 1)
		pr_err("%s : writing %zu bytes.\n", __func__, count);

	/* Write data */
	ret = i2c_master_send(pn547_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}

	if (pn547_debug == 1){
		pr_err("PC->IFD:");
		for(i = 0; i < count; i++){
			pr_err(" %02X", tmp[i]);
		}
		pr_err("\n");
	}
exit:
	return ret;
}

static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	struct pn547_dev *pn547_dev = container_of(filp->private_data,
						struct pn547_dev,
						pn547_device);

	filp->private_data = pn547_dev;

	pr_err("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn547_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn547_dev *pn547_dev = filp->private_data;

	switch (cmd) {
	case PN547_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			pr_err("%s power on with firmware\n", __func__);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			gpio_set_value(pn547_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			pr_err("%s power on\n", __func__);
			if (pn547_dev->irq_wakeup_state == 0){
				pr_err("%s power on, wakeup_state=%d, need setup wakeup\n", __func__,pn547_dev->irq_wakeup_state);
				irq_set_irq_wake(pn547_dev->client->irq,1);//Using irq wakeup system
				pn547_dev->irq_wakeup_state = 1;
			}
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 1);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			pr_err("%s power off\n", __func__);
			if (pn547_dev->irq_wakeup_state == 1){	
				pr_err("%s power off, wakeup_state=%d, need setup not wakeup\n", __func__,pn547_dev->irq_wakeup_state);
				irq_set_irq_wake(pn547_dev->client->irq,0);//Does not using irq wakeup system
				pn547_dev->irq_wakeup_state = 0;
			}
			gpio_set_value(pn547_dev->firm_gpio, 0);
			gpio_set_value(pn547_dev->ven_gpio, 0);
			msleep(10);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.open	= pn547_dev_open,
	.unlocked_ioctl  = pn547_dev_ioctl,
};

static int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	int r = 0;
	int ret;
	struct pn547_dev *pn547_dev;

        pr_err("pn547 enter probe\n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	pr_err("nfc probe step02 is ok\n");

	pn547_dev = kzalloc(sizeof(*pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pr_err("nfc probe step04 is ok\n");

	pn547_dev->client   = client;
	ret=nfc_parse_dt(&client->dev,pn547_dev);
	pr_info("irq_gpio=%d, ven_gpio=%d, firm_gpio=%d \n",
		pn547_dev->irq_gpio, pn547_dev->ven_gpio, pn547_dev->firm_gpio);
	if (ret) {
		pr_err("parse node error. pls check the dts file \n");
		ret = -EINVAL;
		goto err_parser;
	}

 

	pn547_dev->clk_run = false;
	pn547_clock_select(pn547_dev);

	if (gpio_is_valid(pn547_dev->clkreq_gpio)) {
		r = gpio_request(pn547_dev->clkreq_gpio,
			"nfc_clkreq_gpio");
		if (r) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
					pn547_dev->clkreq_gpio);
			goto err_clkreq_gpio;
		}
		r = gpio_direction_input(pn547_dev->clkreq_gpio);
		if (r) {
			dev_err(&client->dev,
					"unable to set direction for gpio [%d]\n",
					pn547_dev->clkreq_gpio);
			goto err_clkreq_gpio;
		}
	} else {
		dev_err(&client->dev, "clkreq gpio not provided\n");
		goto err_clk;
	}

	/* init mutex and queues */
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);

	pn547_dev->pn547_device.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->pn547_device.name = PN547_NAME;
	pn547_dev->pn547_device.fops = &pn547_dev_fops;

	ret = misc_register(&pn547_dev->pn547_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_clkreq_gpio;
	}
	pr_err("nfc probe step05 is ok\n");

	if (gpio_request(pn547_dev->irq_gpio,"nxp,irq-gpio") != 0) {
		pr_err("PN547: gpio_IRQ_request error\n");
		goto err_irq;
	}

	if (gpio_request(pn547_dev->ven_gpio,"nxp,ven-gpio") != 0) {
		pr_err("PN547: gpio_VEN_request error\n");
		goto err_ven;
	}

    if(gpio_request(pn547_dev->firm_gpio,"nxp,firm-gpio") != 0) {
		printk("PN547: gpio_firm_request error\n");
		goto err_firm;
	}

	gpio_direction_output(pn547_dev->firm_gpio, 0);

	gpio_direction_output(pn547_dev->ven_gpio, 0);
	pr_err("nfc probe GPIO is ok\n");

	gpio_direction_input(pn547_dev->irq_gpio);
	
	pr_err("pn547 client->irq = %d", client->irq);
	client->irq = gpio_to_irq(pn547_dev->irq_gpio);
	pn547_dev->irq_wakeup_state = 0;
	pr_err("%s : requesting IRQ %d\n", __func__, client->irq);
	pn547_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn547_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn547_dev);
	if (ret) {
		pr_err("request_irq failed\n");
		goto err_request_irq_failed;
	}
	pr_err("nfc probe step06 is ok\n");

	pn547_disable_irq(pn547_dev);

	i2c_set_clientdata(client, pn547_dev);
	pr_err("nfc probe successful\n");


#if defined(PN547_DEBUG)
		ret = device_create_file(&client->dev, &pn547_dev_attr);
		if (ret) {
			//NFC_ERR("sysfs registration failed, error %d \n", ret);
			goto err_request_irq_failed;
		}
#endif

	return 0;

err_request_irq_failed:
	misc_deregister(&pn547_dev->pn547_device);
err_exit:
	gpio_free(pn547_dev->firm_gpio);
err_firm:
	gpio_free(pn547_dev->ven_gpio);
err_ven:
	gpio_free(pn547_dev->irq_gpio);
err_irq:
	misc_deregister(&pn547_dev->pn547_device);
err_clkreq_gpio:
	gpio_free(pn547_dev->clkreq_gpio);
err_clk:
err_parser:
	kfree(pn547_dev);	

	return ret;
}

static int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *pn547_dev;

	pn547_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn547_dev);
	misc_deregister(&pn547_dev->pn547_device);
	//kingsun, jerome/zhudm add	
#if defined(PN547_DEBUG)
	device_remove_file(&client->dev, &pn547_dev_attr);
#endif
	pn547_clock_deselect(pn547_dev);
	mutex_destroy(&pn547_dev->read_mutex);
	gpio_free(pn547_dev->irq_gpio);
	gpio_free(pn547_dev->ven_gpio);
	gpio_free(pn547_dev->firm_gpio);
	kfree(pn547_dev);

	return 0;
}

static const struct i2c_device_id pn547_id[] = {
	{ PN547_NAME, 0 },
	{ }
};

static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN547_NAME,
		.of_match_table = msm_match_table,
	},
};

static int __init pn547_dev_init(void)
{
	pr_err("Loading pn547 driver\n");
	return i2c_add_driver(&pn547_driver);
}
module_init(pn547_dev_init);

static void __exit pn547_dev_exit(void)
{
	pr_info("Unloading pn547 driver\n");
	i2c_del_driver(&pn547_driver);
}
module_exit(pn547_dev_exit);

MODULE_AUTHOR("Ernest Li");
MODULE_DESCRIPTION("NFC PN547 driver");
MODULE_LICENSE("GPL");
