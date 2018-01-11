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

#define PN544_DEBUG

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	unsigned int		irq_gpio;

	unsigned int		irq_gpio_clk_req;	
	unsigned int		clkreq_gpio;

	unsigned int		clk_src_gpio;
	const	char		*clk_src_name;
	struct	clk		*s_clk;
	bool			clk_run;
};

static struct of_device_id msm_match_table[] = {
	{.compatible = "nxp,nfc-pn544"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

static int pn544_clock_select(struct pn544_dev *pn544_dev)
{
	int r = 0;

	if (!strcmp(pn544_dev->clk_src_name, "BBCLK2")) {
		printk("nfc clk_src is BBCLK2\n");
		pn544_dev->s_clk  =
			clk_get(&pn544_dev->client->dev, "ref_clk");
		if (pn544_dev->s_clk == NULL)
			goto err_invalid_dis_gpio;
	} else {
		pn544_dev->s_clk = NULL;
		goto err_invalid_dis_gpio;
	}
	if (pn544_dev->clk_run == false) {

		r = clk_prepare_enable(pn544_dev->s_clk);
		if (r)
			goto err_invalid_clk;
		pn544_dev->clk_run = true;		
		printk("pn544 clk_run = true\n");
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

static int pn544_clock_deselect(struct pn544_dev *pn544_dev)
{
	int r = -1;
	return 0;
	if (pn544_dev->s_clk != NULL) {
		if (pn544_dev->clk_run == true) {
			clk_disable_unprepare(pn544_dev->s_clk);
			pn544_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}

struct pn544_dev *g_pn544_dev;
#if defined(PN544_DEBUG)
static ssize_t nfc_show(struct device *dev,struct device_attribute *attr, char *buf)
{

	return sprintf(buf,"%s\n","OK");
}

static ssize_t nfc_store(struct device *dev,
        struct device_attribute *attr, const char *buf,size_t count)
{
	g_pn544_dev->clk_run = false;
	pn544_clock_select(g_pn544_dev);
	return count;
}

static struct device_attribute pn544_dev_attr =
	__ATTR(nfc_debug, S_IRUGO | S_IWUGO, nfc_show, nfc_store);
#endif

static int nfc_parse_dt(struct device *dev, struct pn544_dev *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;
    
	pdata->ven_gpio = of_get_named_gpio(np, "nxp,ven-gpio", 0);
	if ((!gpio_is_valid(pdata->ven_gpio)))
		return -EINVAL;

	pdata->irq_gpio = of_get_named_gpio(np, "nxp,irq-gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	pdata->firm_gpio = of_get_named_gpio(np, "nxp,firm-gpio", 0);
	if ((!gpio_is_valid(pdata->firm_gpio)))
		return -EINVAL;

	r = of_property_read_string(np, "nxp,clk-src", &pdata->clk_src_name);

	pdata->clkreq_gpio = of_get_named_gpio(np, "nxp,clkreq-gpio", 0);

         printk("GPIO ven,irq,firm,clk_req = %d, %d, %d, %d",
		 	pdata->ven_gpio, pdata->irq_gpio, pdata->firm_gpio, pdata->clkreq_gpio);

	if (r)
		return -EINVAL;
	return r;
}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	printk("pn544_dev_irq_handler");
	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}

	pn544_disable_irq(pn544_dev);

	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	memset(tmp, 0, MAX_BUFFER_SIZE);

	printk("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		printk("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		printk("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		printk("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	printk("IFD->PC:");
	for(i = 0; i < ret; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	printk("pn544 i2c addr client->addr = 0x%x\n", pn544_dev->client->addr);
	if (copy_from_user(tmp, buf, count)) {
		printk("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	printk("%s : writing %zu bytes.\n", __func__, count);

	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		printk("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}
	printk("PC->IFD:");
	for(i = 0; i < count; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");

exit:
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;

	printk("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
            
			printk("%s power on with firmware\n", __func__);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
            
			printk("%s power on\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else  if (arg == 0) {

			printk("%s power off\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(1);
		} else {
			printk("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		printk("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl  = pn544_dev_ioctl,
};

static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	int r = 0;
	int ret;

	struct pn544_dev *pn544_dev;

    printk("pn547 enter probe\n");	
    
	printk("nfc probe step01 is ok\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	printk("nfc probe step02 is ok\n");


	printk("nfc probe step03 is ok\n");

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	g_pn544_dev = pn544_dev;
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	printk("nfc probe step04 is ok\n");

	pn544_dev->client   = client;
	ret=nfc_parse_dt(&client->dev,pn544_dev);
	
	pr_info("irq_gpio=%d; ven_gpio=%d; firm_gpio=%d; \n", \
		pn544_dev->irq_gpio,pn544_dev->ven_gpio,pn544_dev->firm_gpio);
	if(ret){
		pr_err("parse node error. pls check the dts file \n");
		ret = -EINVAL;
	}

	pn544_dev->clk_run = false;
	if (1)
	  pn544_clock_select(pn544_dev);

	if (gpio_is_valid(pn544_dev->clkreq_gpio)) {
		r = gpio_request(pn544_dev->clkreq_gpio,
			"nfc_clkreq_gpio");
		if (r) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
					pn544_dev->clkreq_gpio);
			goto err_clkreq_gpio;
		}
		r = gpio_direction_input(pn544_dev->clkreq_gpio);
		if (r) {
			dev_err(&client->dev,
					"unable to set direction for gpio [%d]\n",
					pn544_dev->clkreq_gpio);
			goto err_clkreq_gpio;
		}
	} else {
		dev_err(&client->dev, "clkreq gpio not provided\n");
		goto err_clk;
	}

	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = PN544_NAME;
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		printk("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}
	printk("nfc probe step05 is ok\n");


    if(gpio_request(pn544_dev->irq_gpio,"nfc_int") != 0)
	{
	  printk("PN544: gpio_IRQ_request error\n");
	  goto err_irq;
	}

    if(gpio_request(pn544_dev->ven_gpio,"nfc_ven") != 0)
	{
	  printk("PN544: gpio_VEN_request error\n");
	  goto err_ven;
	}

    if(gpio_request(pn544_dev->firm_gpio,"nfc_firm") != 0)
	{
	  printk("PN544: gpio_firm_request error\n");
	  goto err_firm;
	}
 
	gpio_direction_output(pn544_dev->firm_gpio, 0);
	gpio_direction_output(pn544_dev->ven_gpio, 1);
    printk("nfc probe GPIO is ok\n");
	
	gpio_direction_input(pn544_dev->irq_gpio);
	printk("pn544 client->irq = %d", client->irq);
	client->irq = gpio_to_irq(pn544_dev->irq_gpio);
	printk("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn544_dev);

	if (ret) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}

	printk("nfc probe step06 is ok\n");

	pn544_disable_irq(pn544_dev);

	i2c_set_clientdata(client, pn544_dev);

	printk("nfc probe successful\n");


#if defined(PN544_DEBUG)
		ret = device_create_file(&client->dev, &pn544_dev_attr);
		if (ret) {
			goto err_request_irq_failed;
		}
#endif

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
err_clkreq_gpio:
	 gpio_free(pn544_dev->clkreq_gpio);
err_clk:
	pn544_clock_deselect(pn544_dev);
err_exit:
	gpio_free(pn544_dev->firm_gpio);
err_ven:
	gpio_free(pn544_dev->ven_gpio);
err_irq:
	gpio_free(pn544_dev->irq_gpio);
err_firm:
    gpio_free(pn544_dev->firm_gpio);
    kfree(pn544_dev);	
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);

#if defined(PN544_DEBUG)
	device_remove_file(&client->dev, &pn544_dev_attr);
#endif
	pn544_clock_deselect(pn544_dev);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}

static const struct i2c_device_id pn544_id[] = {
	{ PN544_NAME, 0 },
	{ }
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN544_NAME,
		.of_match_table = msm_match_table,
	},
};


static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Ernest Li");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
