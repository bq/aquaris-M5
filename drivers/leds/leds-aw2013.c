/*
 * leds-aw2013.c - RGB LED Driver
 *
 * Copyright (C) 2009 Samsung Electronics
 * Kim Kyuwon <q1.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.rohm.com/products/databook/driver/pdf/aw2013gu-e.pdf
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>

#define debug 			0
#define AW2013_REG_RSTR		0x00
#define AW2013_REG_GCR			0x01
#define AW2013_REG_LEDE			0x30
#define AW2013_REG_LCFG		0x31
#define AW2013_REG_PWM_LEVEL	0x34
#define AW2013_REG_T0			0x37
#define AW2013_REG_T1			0x38
#define AW2013_REG_T2			0x39


#if defined(CONFIG_VEGETALTE_COMMON)


enum led_colors {
	BLUE,
	GREEN,
	RED,
};

#else
 enum led_colors {
	RED,
	GREEN,
	BLUE,
};

#endif

enum led_bits {
	AW2013_OFF,
	AW2013_BLINK,
	AW2013_ON,
};

enum led_Imax {
	AW2013_0mA,
	AW2013_5mA,
	AW2013_10mA,
	AW2013_15mA,
};

struct aw2013_led_platform_data {
	enum led_Imax		led_current;
	unsigned int			rise_time;
	unsigned int			hold_time;
	unsigned int			fall_time;
	unsigned int			off_time;
	unsigned int			delay_time;
	unsigned int			period_num;
};

struct aw2013_led {
	struct i2c_client		*client;

	struct led_classdev		cdev_ledr;
	struct led_classdev		cdev_ledg;
	struct led_classdev		cdev_ledb;

	struct regulator *vdd;
	struct regulator *vcc_i2c;

	struct aw2013_led_platform_data pdata[3];

	enum led_bits			state_ledr;
	enum led_bits			state_ledg;
	enum led_bits			state_ledb;

	struct delayed_work	work_ledr;
	struct delayed_work	work_ledg;
	struct delayed_work	work_ledb;
};


/*--------------------------------------------------------------*/
/*	AW2013 core functions					*/
/*--------------------------------------------------------------*/

static int aw2013_read_reg(struct i2c_client *client, u8 reg)
{
	int value = i2c_smbus_read_byte_data(client, reg);
	if (value < 0)
		dev_err(&client->dev, "%s: read reg 0x%x err! value=0x%x\n",__func__, reg, value);

	return value;
}

static int aw2013_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret >= 0)
		return 0;

	dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
						__func__, reg, val, ret);

	return ret;
}

static int aw2013_turn_off_led(struct aw2013_led *led, enum led_colors color)
{
	u8 value=0;
	int ret = 0;

	value = aw2013_read_reg(led->client, AW2013_REG_LEDE);
	if(value < 0) return -EINVAL;
	value &= ~(1<<color);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, value);
	usleep(5);
	return ret;
}

static int aw2013_turn_off_all_leds(struct aw2013_led *led)
{
	int ret = 0;
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, 0x00);
	usleep(5);
	return ret;
}

static int aw2013_turn_on_led(struct aw2013_led *led, enum led_colors color)
{
	u8 value=0;
	int ret = 0;

	value = aw2013_read_reg(led->client, AW2013_REG_LEDE);
	value |= (1<<color);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, value);
	usleep(5);
	return ret;
}

/*
static int aw2013_turn_on_all_leds(struct aw2013_led *led)
{
	int ret = 0;
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, 0x07);
	usleep(5);
	return ret;
}
*/

#define LED_VTG_MAX_UV		3300000
#define LED_VTG_MIN_UV		2600000
#define LED_I2C_VTG_MIN_UV	1800000
#define LED_I2C_VTG_MAX_UV	1800000
int aw2013_power_on(struct aw2013_led *led, int enable)
{
	int ret = 0;
	
	led->vdd = regulator_get(&led->client->dev, "vdd");
	if (IS_ERR(led->vdd)) {
		ret = -1;
		dev_err(&led->client->dev,
			"Regulator get failed vdd ret=%d\n", ret);
		return ret;
	}
			
	if (regulator_count_voltages(led->vdd) > 0) {
		ret = regulator_set_voltage(led->vdd, LED_VTG_MIN_UV,	LED_VTG_MAX_UV);
		if (ret) {
			dev_err(&led->client->dev,
				"Regulator set_vtg failed vdd ret=%d\n", ret);
			goto reg_vdd_put;
		}
	}
	
	led->vcc_i2c = regulator_get(&led->client->dev, "vcc_i2c");
	if (IS_ERR(led->vcc_i2c)) {
		ret = PTR_ERR(led->vcc_i2c);
		dev_err(&led->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", ret);
		//goto reg_vdd_set_vtg;
	}
		
	if (regulator_count_voltages(led->vcc_i2c) > 0) {
		ret = regulator_set_voltage(led->vcc_i2c, LED_I2C_VTG_MIN_UV, LED_I2C_VTG_MAX_UV);
		if (ret) {
			dev_err(&led->client->dev,
				"Regulator set_vtg failed vcc_i2c ret=%d\n", ret);
			goto reg_vcc_i2c_put;
		}
	}

	//if(!enable)
		//goto disable_power;

	ret = regulator_enable(led->vdd);		
	if (ret) {
		dev_err(&led->client->dev,
			"Regulator vdd enable failed ret=%d\n", ret);
		return ret;
	}

	ret = regulator_enable(led->vcc_i2c);
	if (ret) {
		dev_err(&led->client->dev,
			"Regulator vcc_i2c enable failed ret=%d\n", ret);
		regulator_disable(led->vdd);
	}
reg_vcc_i2c_put:
	regulator_put(led->vcc_i2c);
reg_vdd_put:
	regulator_put(led->vdd);
	return ret;
}

static int aw2013_enable(struct aw2013_led *led, int enable)
{
	int ret = 0;
	if(!enable){
		ret = aw2013_turn_off_all_leds(led);
		if(ret < 0)
			pr_err("%s can't turn off all leds!\n",__func__);
	}

	ret = aw2013_write_reg(led->client, AW2013_REG_GCR, (u8)enable);
	return ret;
}


static int aw2013_set_led_brightness(struct aw2013_led *led, enum led_colors color, enum led_brightness value)
{
	int ret = 0;

	if(value>255) value = 255;
	if(value<0) value = 0;
	ret = aw2013_write_reg(led->client, AW2013_REG_LCFG+color, (u8)(0x60 |led->pdata[color].led_current));  //mod=direct turn on/off
	ret |= aw2013_write_reg(led->client, AW2013_REG_PWM_LEVEL+color, (u8)value);
	ret |= aw2013_turn_on_led(led, color);
	return ret;
}

static void aw2013_set_ledr_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, cdev_ledr);

	if(led->state_ledr == AW2013_BLINK){
		cancel_delayed_work(&led->work_ledr);
		aw2013_turn_off_led(led, RED);
	}

	if (value == LED_OFF){
		led->state_ledr = AW2013_OFF;
		aw2013_turn_off_led(led, RED);
	}
	else{
		led->state_ledr = AW2013_ON;
		aw2013_set_led_brightness(led, RED, value);
	}
}

static enum led_brightness aw2013_get_ledr_brightness(struct led_classdev *led_cdev)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, cdev_ledr);

	return led->cdev_ledr.brightness;
}

static void aw2013_set_ledg_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, cdev_ledg);

	if(led->state_ledg == AW2013_BLINK){
		cancel_delayed_work(&led->work_ledg);
		aw2013_turn_off_led(led, GREEN);
	}

	if (value == LED_OFF){
		led->state_ledg = AW2013_OFF;
		aw2013_turn_off_led(led, GREEN);
	}else{
		led->state_ledg = AW2013_ON;
		aw2013_set_led_brightness(led, GREEN, value);
	}
}

static enum led_brightness aw2013_get_ledg_brightness(struct led_classdev *led_cdev)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, cdev_ledg);

	return led->cdev_ledg.brightness;
}

static void aw2013_set_ledb_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, cdev_ledb);

	if(led->state_ledb == AW2013_BLINK){
		cancel_delayed_work(&led->work_ledb);
		aw2013_turn_off_led(led, BLUE);
	}

	if (value == LED_OFF){
		led->state_ledb = AW2013_OFF;
		aw2013_turn_off_led(led, BLUE);
	}
	else{
		led->state_ledb = AW2013_ON;
		aw2013_set_led_brightness(led, BLUE, value);
	}
}

static enum led_brightness aw2013_get_ledb_brightness(struct led_classdev *led_cdev)
{
	struct aw2013_led *led = container_of(led_cdev, struct aw2013_led, cdev_ledb);

	return led->cdev_ledb.brightness;
}

static u8 aw2013_get_level(int time)
{
	int level = -1;
	if (time < 150)
		level = 0;
	else if (time < 260)
		level = 1;
	else if (time < 520)
		level = 2;
	else if (time < 1020)
		level = 3;
	else if (time < 2080)
		level = 4;
	else if (time < 4160)
		level = 5;
	else if (time < 8320)
		level = 6;
	else 
		level = 7;

	if (level > 0)
		level = level -1;
	
	return level;
}

static u8 aw2013_get_imax_level(int imax)
{
	int level = 0;
	if (imax < 5)
		level = 0;
	else if (imax < 65)
		level = 1;
	else if (imax < 129)
		level = 2;
	else 
		level = 3;

	return level;
}

static int aw2013_set_led_blink(struct aw2013_led *led, enum led_colors color,
								unsigned int rising_time, unsigned int hold_time,
								unsigned int falling_time, unsigned int off_time,
								unsigned int delay_time, unsigned int period_num,
								unsigned int brightness)
{
	int ret = 0;
	u8 period = 0;
	u8 state_led = 0x00;
	u8 rising_level, falling_level, hold_level, off_level, delay_level, brightness_level;
	u8 value;

	state_led = aw2013_read_reg(led->client, AW2013_REG_LEDE);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, state_led & 0xf8);

//	aw2013_set_led_brightness(led, color, brightness);
	
	rising_level = aw2013_get_level(rising_time);
	hold_level = aw2013_get_level(hold_time);
	falling_level = aw2013_get_level(falling_time);
	off_level = aw2013_get_level(off_time);
	delay_level = aw2013_get_level(delay_time);
	period = period_num & 0xf;
	if (brightness > 10 && brightness < 250)
	{
		rising_level += 1;
		value = 127;
		falling_level += 1;
	}
 	else if (255 == brightness)
 	{
 		value = 254;
 	}
	
	ret |= aw2013_write_reg(led->client, AW2013_REG_PWM_LEVEL+color, (u8)value);
	
	/* Limit maximum Current to MAX_BRIGHT_CURR */
	brightness_level = aw2013_get_imax_level(brightness);

	if(debug)
 	{
		printk(KERN_ERR " %s  led->cdev_ledr.brightness  %d  led->cdev_ledg.brightness  %d  led->cdev_ledb.brightness %d \n", 
			__func__,  led->cdev_ledr.brightness,  led->cdev_ledg.brightness,  led->cdev_ledb.brightness );
			printk(KERN_ERR "%s color %d  rising_time %d hold_time %d falling_time %d off_time %d delay_time %d period_num %d\n", 
			__func__, color, rising_time, hold_time, falling_time, off_time, delay_time, period_num);
	}
	
	ret |= aw2013_write_reg(led->client, AW2013_REG_LCFG+color, (0x1 << 4) | brightness_level);  //mod=flash

	//ret |= aw2013_write_reg(led->client, AW2013_REG_PWM_LEVEL+color*3, (u8)brightness);

	ret |= aw2013_write_reg(led->client, AW2013_REG_T0+color*3, (u8)((rising_level <<4) |hold_level));
	ret |= aw2013_write_reg(led->client, AW2013_REG_T1+color*3, (u8)((falling_level <<4) |off_level));
	ret |= aw2013_write_reg(led->client, AW2013_REG_T2+color*3, (u8)((delay_level <<4) |period));

	//ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, 0x1<<color);
	ret = aw2013_write_reg(led->client, AW2013_REG_LEDE, (u8)(state_led |  (1<<color)));
	usleep(5);
	return ret;
}

static int aw2013_set_led_blink_time(struct led_classdev *led_cdev, unsigned long *delay_on, unsigned long *delay_off)
{
	int ret = 0;

	if (*delay_on == 0 || *delay_off == 0){
		*delay_on = led_cdev->blink_delay_on;
		*delay_off = led_cdev->blink_delay_off;
	}else{
		led_cdev->blink_delay_on = *delay_on;
		led_cdev->blink_delay_off = *delay_off;
	}

	return ret;
}

static void aw2013_switch_ledr_blink_work(struct work_struct *work)
{
	struct aw2013_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;

	led = container_of(to_delayed_work(work),
		struct aw2013_led, work_ledr);

	if (!led) {
		pr_err("%s: led data not available\n", __func__);
		return;
	}

	if(led->cdev_ledr.brightness_get(&led->cdev_ledr) != LED_OFF){
		switch_brightness = LED_OFF;
		delay_ms = led->cdev_ledr.blink_delay_off;
	}else{
		switch_brightness = led->cdev_ledr.blink_brightness;
		delay_ms = led->cdev_ledr.blink_delay_on;
	}

	led->cdev_ledr.brightness = switch_brightness;
	aw2013_set_led_brightness(led, RED, switch_brightness);

	schedule_delayed_work(&led->work_ledr, msecs_to_jiffies(delay_ms));

}

static void aw2013_switch_ledg_blink_work(struct work_struct *work)
{
	struct aw2013_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;

	led = container_of(to_delayed_work(work),
		struct aw2013_led, work_ledg);

	if (!led) {
		pr_err("%s: led data not available\n", __func__);
		return;
	}

	if(led->cdev_ledg.brightness_get(&led->cdev_ledg) != LED_OFF){
		switch_brightness = LED_OFF;
		delay_ms = led->cdev_ledg.blink_delay_off;
	}else{
		switch_brightness = led->cdev_ledg.blink_brightness;
		delay_ms = led->cdev_ledg.blink_delay_on;
	}

	led->cdev_ledg.brightness = switch_brightness;
	aw2013_set_led_brightness(led, GREEN, switch_brightness);

	schedule_delayed_work(&led->work_ledg, msecs_to_jiffies(delay_ms));

}

static void aw2013_switch_ledb_blink_work(struct work_struct *work)
{
	struct aw2013_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;

	led = container_of(to_delayed_work(work),
		struct aw2013_led, work_ledb);

	if (!led) {
		pr_err("%s: led data not available\n", __func__);
		return;
	}

	if(led->cdev_ledb.brightness_get(&led->cdev_ledb) != LED_OFF){
		switch_brightness = LED_OFF;
		delay_ms = led->cdev_ledb.blink_delay_off;
	}else{
		switch_brightness = led->cdev_ledb.blink_brightness;
		delay_ms = led->cdev_ledb.blink_delay_on;
	}

	led->cdev_ledb.brightness = switch_brightness;
	aw2013_set_led_brightness(led, BLUE, switch_brightness);

	schedule_delayed_work(&led->work_ledb, msecs_to_jiffies(delay_ms));

}

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led;
	unsigned int value;
	unsigned int ontime = 2;
	unsigned int offtime = 2;
	unsigned int rise_time;
	unsigned int fall_time;

	ssize_t ret = -EINVAL;

	if (sscanf(buf,"%d %d %d",&value,&ontime,&offtime) < 0) {
		dev_err(led_cdev->dev, "%s fail\n", __FUNCTION__);
		return ret;
	}
//	printk(" led %s  led_cdev->name: %s value :%d, ontime :%d,, offtime :%d,\n", __func__, led_cdev->name, value, ontime, offtime);

   if (ontime > offtime)
        rise_time = fall_time = offtime / 2;
    else
        rise_time = fall_time = ontime / 2;

	if(debug)
 	{
		printk(KERN_ERR "%s  rise_time:%d fall_time:%d \n", __func__, rise_time, fall_time);
	}


//	printk("%s  hold_time %d \n", __func__, hold_time);

	if(!strcmp(led_cdev->name,"red")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledr);
		if(!value){
			led->state_ledr = AW2013_OFF;
			cancel_delayed_work(&led->work_ledr);
			aw2013_turn_off_led(led,RED);
		}else{
		
				led->state_ledr = AW2013_BLINK;
				
			ret = aw2013_set_led_blink(led,RED,
							rise_time,/*led->pdata[RED].rise_time,*/
							ontime, /*led->pdata[RED].hold_time,*/
							fall_time,/*led->pdata[RED].fall_time,*/
							offtime,
							led->pdata[RED].delay_time,
							led->pdata[RED].period_num, value);
		}
	}else if(!strcmp(led_cdev->name,"green")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledg);
		if(!value){
			led->state_ledg = AW2013_OFF;
			cancel_delayed_work(&led->work_ledg);
			aw2013_turn_off_led(led,GREEN);
		}else{
				led->state_ledg = AW2013_BLINK;
			ret = aw2013_set_led_blink(led,GREEN,
							rise_time, /*led->pdata[GREEN].rise_time,*/
							ontime, /*led->pdata[GREEN].hold_time,*/
							fall_time, /*led->pdata[GREEN].fall_time,*/
							offtime, /*led->pdata[GREEN].off_time,*/
							led->pdata[GREEN].delay_time,
							led->pdata[GREEN].period_num, value);
		}
	}else if(!strcmp(led_cdev->name,"blue")){
		led = container_of(led_cdev, struct aw2013_led, cdev_ledb);
		if(!value){
			led->state_ledb = AW2013_OFF;
			cancel_delayed_work(&led->work_ledb);
			aw2013_turn_off_led(led,GREEN);
		}else{
				led->state_ledb = AW2013_BLINK;
			ret = aw2013_set_led_blink(led,BLUE,
							rise_time, /*led->pdata[BLUE].rise_time,*/
							ontime, /*led->pdata[BLUE].hold_time,*/
							fall_time, /*led->pdata[BLUE].fall_time,*/
							offtime, /*led->pdata[BLUE].off_time,*/
							led->pdata[BLUE].delay_time,
							led->pdata[BLUE].period_num, value);
		}
	}else{
		pr_err("%s invalid led color!\n",__func__);
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(blink, 0664, NULL, blink_store);

static struct attribute *blink_attrs[] = {
	&dev_attr_blink.attr,
	NULL
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static int aw2013_register_led_classdev(struct aw2013_led *led)
{
	int ret=0;

	led->cdev_ledr.name = "red";
	led->cdev_ledr.brightness = LED_OFF;
	led->cdev_ledr.max_brightness = LED_FULL;
	led->cdev_ledr.blink_brightness = LED_HALF;
	led->cdev_ledr.blink_delay_on = 1000;
	led->cdev_ledr.blink_delay_off = 3000;
	led->cdev_ledr.brightness_set = aw2013_set_ledr_brightness;
	led->cdev_ledr.brightness_get = aw2013_get_ledr_brightness;
	led->cdev_ledr.blink_set = aw2013_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledr);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledr.name);
		goto failed_unregister_led1_R;
	}
	ret = sysfs_create_group(&led->cdev_ledr.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_R;

	INIT_DELAYED_WORK(&led->work_ledr, aw2013_switch_ledr_blink_work);

	led->cdev_ledg.name = "green";
	led->cdev_ledg.brightness = LED_OFF;
	led->cdev_ledg.max_brightness = LED_FULL;
	led->cdev_ledg.blink_brightness = LED_HALF;
	led->cdev_ledg.blink_delay_on = 1000;
	led->cdev_ledg.blink_delay_off = 3000;
	led->cdev_ledg.brightness_set = aw2013_set_ledg_brightness;
	led->cdev_ledg.brightness_get = aw2013_get_ledg_brightness;
	led->cdev_ledg.blink_set = aw2013_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledg);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledg.name);
		goto failed_unregister_led1_G;
	}
	ret = sysfs_create_group(&led->cdev_ledg.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_G;

	INIT_DELAYED_WORK(&led->work_ledg, aw2013_switch_ledg_blink_work);

	led->cdev_ledb.name = "blue";
	led->cdev_ledb.brightness = LED_OFF;
	led->cdev_ledb.max_brightness = LED_FULL;
	led->cdev_ledb.blink_brightness = LED_HALF;
	led->cdev_ledb.blink_delay_on = 1000;
	led->cdev_ledb.blink_delay_off = 3000;
	led->cdev_ledb.brightness_set = aw2013_set_ledb_brightness;
	led->cdev_ledb.brightness_get = aw2013_get_ledb_brightness;
	led->cdev_ledb.blink_set = aw2013_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledb);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledb.name);
		goto failed_unregister_led1_B;
	}
	ret = sysfs_create_group(&led->cdev_ledb.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_B;

	INIT_DELAYED_WORK(&led->work_ledb, aw2013_switch_ledb_blink_work);

	return 0;

failed_unregister_led1_B:
	led_classdev_unregister(&led->cdev_ledg);
failed_unregister_led1_G:
	led_classdev_unregister(&led->cdev_ledr);
failed_unregister_led1_R:

	return ret;
}

static void aw2013_unregister_led_classdev(struct aw2013_led *led)
{
	led_classdev_unregister(&led->cdev_ledb);
	led_classdev_unregister(&led->cdev_ledg);
	led_classdev_unregister(&led->cdev_ledr);
	cancel_delayed_work(&led->work_ledr);
	cancel_delayed_work(&led->work_ledg);
	cancel_delayed_work(&led->work_ledb);
}

static int aw2013_led_parse_dt_platform(struct device_node *np,
				const char *prop_name,
				struct aw2013_led_platform_data *pdata)
{
	struct property *prop;
	int rc,len;
	u32 tmp[10];

	prop = of_find_property(np, prop_name, &len);
	len = len/sizeof(u32);
	if (!prop || len < 1) {
		pr_err("prop %s : doesn't exist in device tree\n",prop_name);
		return -ENODEV;
	}

	rc = of_property_read_u32_array(np, prop_name, tmp, len);
	if (rc){
		pr_err("%s:%d, error reading %s, rc = %d\n",
			__func__, __LINE__, prop_name, rc);
		return -EINVAL;
	}

	pdata->led_current = tmp[0];
	pdata->rise_time = tmp[1];
	pdata->hold_time = tmp[2];
	pdata->fall_time = tmp[3];
	pdata->off_time = tmp[4];
	pdata->delay_time = tmp[5];
	pdata->period_num = tmp[6];

	return 0;
}

static int aw2013_led_parse_dt(struct device *dev,
				struct aw2013_led *led)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = aw2013_led_parse_dt_platform(np,"awinic,ledr-parameter_array",&led->pdata[RED]);
	if (rc){
		pr_err("%s:%d, error reading RED light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = aw2013_led_parse_dt_platform(np,"awinic,ledg-parameter_array",&led->pdata[GREEN]);
	if (rc){
		pr_err("%s:%d, error reading GREEN light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = aw2013_led_parse_dt_platform(np,"awinic,ledb-parameter_array",&led->pdata[BLUE]);
	if (rc){
		pr_err("%s:%d, error reading BLUE light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	return 0;
}

static int aw2013_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct aw2013_led *led;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}
	
	led = devm_kzalloc(&client->dev, sizeof(struct aw2013_led), GFP_KERNEL);
	if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		memset(&led->pdata, 0 , sizeof(led->pdata));
		ret = aw2013_led_parse_dt(&client->dev, led);
		if (ret) {
			dev_err(&client->dev,"Unable to parse platfrom data ret=%d\n", ret);
			ret = -EINVAL;
			goto err_exit;
		}
	} else {
		if (client->dev.platform_data)
			memcpy(&led->pdata, client->dev.platform_data, sizeof(led->pdata));
		else {
			dev_err(&client->dev,"platform data is NULL; exiting\n");
			ret = -EINVAL;
			goto err_exit;
		}
	}

	led->client = client;
	i2c_set_clientdata(client, led);

	aw2013_power_on(led, 1);

	/* Detect AW2013GU */
	ret = aw2013_read_reg(client, AW2013_REG_RSTR);
	if (ret != 0x33) {
		dev_err(&client->dev, "failed to detect device ret=0x%x\n",ret);
		goto err_exit;
	}

	/* Reset chip */
	ret = aw2013_write_reg(client, AW2013_REG_RSTR,0x55);
	if (ret < 0) {
		dev_err(&client->dev, "failed to reset device ret=0x%x\n",ret);
		goto err_exit;
	}

	/* Enable aw2013 sub module */
	aw2013_enable(led, 1);

	/* register class dev */
	ret = aw2013_register_led_classdev(led);
	if (ret < 0)
		goto err_exit;

	return 0;

err_exit:
	devm_kfree(&client->dev, led);

	return ret;
}

static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013_led *led = i2c_get_clientdata(client);

	aw2013_unregister_led_classdev(led);
	devm_kfree(&client->dev, led);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int aw2013_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw2013_led *led = i2c_get_clientdata(client);

	if((led->state_ledr != AW2013_OFF)
		|| (led->state_ledg != AW2013_OFF)
		|| (led->state_ledb != AW2013_OFF) )
		return 0;

	aw2013_enable(led, 0);

	return 0;
}

static int aw2013_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw2013_led *led = i2c_get_clientdata(client);

	aw2013_enable(led, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(aw2013_pm, aw2013_suspend, aw2013_resume);

static const struct i2c_device_id aw2013_id[] = {
	{ "aw2013", 0 },
	{ }
};

// #ifdef CONFIG_OF
static struct of_device_id bd_match_table[] = {
		{ .compatible = "awinic,aw2013",},
		{ },
};

MODULE_DEVICE_TABLE(i2c, aw2013_id);

static struct i2c_driver aw2013_i2c_driver = {
	.driver	= {
		.name	= "aw2013",
		.pm	= &aw2013_pm,
		.of_match_table = bd_match_table,
	},
	.probe		= aw2013_probe,
	.remove		= aw2013_remove,
	.id_table	= aw2013_id,
};

static int aw2013_driver_init(void)
{
	return i2c_add_driver(&aw2013_i2c_driver);
};

static void aw2013_driver_exit(void)
{
	i2c_del_driver(&aw2013_i2c_driver);
}

module_init(aw2013_driver_init);
module_exit(aw2013_driver_exit);

//module_i2c_driver(aw2013_i2c_driver);

MODULE_AUTHOR("Kim Kyuwon <q1.kim@samsung.com>");
MODULE_DESCRIPTION("AW2013 LED driver");
MODULE_LICENSE("GPL v2");
