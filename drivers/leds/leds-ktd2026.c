/*
 * leds-ktd.c - RGB LED Driver
 *
 * Copyright (C) 2009 Samsung Electronics
 * Kim Kyuwon <q1.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.rohm.com/products/databook/driver/pdf/ktdgu-e.pdf
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
#include <linux/of_gpio.h>
#include <linux/of.h>

#define debug 0
#define KTD_REG_RSTR		0x00
#define KTD_REG_GCR			0x00
#define KTD_REG_LEDE		0x04
#define KTD_REG_LCFG		0x06
//#define KTD_REG_PWM_LEVEL	0x34
#define KTD_REG_R_F			0x05
#define KTD_REG_HOLD		0x01
#define KTD_REG_PWM1		0x02
#define KTD_REG_PWM2		0x03

#define MS_TO_LSB      (128)
#define MIN_DC_VALUE	4
#define MIN_RAMP_VALUE	96
#define RISE_TIME_MASK	0xF0
#define FALL_TIME_MASK	0x0F

#define KTD_I2C_NAME			"ktd2026"




 enum led_colors {
	GREEN,
	RED,
	BLUE,
	BUTTON,
};


enum led_bits {
	KTD_OFF,
	KTD_ON,
	KTD_BLINK,
};

enum led_Imax {
	KTD_0mA,
	KTD_5mA,
	KTD_10mA,
	KTD_15mA,
};

struct ktd20xx_led_platform_data {
	enum led_Imax			led_current;
	unsigned int			rise_time;
	unsigned int			hold_time;
	unsigned int			fall_time;
	unsigned int			off_time;
	unsigned int			delay_time;
	unsigned int			period_num;
	unsigned int			led_number;
};

struct ktd20xx_led {
	struct i2c_client		*client;

	struct led_classdev		cdev_ledr;
	struct led_classdev		cdev_ledg;
	struct led_classdev		cdev_ledb;
	struct led_classdev		cdev_ledbutton;

	struct regulator *vdd;
	struct regulator *vcc_i2c;

	struct ktd20xx_led_platform_data pdata[4];

	enum led_bits			state_ledr;
	enum led_bits			state_ledg;
	enum led_bits			state_ledb;
	enum led_bits			state_ledbutton;

	struct delayed_work	work_ledr;
	struct delayed_work	work_ledg;
	struct delayed_work	work_ledb;
};


/*--------------------------------------------------------------*/
/*	KTD core functions					*/
/*--------------------------------------------------------------*/

/*
static int ktd20xx_read_reg(struct i2c_client *client, u8 reg)
{
	int value = i2c_smbus_read_byte_data(client, reg);
	if (value < 0)
		dev_err(&client->dev, "%s: read reg 0x%x err! value=0x%x\n",__func__, reg, value);

	return value;
}
*/
static int ktd20xx_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret >= 0)
		return 0;

	dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
						__func__, reg, val, ret);

	return ret;
}

static int ktd20xx_color_to_led_position(struct ktd20xx_led *led, enum led_colors color) {
	return (led->pdata[color].led_number - 1);
}

static int ktd20xx_state_led_bit(struct ktd20xx_led *led, enum led_bits state,
							enum led_colors color) {
	return state << (ktd20xx_color_to_led_position(led, color) * 2);
}


static int ktd20xx_turn_on_led(struct ktd20xx_led *led)
{	
	u8 state_led = 0x00;
	int ret = 0;

	state_led |= ktd20xx_state_led_bit(led, led->state_ledr, RED);
	state_led |= ktd20xx_state_led_bit(led, led->state_ledg, GREEN);
	state_led |= ktd20xx_state_led_bit(led, led->state_ledb, BLUE);
	state_led |= ktd20xx_state_led_bit(led, led->state_ledbutton, BUTTON);

	ret = ktd20xx_write_reg(led->client, KTD_REG_LEDE, state_led);

	if (state_led == 0x00) {
		ret = ktd20xx_write_reg(led->client, KTD_REG_RSTR, 0x08);
	}
	
	usleep(5);
	return ret;
}

static int ktd20xx_turn_off_led(struct ktd20xx_led *led, enum led_colors color)
{
	return ktd20xx_turn_on_led(led);
}

#define LED_VTG_MAX_UV		3300000
#define LED_VTG_MIN_UV		2600000
#define LED_I2C_VTG_MIN_UV	1800000
#define LED_I2C_VTG_MAX_UV	1800000
int ktd20xx_power_on(struct ktd20xx_led *led, int enable)
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
	}
		
	if (regulator_count_voltages(led->vcc_i2c) > 0) {
		ret = regulator_set_voltage(led->vcc_i2c, LED_I2C_VTG_MIN_UV, LED_I2C_VTG_MAX_UV);
		if (ret) {
			dev_err(&led->client->dev,
				"Regulator set_vtg failed vcc_i2c ret=%d\n", ret);
			goto reg_vcc_i2c_put;
		}
	}

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

static int ktd20xx_enable(struct ktd20xx_led *led, int enable)
{
	int ret = 0;
	if (!enable) {
		if(ret < 0)
			pr_err("%s can't turn off all leds!\n",__func__);
	}

	return ret;
}


static int ktd20xx_set_led_brightness(struct ktd20xx_led *led, enum led_colors color, enum led_brightness value)
{
	int ret = 0;
	if (debug) {
		printk(KERN_ERR "%s %d color %d value %d \n", __func__, __LINE__, color, value);
	}

	ret |= ktd20xx_write_reg(led->client, KTD_REG_LCFG + ktd20xx_color_to_led_position(led, color), (u8)value);
	ret |= ktd20xx_write_reg(led->client, KTD_REG_RSTR, 0x00);
	ret |= ktd20xx_turn_on_led(led);
	return ret;
}

static void ktd20xx_set_ledr_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledr);

	if (debug) {
		printk(KERN_ERR "%s line=%d value=%d\n", __func__, __LINE__, value);
	}

	if (led->state_ledr == KTD_BLINK) {
		cancel_delayed_work(&led->work_ledr);
		ktd20xx_turn_off_led(led, RED);
	}
	
	if (value > 255)
		value = 255;
	if(value < 0)
		value = 0;

	if (value == LED_OFF) {
		led->state_ledr = KTD_OFF;
		ktd20xx_turn_off_led(led,  RED);
	} else {
		led->cdev_ledr.brightness = value;
		led->state_ledr = KTD_ON;
		ktd20xx_set_led_brightness(led, RED, value);
	}
}

static enum led_brightness ktd20xx_get_ledr_brightness(struct led_classdev *led_cdev)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledr);

	return led->cdev_ledr.brightness;
}

static void ktd20xx_set_ledg_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledg);
	if(debug)
 	{
 		printk(KERN_ERR "%s %d value %d \n", __func__, __LINE__, value);

	}
	if(led->state_ledg == KTD_BLINK){
		cancel_delayed_work(&led->work_ledg);
		ktd20xx_turn_off_led(led, GREEN);
	}
	
	if(value>255) value = 255;
	if(value<0) value = 0;
	

	if (value == LED_OFF){
		led->state_ledg = KTD_OFF;
		ktd20xx_turn_off_led(led, GREEN);
	}else
	{
		led->cdev_ledg.brightness = value;

		led->state_ledg = KTD_ON;
		ktd20xx_set_led_brightness(led, GREEN, value);
	}
}

static enum led_brightness ktd20xx_get_ledg_brightness(struct led_classdev *led_cdev)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledg);

	return led->cdev_ledg.brightness;
}

static void ktd20xx_set_ledb_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledb);

	if(debug)
 	{
 		printk(KERN_ERR "%s %d value %d \n", __func__, __LINE__, value);

	}

	if(led->state_ledb == KTD_BLINK){
		cancel_delayed_work(&led->work_ledb);
		ktd20xx_turn_off_led(led, BLUE);
	}
	
	if(value>255) value = 255;
	if(value<0) value = 0;
	

	if (value == LED_OFF){
		led->state_ledb = KTD_OFF;
		ktd20xx_turn_off_led(led, BLUE);
	}
	else
	{
		led->cdev_ledb.brightness = value;
		led->state_ledb = KTD_ON;
		ktd20xx_set_led_brightness(led, BLUE, value);
	}
}

static enum led_brightness ktd20xx_get_ledb_brightness(struct led_classdev *led_cdev)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledb);

	return led->cdev_ledb.brightness;
}

static void ktd20xx_set_ledbutton_brightness(struct led_classdev *led_cdev, enum led_brightness value)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledbutton);

	printk(KERN_ERR "%s %d value %d \n", __func__, __LINE__, value);

	if(debug) {
		printk(KERN_ERR "%s %d value %d \n", __func__, __LINE__, value);
	}

	if (value > 120)
		value = 120;
	if (value < 0)
		value = 0;

	if (value == LED_OFF){
		led->state_ledbutton = KTD_OFF;
		ktd20xx_turn_off_led(led, BUTTON);
	} else {
		led->cdev_ledbutton.brightness = value;
		led->state_ledbutton = KTD_ON;
		ktd20xx_set_led_brightness(led, BUTTON, value);
	}
}

static enum led_brightness ktd20xx_get_ledbutton_brightness(struct led_classdev *led_cdev)
{
	struct ktd20xx_led *led = container_of(led_cdev, struct ktd20xx_led, cdev_ledbutton);

	return led->cdev_ledbutton.brightness;
}

static int ktd20xx_set_led_blink(struct ktd20xx_led *led, enum led_colors color,
								unsigned int rising_time, unsigned int hold_time,
								unsigned int falling_time, unsigned int off_time,
								unsigned int delay_time, unsigned int period_num,
								unsigned int brightness)
{
	int ret = 0;
	u8 period = 0;
	u8 duty_cycle = 0;
	u8 rise_time, fall_time;
	u8 ramp_times;

	period = (hold_time + off_time)/MS_TO_LSB;

	if (period < 2)
		period = 0;
	else
		period = period - 2;

	if (period > 127)
		period = 127;

	duty_cycle = ((hold_time * 1000) / (hold_time + off_time))/ MIN_DC_VALUE;

	rise_time = (rising_time / MIN_RAMP_VALUE);
	if (rise_time > 15)
		rise_time = 15;

	fall_time = (falling_time / MIN_RAMP_VALUE);
	if (fall_time > 15)
		fall_time = 15;

	ramp_times = (rise_time << 4 & RISE_TIME_MASK) | (fall_time & FALL_TIME_MASK);

	if(debug)
 	{
		printk(KERN_ERR "%s  led->cdev_ledr.brightness  %d  led->cdev_ledg.brightness  %d  led->cdev_ledb.brightness %d \n", 
			__func__,  led->cdev_ledr.brightness,  led->cdev_ledg.brightness,  led->cdev_ledb.brightness );
		printk(KERN_ERR "%s color %d  rising_time %d hold_time %d falling_time %d off_time %d delay_time %d period_num %d\n", 
			__func__, color, rising_time, hold_time, falling_time, off_time, delay_time, period_num);
	}

	ktd20xx_write_reg(led->client, KTD_REG_RSTR, 0x00);// mode set---IC work when both SCL and SDA goes high
	ktd20xx_write_reg(led->client, KTD_REG_LCFG + ktd20xx_color_to_led_position(led, color), brightness);
	
	ret |= ktd20xx_write_reg(led->client, KTD_REG_R_F,  ramp_times);
	ret |= ktd20xx_write_reg(led->client, KTD_REG_HOLD, period);
	ktd20xx_write_reg(led->client, KTD_REG_PWM1, duty_cycle);//reset internal counter

	ktd20xx_turn_on_led(led);

	if(debug) {
		printk(KERN_ERR "%s  led->state_ledr %d led->state_ledg %d  led->state_ledb %d\n", 
			__func__, led->state_ledr, led->state_ledg, led->state_ledb);
	}

	return ret;
}



static int ktd20xx_set_led_blink_time(struct led_classdev *led_cdev, unsigned long *delay_on, unsigned long *delay_off)
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

static void ktd20xx_switch_ledr_blink_work(struct work_struct *work)
{
	struct ktd20xx_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;
	if(debug)
 	{
 		printk(KERN_ERR "%s %d \n", __func__, __LINE__);

	}
	led = container_of(to_delayed_work(work),
		struct ktd20xx_led, work_ledr);

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
	if(debug)
 	{
 		printk(KERN_ERR "%s %d switch_brightness %d \n", __func__, __LINE__, switch_brightness);

	}
	led->cdev_ledr.brightness = switch_brightness;
	ktd20xx_set_led_brightness(led, RED, switch_brightness);

	schedule_delayed_work(&led->work_ledr, msecs_to_jiffies(delay_ms));

}

static void ktd20xx_switch_ledg_blink_work(struct work_struct *work)
{
	struct ktd20xx_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;
	if(debug)
 	{
 		printk(KERN_ERR "%s %d \n", __func__, __LINE__);

	}
	led = container_of(to_delayed_work(work),
		struct ktd20xx_led, work_ledg);

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
	ktd20xx_set_led_brightness(led, GREEN, switch_brightness);

	schedule_delayed_work(&led->work_ledg, msecs_to_jiffies(delay_ms));

}

static void ktd20xx_switch_ledb_blink_work(struct work_struct *work)
{
	struct ktd20xx_led *led = NULL;
	int switch_brightness = 0;
	int delay_ms=0;
	if(debug)
 	{
 		printk(KERN_ERR "%s %d \n", __func__, __LINE__);

	}
	led = container_of(to_delayed_work(work),
		struct ktd20xx_led, work_ledb);

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
	ktd20xx_set_led_brightness(led, BLUE, switch_brightness);

	schedule_delayed_work(&led->work_ledb, msecs_to_jiffies(delay_ms));

}

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct ktd20xx_led *led;
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

	if (ontime > offtime)
		rise_time = fall_time = offtime / 2;
	else
		rise_time = fall_time = ontime / 2;

	if(debug) {
		printk(KERN_ERR "%s  rise_time:%d fall_time:%d \n", __func__, rise_time, fall_time);
	}
	if (!strcmp(led_cdev->name,"red")) {
		led = container_of(led_cdev, struct ktd20xx_led, cdev_ledr);

	if (!value) {
			led->state_ledr = KTD_OFF;
			cancel_delayed_work(&led->work_ledr);
			ktd20xx_turn_off_led(led,RED);
		} else {
			led->state_ledr = KTD_BLINK;
			ret = ktd20xx_set_led_blink(led,RED,
							rise_time,/*led->pdata[RED].rise_time,*/
							ontime, /*led->pdata[RED].hold_time,*/
							fall_time,/*led->pdata[RED].fall_time,*/
							offtime,
							led->pdata[RED].delay_time,
							led->pdata[RED].period_num, value);
		}
	} else if (!strcmp(led_cdev->name, "green")) {
		led = container_of(led_cdev, struct ktd20xx_led, cdev_ledg);
		if(!value){
			led->state_ledg = KTD_OFF;
			cancel_delayed_work(&led->work_ledg);
			ktd20xx_turn_off_led(led,GREEN);
		}else{
			led->state_ledg = KTD_BLINK;
			ret = ktd20xx_set_led_blink(led,GREEN,
							rise_time, /*led->pdata[GREEN].rise_time,*/
							ontime, /*led->pdata[GREEN].hold_time,*/
							fall_time, /*led->pdata[GREEN].fall_time,*/
							offtime, /*led->pdata[GREEN].off_time,*/
							led->pdata[GREEN].delay_time,
							led->pdata[GREEN].period_num, value);
		}
	} else if (!strcmp(led_cdev->name, "blue")){
		led = container_of(led_cdev, struct ktd20xx_led, cdev_ledb);
		if (!value) {
			led->state_ledb = KTD_OFF;
			cancel_delayed_work(&led->work_ledb);
			ktd20xx_turn_off_led(led,GREEN);
		} else {
			led->state_ledb = KTD_BLINK;
			ret = ktd20xx_set_led_blink(led,BLUE,
							rise_time, /*led->pdata[BLUE].rise_time,*/
							ontime, /*led->pdata[BLUE].hold_time,*/
							fall_time, /*led->pdata[BLUE].fall_time,*/
							offtime, /*led->pdata[BLUE].off_time,*/
							led->pdata[BLUE].delay_time,
							led->pdata[BLUE].period_num, value);
		}
	} else {
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

static int ktd20xx_register_led_classdev(struct ktd20xx_led *led)
{
	int ret=0;

	led->cdev_ledr.name = "red";
	led->cdev_ledr.brightness = led->pdata[RED].led_current;
	led->cdev_ledr.max_brightness = LED_FULL;
	led->cdev_ledr.blink_brightness = LED_HALF;
	led->cdev_ledr.blink_delay_on = 1000;
	led->cdev_ledr.blink_delay_off = 3000;
	led->cdev_ledr.brightness_set = ktd20xx_set_ledr_brightness;
	led->cdev_ledr.brightness_get = ktd20xx_get_ledr_brightness;
	led->cdev_ledr.blink_set = ktd20xx_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledr);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledr.name);
		goto failed_unregister_led1_R;
	}
	ret = sysfs_create_group(&led->cdev_ledr.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_R;

	INIT_DELAYED_WORK(&led->work_ledr, ktd20xx_switch_ledr_blink_work);

	led->cdev_ledg.name = "green";
	led->cdev_ledg.brightness =  led->pdata[GREEN].led_current;
	led->cdev_ledg.max_brightness = LED_FULL;
	led->cdev_ledg.blink_brightness = LED_HALF;
	led->cdev_ledg.blink_delay_on = 1000;
	led->cdev_ledg.blink_delay_off = 3000;
	led->cdev_ledg.brightness_set = ktd20xx_set_ledg_brightness;
	led->cdev_ledg.brightness_get = ktd20xx_get_ledg_brightness;
	led->cdev_ledg.blink_set = ktd20xx_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledg);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledg.name);
		goto failed_unregister_led1_G;
	}
	ret = sysfs_create_group(&led->cdev_ledg.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_G;

	INIT_DELAYED_WORK(&led->work_ledg, ktd20xx_switch_ledg_blink_work);

	led->cdev_ledb.name = "blue";
	led->cdev_ledb.brightness =  led->pdata[BLUE].led_current;
	led->cdev_ledb.max_brightness = LED_FULL;
	led->cdev_ledb.blink_brightness = LED_HALF;
	led->cdev_ledb.blink_delay_on = 1000;
	led->cdev_ledb.blink_delay_off = 3000;
	led->cdev_ledb.brightness_set = ktd20xx_set_ledb_brightness;
	led->cdev_ledb.brightness_get = ktd20xx_get_ledb_brightness;
	led->cdev_ledb.blink_set = ktd20xx_set_led_blink_time;

	ret = led_classdev_register(&led->client->dev, &led->cdev_ledb);
	if (ret < 0) {
		dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledb.name);
		goto failed_unregister_led1_B;
	}
	ret = sysfs_create_group(&led->cdev_ledb.dev->kobj, &blink_attr_group);
	if (ret)
		goto failed_unregister_led1_B;

	INIT_DELAYED_WORK(&led->work_ledb, ktd20xx_switch_ledb_blink_work);

	if (led->pdata[BUTTON].led_number) {
		led->cdev_ledbutton.name = "button-backlight";
		led->cdev_ledbutton.brightness =  led->pdata[BUTTON].led_current;
		led->cdev_ledbutton.max_brightness = LED_FULL;
		led->cdev_ledbutton.blink_brightness = LED_HALF;
		led->cdev_ledbutton.brightness_set = ktd20xx_set_ledbutton_brightness;
		led->cdev_ledbutton.brightness_get = ktd20xx_get_ledbutton_brightness;

		ret = led_classdev_register(&led->client->dev, &led->cdev_ledbutton);
		if (ret < 0) {
			dev_err(&led->client->dev, "couldn't register LED %s\n",
							led->cdev_ledbutton.name);
			goto failed_unregister_led1_B;
		}
	}


	return 0;

failed_unregister_led1_B:
	led_classdev_unregister(&led->cdev_ledg);
failed_unregister_led1_G:
	led_classdev_unregister(&led->cdev_ledr);
failed_unregister_led1_R:

	return ret;
}

static void ktd20xx_unregister_led_classdev(struct ktd20xx_led *led)
{
	led_classdev_unregister(&led->cdev_ledb);
	led_classdev_unregister(&led->cdev_ledg);
	led_classdev_unregister(&led->cdev_ledr);
	if (led->pdata[BUTTON].led_number)
		led_classdev_unregister(&led->cdev_ledbutton);
	cancel_delayed_work(&led->work_ledr);
	cancel_delayed_work(&led->work_ledg);
	cancel_delayed_work(&led->work_ledb);
}

static int ktd20xx_led_parse_dt_platform(struct device_node *np,
				const char *prop_name,
				struct ktd20xx_led_platform_data *pdata)
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
	pdata->led_number = tmp[7];

	return 0;
}

static int ktd20xx_led_parse_dt(struct device *dev,
				struct ktd20xx_led *led)
{
	struct device_node *np = dev->of_node;
	int ret;
	int rc;
	int led_ctrl;
	led_ctrl = of_get_named_gpio(np, "qcom,led-ctrl-gpio", 0);
	if(!(led_ctrl < 0))
	{
		ret  = gpio_request(led_ctrl , "LED_CTRL");
		if(!ret)
		{
			ret = gpio_direction_output(led_ctrl , 1);
		}
	}

	rc = ktd20xx_led_parse_dt_platform(np,"ktd,ledr-parameter_array",&led->pdata[RED]);
	if (rc){
		pr_err("%s:%d, error reading RED light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = ktd20xx_led_parse_dt_platform(np,"ktd,ledg-parameter_array",&led->pdata[GREEN]);
	if (rc){
		pr_err("%s:%d, error reading GREEN light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = ktd20xx_led_parse_dt_platform(np,"ktd,ledb-parameter_array",&led->pdata[BLUE]);
	if (rc){
		pr_err("%s:%d, error reading BLUE light, rc = %d\n",
			__func__, __LINE__, rc);
		return -ENODEV;
	}

	rc = ktd20xx_led_parse_dt_platform(np,"ktd,button-parameter_array",&led->pdata[BUTTON]);
	if (rc){
		pr_info("%s:%d, no button backlight, continue\n",
			__func__, __LINE__);
	}

	return 0;
}

static int ktd20xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ktd20xx_led *led;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}
	
	led = devm_kzalloc(&client->dev, sizeof(struct ktd20xx_led), GFP_KERNEL);
	if (!led) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		memset(&led->pdata, 0 , sizeof(led->pdata));
		ret = ktd20xx_led_parse_dt(&client->dev, led);
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

	ktd20xx_power_on(led, 1);

	ret = i2c_smbus_write_byte_data(led->client , 0x06, 0x00);//set current is 0.125mA
	if (ret < 0)
		goto err_exit;
	ret = i2c_smbus_write_byte_data(led->client , 0x04, 0x00);//turn off leds
	if(ret < 0){
		goto err_exit;
	}
	else{
		/* register class dev */
		ret = ktd20xx_register_led_classdev(led);
		
		if (ret < 0)
			goto err_exit;
	}

	return 0;

err_exit:
	devm_kfree(&client->dev, led);

	return ret;
}

static int ktd20xx_remove(struct i2c_client *client)
{
	struct ktd20xx_led *led = i2c_get_clientdata(client);

	ktd20xx_unregister_led_classdev(led);
	devm_kfree(&client->dev, led);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int ktd20xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ktd20xx_led *led = i2c_get_clientdata(client);

	if(debug)
 	{
 		printk(KERN_ERR "%s %d\n", __func__, __LINE__);
	}

	if((led->state_ledr != KTD_OFF)
		|| (led->state_ledg != KTD_OFF)
		|| (led->state_ledb != KTD_OFF) )
		return 0;

	ktd20xx_enable(led, 0);

	return 0;
}

static int ktd20xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ktd20xx_led *led = i2c_get_clientdata(client);
	if(debug)
 	{
 		printk(KERN_ERR "%s %d\n", __func__, __LINE__);
	}
	ktd20xx_enable(led, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ktd20xx_pm, ktd20xx_suspend, ktd20xx_resume);

static const struct i2c_device_id ktd20xx_id[] = {
	{KTD_I2C_NAME, 0},
	{ }
};

// #ifdef CONFIG_OF
static struct of_device_id bd_match_table[] = {
        { .compatible = "ktd,ktd2026",},
		{ },
};

MODULE_DEVICE_TABLE(i2c, ktd20xx_id);

static struct i2c_driver ktd20xx_i2c_driver = {
	.driver	= {
		.name	= KTD_I2C_NAME,
		.pm	= &ktd20xx_pm,
		.of_match_table = bd_match_table,
	},
	.probe		= ktd20xx_probe,
	.remove		= ktd20xx_remove,
	.id_table	= ktd20xx_id,
};

static int ktd20xx_driver_init(void)
{
	return i2c_add_driver(&ktd20xx_i2c_driver);
};

static void ktd20xx_driver_exit(void)
{
	i2c_del_driver(&ktd20xx_i2c_driver);
}

module_init(ktd20xx_driver_init);
module_exit(ktd20xx_driver_exit);

//module_i2c_driver(ktd20xx_i2c_driver);

MODULE_AUTHOR("Kim Kyuwon <q1.kim@samsung.com>");
MODULE_DESCRIPTION("KTD LED driver");
MODULE_LICENSE("GPL v2");
