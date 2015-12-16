/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_camera_io_util.h"
#include "msm_led_flash.h"

#define FLASH_NAME "silergy-flash,sy7802"

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sy7802_i2c_driver;

static struct msm_camera_i2c_reg_array sy7802_init_array[] = {
	{0xE0, 0x6C},//configuration, bit2 FLEN
};

static struct msm_camera_i2c_reg_array sy7802_off_array[] = {
	{0x10, 0x18},
};

static struct msm_camera_i2c_reg_array sy7802_release_array[] = {
	{0xE0, 0x68},//configuration, close bit2 FLEN
};

static struct msm_camera_i2c_reg_array sy7802_low_array[] = {
        {0xA0, 0x2C},//Modify lizhi 2d
	{0x10, 0x1A},//torch
};

static struct msm_camera_i2c_reg_array sy7802_high_array[] = {
	{0x10, 0x1B},//flash
};


static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	CDBG("%s: enter\n", __func__);
	msm_flash_led_init(&fctrl);
	if (value > LED_OFF) {
	    CDBG("%s: torch\n", __func__);
		if(fctrl.func_tbl->flash_led_low)
			fctrl.func_tbl->flash_led_low(&fctrl);
	} else {
	    CDBG("%s: close torch\n", __func__);
		if(fctrl.func_tbl->flash_led_off)
			fctrl.func_tbl->flash_led_off(&fctrl);
	}
};


static struct led_classdev msm_torch_led = {
	.name			= "flashlight",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
	.max_brightness = LED_FULL,
};


static void msm_sy7802_unregister_torch_classdev(void)
{
	led_classdev_unregister(&msm_torch_led);
}


static int msm_flash_sy7802_i2c_remove(struct i2c_client *client)
{
	CDBG("%s: enter\n", __func__);
	msm_sy7802_unregister_torch_classdev();
	i2c_del_driver(&sy7802_i2c_driver);
	return 0;
}

static const struct of_device_id sy7802_trigger_dt_match[] = {
	{.compatible = FLASH_NAME, .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(i2c, sy7802_trigger_dt_match);

static const struct i2c_device_id sy7802_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int32_t msm_sy7802_register_torch_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};

static int msm_flash_sy7802_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0 ;

	if (!id) {
		pr_err("msm_flash_sy7802_i2c_probe: id is NULL");
		id = sy7802_i2c_id;
	}
    CDBG("%s: enter\n", __func__);
	rc = msm_flash_i2c_probe(client, id);

	if (!rc)
		msm_sy7802_register_torch_classdev(&(client->dev),NULL);
	else
		pr_err("msm_flash_sy7802_i2c_probe failed");
	return rc;
}

static struct i2c_driver sy7802_i2c_driver = {
	.id_table = sy7802_i2c_id,
	.probe  = msm_flash_sy7802_i2c_probe,
	.remove = msm_flash_sy7802_i2c_remove,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sy7802_trigger_dt_match,
	},
};

static int __init msm_flash_sy7802_init_module(void)
{
	CDBG("%s: enter\n", __func__);
	return i2c_add_driver(&sy7802_i2c_driver);
}

static void __exit msm_flash_sy7802_exit_module(void)
{
	i2c_del_driver(&sy7802_i2c_driver);
}

static struct msm_camera_i2c_client sy7802_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sy7802_init_setting = {
	.reg_setting = sy7802_init_array,
	.size = ARRAY_SIZE(sy7802_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7802_off_setting = {
	.reg_setting = sy7802_off_array,
	.size = ARRAY_SIZE(sy7802_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7802_release_setting = {
	.reg_setting = sy7802_release_array,
	.size = ARRAY_SIZE(sy7802_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7802_low_setting = {
	.reg_setting = sy7802_low_array,
	.size = ARRAY_SIZE(sy7802_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sy7802_high_setting = {
	.reg_setting = sy7802_high_array,
	.size = ARRAY_SIZE(sy7802_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t sy7802_regs = {
	.init_setting = &sy7802_init_setting,
	.off_setting = &sy7802_off_setting,
	.low_setting = &sy7802_low_setting,
	.high_setting = &sy7802_high_setting,
	.release_setting = &sy7802_release_setting,
};

static struct msm_flash_fn_t sy7802_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sy7802_i2c_client,
	.reg_setting = &sy7802_regs,
	.func_tbl = &sy7802_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_sy7802_init_module);
module_exit(msm_flash_sy7802_exit_module);
MODULE_DESCRIPTION("sy7802 FLASH");
MODULE_LICENSE("GPL v2");
