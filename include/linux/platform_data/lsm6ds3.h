/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 */

#ifndef __LSM6DS3_H__
#define __LSM6DS3_H__

#define LSM6DS3_ACC_GYR_DEV_NAME		"lsm6ds3"
#define LSM6DS3_ACC_INPUT_DEV_NAME	"lsm6ds3_acc"
#define LSM6DS3_GYR_INPUT_DEV_NAME	"lsm6ds3_gyr"

struct lsm6ds3_platform_data {
	u8 drdy_int_pin;
};

#endif /* __LSM6DS3_H__ */
