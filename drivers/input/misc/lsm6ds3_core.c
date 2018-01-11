/*
 * STMicroelectronics lsm6ds3 driver
 *
 * Copyright 2014 STMicroelectronics Inc.
 *
 * Giuseppe Barba <giuseppe.barba@st.com>
 * v 1.1.0
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include	<linux/platform_data/lsm6ds3.h>
#include	"lsm6ds3_core.h"

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#endif
/* COMMON VALUES FOR ACCEL-GYRO SENSORS */
#define LSM6DS3_WHO_AM_I			0x0f
#define LSM6DS3_WHO_AM_I_DEF			0x69
#define LSM6DS3_AXIS_EN_MASK			0x38
#define LSM6DS3_INT1_CTRL_ADDR			0x0d
#define LSM6DS3_INT2_CTRL_ADDR			0x0e
#define LSM6DS3_INT1_FULL			0x20
#define LSM6DS3_INT1_FTH			0x08
#define LSM6DS3_MD1_ADDR			0x5e
#define LSM6DS3_ODR_LIST_NUM			7
#define LSM6DS3_ODR_POWER_OFF_VAL		0x00
#define LSM6DS3_ODR_13HZ_VAL            0x01
#define LSM6DS3_ODR_26HZ_VAL			0x02
#define LSM6DS3_ODR_52HZ_VAL			0x03
#define LSM6DS3_ODR_104HZ_VAL			0x04
#define LSM6DS3_ODR_208HZ_VAL			0x05
#define LSM6DS3_ODR_416HZ_VAL			0x06
#define LSM6DS3_ODR_833HZ_VAL           0x07
#define LSM6DS3_ODR_1660HZ_VAL          0x08
#define LSM6DS3_FS_LIST_NUM			4
#define LSM6DS3_BDU_ADDR			0x12
#define LSM6DS3_BDU_MASK			0x40
#define LSM6DS3_EN_BIT				0x01
#define LSM6DS3_DIS_BIT				0x00
#define LSM6DS3_FUNC_EN_ADDR			0x19
#define LSM6DS3_FUNC_EN_MASK			0x04
#define LSM6DS3_FUNC_CFG_ACCESS_ADDR		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK		0x01
#define LSM6DS3_FUNC_CFG_ACCESS_MASK2		0x04
#define LSM6DS3_FUNC_CFG_REG2_MASK		0x80
#define LSM6DS3_FUNC_CFG_START1_ADDR		0x62
#define LSM6DS3_FUNC_CFG_START2_ADDR		0x63
#define LSM6DS3_SELFTEST_ADDR			0x14
#define LSM6DS3_SELFTEST_ACCEL_MASK		0x03
#define LSM6DS3_SELFTEST_GYRO_MASK		0x0c
#define LSM6DS3_SELF_TEST_DISABLED_VAL		0x00
#define LSM6DS3_SELF_TEST_POS_SIGN_VAL		0x01
#define LSM6DS3_SELF_TEST_NEG_ACCEL_SIGN_VAL	0x02
#define LSM6DS3_SELF_TEST_NEG_GYRO_SIGN_VAL	0x03
#define LSM6DS3_LIR_ADDR			0x58
#define LSM6DS3_LIR_MASK			0x01
#define LSM6DS3_TIMER_EN_ADDR			0x58
#define LSM6DS3_TIMER_EN_MASK			0x80
#define LSM6DS3_PEDOMETER_EN_ADDR		0x58
#define LSM6DS3_PEDOMETER_EN_MASK		0x40
#define LSM6DS3_INT2_ON_INT1_ADDR		0x13
#define LSM6DS3_INT2_ON_INT1_MASK		0x20
#define LSM6DS3_MIN_DURATION_MS			1638
#define LSM6DS3_ROUNDING_ADDR			0x16
#define LSM6DS3_ROUNDING_MASK			0x04
#define LSM6DS3_FIFO_MODE_ADDR			0x0a
#define LSM6DS3_FIFO_MODE_MASK			0x07
#define LSM6DS3_FIFO_MODE_BYPASS		0x00
#define LSM6DS3_FIFO_MODE_CONTINUOS		0x06
#define LSM6DS3_FIFO_THRESHOLD_IRQ_MASK		0x08
#define LSM6DS3_FIFO_ODR_ADDR			0x0a
#define LSM6DS3_FIFO_ODR_MASK			0x78
#define LSM6DS3_FIFO_ODR_MAX			0x07
#define LSM6DS3_FIFO_ODR_MAX_HZ			800
#define LSM6DS3_FIFO_ODR_OFF			0x00
#define LSM6DS3_FIFO_CTRL3_ADDR			0x08
#define LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK	0x07
#define LSM6DS3_FIFO_GYRO_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_CTRL4_ADDR			0x09
#define LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK	0x38
#define LSM6DS3_FIFO_THR_L_ADDR			0x06
#define LSM6DS3_FIFO_THR_H_ADDR			0x07
#define LSM6DS3_FIFO_THR_H_MASK			0x0f
#define LSM6DS3_FIFO_THR_IRQ_MASK		0x08
#define LSM6DS3_FIFO_PEDO_E_ADDR		0x07
#define LSM6DS3_FIFO_PEDO_E_MASK		0x80
#define LSM6DS3_FIFO_STEP_C_FREQ		25

/* CUSTOM VALUES FOR ACCEL SENSOR */
#define LSM6DS3_ACCEL_ODR_ADDR			0x10
#define LSM6DS3_ACCEL_ODR_MASK			0xf0
#define LSM6DS3_ACCEL_FS_ADDR			0x10
#define LSM6DS3_ACCEL_FS_MASK			0x0c
#define LSM6DS3_ACCEL_FS_2G_VAL			0x00
#define LSM6DS3_ACCEL_FS_4G_VAL			0x02
#define LSM6DS3_ACCEL_FS_8G_VAL			0x03
#define LSM6DS3_ACCEL_FS_16G_VAL		0x01
#define LSM6DS3_ACCEL_FS_2G_GAIN		610
#define LSM6DS3_ACCEL_FS_4G_GAIN		1220
#define LSM6DS3_ACCEL_FS_8G_GAIN		2440
#define LSM6DS3_ACCEL_FS_16G_GAIN		4880
#define LSM6DS3_ACCEL_OUT_X_L_ADDR		0x28
#define LSM6DS3_ACCEL_OUT_Y_L_ADDR		0x2a
#define LSM6DS3_ACCEL_OUT_Z_L_ADDR		0x2c
#define LSM6DS3_ACCEL_AXIS_EN_ADDR		0x18
#define LSM6DS3_ACCEL_DRDY_IRQ_MASK		0x01
#define LSM6DS3_ACCEL_STD			1
#define LSM6DS3_ACCEL_STD_FROM_PD		2

/* CUSTOM VALUES FOR GYRO SENSOR */
#define LSM6DS3_GYRO_ODR_ADDR			0x11
#define LSM6DS3_GYRO_ODR_MASK			0xf0
#define LSM6DS3_GYRO_FS_ADDR			0x11
#define LSM6DS3_GYRO_FS_MASK			0x0c
#define LSM6DS3_GYRO_FS_245_VAL			0x00
#define LSM6DS3_GYRO_FS_500_VAL			0x01
#define LSM6DS3_GYRO_FS_1000_VAL		0x02
#define LSM6DS3_GYRO_FS_2000_VAL		0x03
#define LSM6DS3_GYRO_FS_245_GAIN		8750
#define LSM6DS3_GYRO_FS_500_GAIN		17500
#define LSM6DS3_GYRO_FS_1000_GAIN		35000
#define LSM6DS3_GYRO_FS_2000_GAIN		70000
#define LSM6DS3_GYRO_OUT_X_L_ADDR		0x22
#define LSM6DS3_GYRO_OUT_Y_L_ADDR		0x24
#define LSM6DS3_GYRO_OUT_Z_L_ADDR		0x26
#define LSM6DS3_GYRO_AXIS_EN_ADDR		0x19
#define LSM6DS3_GYRO_DRDY_IRQ_MASK		0x02
#define LSM6DS3_GYRO_STD			6
#define LSM6DS3_GYRO_STD_FROM_PD		2

#define LSM6DS3_OUT_XYZ_SIZE			8

/* CUSTOM VALUES FOR SIGNIFICANT MOTION SENSOR */
#define LSM6DS3_SIGN_MOTION_EN_ADDR		0x19
#define LSM6DS3_SIGN_MOTION_EN_MASK		0x01
#define LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK	0x40

/* CUSTOM VALUES FOR STEP DETECTOR SENSOR */
#define LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK	0x80

/* CUSTOM VALUES FOR STEP COUNTER SENSOR */
#define LSM6DS3_STEP_COUNTER_DRDY_IRQ_MASK	0x80
#define LSM6DS3_STEP_COUNTER_OUT_L_ADDR		0x4b
#define LSM6DS3_STEP_COUNTER_OUT_SIZE		2
#define LSM6DS3_STEP_COUNTER_RES_ADDR		0x19
#define LSM6DS3_STEP_COUNTER_RES_MASK		0x06
#define LSM6DS3_STEP_COUNTER_RES_ALL_EN		0x03
#define LSM6DS3_STEP_COUNTER_RES_FUNC_EN	0x02
#define LSM6DS3_STEP_COUNTER_DURATION_ADDR	0x15

/* CUSTOM VALUES FOR TILT SENSOR */
#define LSM6DS3_TILT_EN_ADDR			0x58
#define LSM6DS3_TILT_EN_MASK			0x20
#define LSM6DS3_TILT_DRDY_IRQ_MASK		0x02

#define LSM6DS3_ENABLE_AXIS			0x07
#define LSM6DS3_FIFO_DIFF_L			0x3a
#define LSM6DS3_FIFO_DIFF_MASK			0x0fff
#define LSM6DS3_FIFO_DATA_OUT_L			0x3e
#define LSM6DS3_FIFO_ELEMENT_LEN_BYTE		6
#define LSM6DS3_FIFO_BYTE_FOR_CHANNEL		2
#define LSM6DS3_FIFO_DATA_OVR_2REGS		0x4000
#define LSM6DS3_FIFO_DATA_OVR			0x40

#define LSM6DS3_SRC_FUNC_ADDR			0x53
#define LSM6DS3_FIFO_DATA_AVL_ADDR		0x3b

#define LSM6DS3_SRC_SIGN_MOTION_DATA_AVL	0x40
#define LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL	0x10
#define LSM6DS3_SRC_TILT_DATA_AVL		0x20
#define LSM6DS3_SRC_STEP_COUNTER_DATA_AVL	0x80
#define LSM6DS3_FIFO_DATA_AVL			0x80
#define LSM6DS3_RESET_ADDR			0x12
#define LSM6DS3_RESET_MASK			0x01
#define LSM6DS3_MAX_FIFO_SIZE			(8 * 1024)
#define LSM6DS3_MAX_FIFO_LENGHT			(LSM6DS3_MAX_FIFO_SIZE / \
						LSM6DS3_FIFO_ELEMENT_LEN_BYTE)
#define ACC_GYRO_MAX_POS			(28672000)/** max positive value acc [ug] */
#define ACC_GYRO_MAX_NEG			(28672001)/** max negative value acc [ug] */
#define FUZZ				(0)
#define FLAT				(0)
#define CAL_REPT (30)
#define GRAVITY_HAL_UNIT       9800000
#define ABS(x)		((x) < 0 ? (-x) : (x))




#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)				(((a) < (b)) ? (a) : (b))
#endif

static struct sensors_classdev lsm6ds3_acc_classdev = {
    .name = "lsm6ds3_accelerometer",
	.vendor = "st",
	.version = 1,
	.handle = SENSORS_ACCELERATION_HANDLE,
	.type = SENSOR_TYPE_ACCELEROMETER,
	.max_range = "19.6",
	.resolution = "0.00061",
	.sensor_power = "4",
	.min_delay = 5000,//us
	.max_delay = 1000,//ms
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev lsm6ds3_gyr_classdev = {
    .name = "lsm6ds3_gyroscope",
	.vendor = "st",
	.version = 1,
	.handle = SENSORS_GYROSCOPE_HANDLE,
	.type = SENSOR_TYPE_GYROSCOPE,
	.max_range = "8.73",
	.resolution = "0.000305",
	.sensor_power = "4",
	.min_delay = 5000,//us
	.max_delay = 1000,//ms
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev lsm6ds3_step_counter_classdev = {
    .name = "lsm6ds3_step_counter",
	.vendor = "st",
	.version = 1,
	.handle = SENSOR_STEP_COUNTER_HANDLE,
	.type = SENSOR_TYPE_STEP_COUNTER,
	.max_range = "65535",
	.resolution = "1",
	.sensor_power = "4",
	.min_delay = 2000,//us
	.max_delay = 1000,//ms
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev lsm6ds3_step_detector_classdev = {
    .name = "lsm6ds3_step_detector",
	.vendor = "st",
	.version = 1,
	.handle = SENSOR_STEP_DETECTOR_HANDLE,
	.type = SENSOR_TYPE_STEP_DETECTOR,
	.max_range = "1",
	.resolution = "1",
	.sensor_power = "4",
	.min_delay = 2000,//us
	.max_delay = 1000,//ms
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


static const struct lsm6ds3_sensor_name {
	const char *name;
	const char *description;
} lsm6ds3_sensor_name[LSM6DS3_SENSORS_NUMB] = {
	[LSM6DS3_ACCEL] = {
			.name = "accelerometer",
			.description = "accelerometer",
	},
	[LSM6DS3_GYRO] = {
			.name = "gyroscope",
			.description = "gyroscope",
	},
	[LSM6DS3_SIGN_MOTION] = {
			.name = "significant_motion",
			.description = "significant_motion",
	},
	[LSM6DS3_STEP_COUNTER] = {
			.name = "step_counter",
			.description = "step_counter",
	},
	[LSM6DS3_STEP_DETECTOR] = {
			.name = "step_detector",
			.description = "step_detector",
	},
	[LSM6DS3_TILT] = {
			.name = "tilt",
			.description = "tilt",
	},
};

struct lsm6ds3_odr_reg {
	u32 hz;
	u8 value;
};

static const struct lsm6ds3_odr_table {
	u8 addr[2];
	u8 mask[2];
	struct lsm6ds3_odr_reg odr_avl[7];
} lsm6ds3_odr_table = {
	.addr[LSM6DS3_ACCEL] = LSM6DS3_ACC_ODR_ADDR,
	.mask[LSM6DS3_ACCEL] = LSM6DS3_ACC_ODR_MASK,
	.addr[LSM6DS3_GYRO] = LSM6DS3_GYR_ODR_ADDR,
	.mask[LSM6DS3_GYRO] = LSM6DS3_GYR_ODR_MASK,
	.odr_avl[0] = { .hz = 26, .value = LSM6DS3_ODR_26HZ_VAL },
	.odr_avl[1] = { .hz = 52, .value = LSM6DS3_ODR_52HZ_VAL },
	.odr_avl[2] = { .hz = 104, .value = LSM6DS3_ODR_104HZ_VAL },
	.odr_avl[3] = { .hz = 208, .value = LSM6DS3_ODR_208HZ_VAL },
	.odr_avl[4] = { .hz = 416, .value = LSM6DS3_ODR_416HZ_VAL },
	.odr_avl[5] = { .hz = 833, .value = LSM6DS3_ODR_833HZ_VAL },
	.odr_avl[6] = { .hz = 1660, .value = LSM6DS3_ODR_1660HZ_VAL },
};

struct lsm6ds3_fs_reg {
	unsigned int gain;
	u8 value;
	int urv;
};

static struct lsm6ds3_fs_table {
	u8 addr;
	u8 mask;
	struct lsm6ds3_fs_reg fs_avl[LSM6DS3_FS_LIST_NUM];
} lsm6ds3_fs_table[LSM6DS3_SENSORS_NUMB] = {
	[LSM6DS3_ACCEL] = {
		.addr = LSM6DS3_ACCEL_FS_ADDR,
		.mask = LSM6DS3_ACCEL_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_ACCEL_FS_2G_GAIN,
					.value = LSM6DS3_ACCEL_FS_2G_VAL,
					.urv = 2, },
		.fs_avl[1] = { .gain = LSM6DS3_ACCEL_FS_4G_GAIN,
					.value = LSM6DS3_ACCEL_FS_4G_VAL,
					.urv = 4, },
		.fs_avl[2] = { .gain = LSM6DS3_ACCEL_FS_8G_GAIN,
					.value = LSM6DS3_ACCEL_FS_8G_VAL,
					.urv = 8, },
		.fs_avl[3] = { .gain = LSM6DS3_ACCEL_FS_16G_GAIN,
					.value = LSM6DS3_ACCEL_FS_16G_VAL,
					.urv = 16, },
	},
	[LSM6DS3_GYRO] = {
		.addr = LSM6DS3_GYRO_FS_ADDR,
		.mask = LSM6DS3_GYRO_FS_MASK,
		.fs_avl[0] = { .gain = LSM6DS3_GYRO_FS_245_GAIN,
					.value = LSM6DS3_GYRO_FS_245_VAL,
					.urv = 245, },
		.fs_avl[1] = { .gain = LSM6DS3_GYRO_FS_500_GAIN,
					.value = LSM6DS3_GYRO_FS_500_VAL,
					.urv = 500, },
		.fs_avl[2] = { .gain = LSM6DS3_GYRO_FS_1000_GAIN,
					.value = LSM6DS3_GYRO_FS_1000_VAL,
					.urv = 1000, },
		.fs_avl[3] = { .gain = LSM6DS3_GYRO_FS_2000_GAIN,
					.value = LSM6DS3_GYRO_FS_2000_VAL,
					.urv = 2000, },
	}
};

static struct workqueue_struct *lsm6ds3_workqueue = 0;

static inline void lsm6ds3_flush_works(void)
{
	flush_workqueue(lsm6ds3_workqueue);
}

static inline s64 lsm6ds3_get_time_ns(void)
{
	struct timespec ts;
	/*
	 * calls getnstimeofday.
	 * If hrtimers then up to ns accurate, if not microsecond.
	 */
	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

static int lsm6ds3_write_data_with_mask(struct lsm6ds3_data *cdata,
				u8 reg_addr, u8 mask, u8 data, bool b_lock)
{
	int err;
	u8 new_data = 0x00, old_data = 0x00;

	err = cdata->tf->read(cdata, reg_addr, 1, &old_data, b_lock);
	if (err < 0)
		return err;

	new_data = ((old_data & (~mask)) | ((data << __ffs(mask)) & mask));

	if (new_data == old_data)
		return 1;

	return cdata->tf->write(cdata, reg_addr, 1, &new_data, b_lock);
}

static int lsm6ds3_input_init(struct lsm6ds3_sensor_data *sdata, u16 bustype,
							const char *description)
{
	int err = 0;

	sdata->input_dev = input_allocate_device();
	if (!sdata->input_dev) {
		dev_err(sdata->cdata->dev, "failed to allocate input device");
		return -ENOMEM;
	}

	sdata->input_dev->id.bustype = bustype;
	sdata->input_dev->dev.parent = sdata->cdata->dev;
	sdata->input_dev->name = description;
	input_set_drvdata(sdata->input_dev, sdata);

	set_bit(EV_ABS, sdata->input_dev->evbit);

	if ((sdata->sindex == LSM6DS3_ACCEL) || (sdata->sindex == LSM6DS3_GYRO)) {
		input_set_abs_params(sdata->input_dev, ABS_X, -ACC_GYRO_MAX_NEG,
						ACC_GYRO_MAX_POS, FUZZ, FLAT);
		input_set_abs_params(sdata->input_dev, ABS_Y, -ACC_GYRO_MAX_NEG,
						ACC_GYRO_MAX_POS, FUZZ, FLAT);
		input_set_abs_params(sdata->input_dev, ABS_Z,-ACC_GYRO_MAX_NEG,
						ACC_GYRO_MAX_POS, FUZZ, FLAT);
	}

	if ((sdata->sindex == LSM6DS3_STEP_COUNTER) ||
		(sdata->sindex ==LSM6DS3_STEP_DETECTOR)) {
		input_set_abs_params(sdata->input_dev, ABS_MISC, -ACC_GYRO_MAX_NEG,
						ACC_GYRO_MAX_POS, FUZZ, FLAT);
	}

	err = input_register_device(sdata->input_dev);
	if (err) {
		dev_err(sdata->cdata->dev, "unable to register sensor %s\n",
								sdata->name);
		input_free_device(sdata->input_dev);
	}

	return err;
}

static void lsm6ds3_input_cleanup(struct lsm6ds3_sensor_data *sdata)
{
	input_unregister_device(sdata->input_dev);
	input_free_device(sdata->input_dev);
}
#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
static void lsm6ds3_report_3axes_event(struct lsm6ds3_sensor_data *sdata, s32 *xyz){
	struct timespec ts;
	struct input_dev  * input  	= sdata->input_dev;

	if (unlikely(!sdata->enabled))
		return;

	input_report_abs(input, ABS_X, xyz[0]);
	input_report_abs(input, ABS_Y, xyz[1]);
	input_report_abs(input, ABS_Z, xyz[2]);
	get_monotonic_boottime(&ts);
	input_event(input, EV_SYN, SYN_TIME_SEC, ts.tv_sec);
	input_event(input, EV_SYN, SYN_TIME_NSEC, ts.tv_nsec);
	input_sync(input);
}
#endif
static void lsm6ds3_report_single_event(struct lsm6ds3_sensor_data *sdata, s32 data){
	struct timespec ts;
	struct input_dev  * input  	= sdata->input_dev;

	if (unlikely(!sdata->enabled))
		return;

	input_report_abs(input, ABS_MISC, data);
	get_monotonic_boottime(&ts);
	input_event(input, EV_SYN, SYN_TIME_SEC, ts.tv_sec);
	input_event(input, EV_SYN, SYN_TIME_NSEC, ts.tv_nsec);
	input_sync(input);
}

#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
static void lsm6ds3_push_data_with_timestamp(struct lsm6ds3_sensor_data *sdata,
							u16 offset)
{
	s32 data[3];
	s32 tmp;
	data[0] = (s32)((s16)(sdata->cdata->fifo_data_buffer[offset] |
			(sdata->cdata->fifo_data_buffer[offset + 1] << 8)));
	data[1] = (s32)((s16)(sdata->cdata->fifo_data_buffer[offset + 2] |
			(sdata->cdata->fifo_data_buffer[offset + 3] << 8)));
	data[2] = (s32)((s16)(sdata->cdata->fifo_data_buffer[offset + 4] |
			(sdata->cdata->fifo_data_buffer[offset + 5] << 8)));

	data[0] *= sdata->c_gain;
	data[1] *= sdata->c_gain;
	data[2] *= sdata->c_gain;
	tmp = data[0];
	data[0] = data[1];
	data[1] = tmp;
	data[0] += sdata->cali_data[0];
	data[1] += sdata->cali_data[1];
	data[2] += sdata->cali_data[2];
	lsm6ds3_report_3axes_event(sdata, data);
}
#else
enum hrtimer_restart lsm6ds3_poll_function_read(struct hrtimer *timer)
{
	struct lsm6ds3_sensor_data *sdata = container_of((struct hrtimer *)timer, 
							struct lsm6ds3_sensor_data,	hr_timer);

	queue_work(lsm6ds3_workqueue, &sdata->input_work);
	
//	sdata->timestamp += MS_TO_NS(sdata->poll_interval);
//	hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
	hrtimer_forward_now(&sdata->hr_timer, sdata->ktime);

	return HRTIMER_RESTART;
}

static int lsm6ds3_get_step_c_data(struct lsm6ds3_sensor_data *sdata, u16 *steps)
{
	u8 data[2];
	if (sdata->cdata->tf->read(sdata->cdata,
					LSM6DS3_STEP_COUNTER_OUT_L_ADDR,
					LSM6DS3_STEP_COUNTER_OUT_SIZE,
					data, true) < 0)
		return -1;

	*steps = data[0] | (data[1] << 8);

	return 0;
}

static int lsm6ds3_get_poll_data(struct lsm6ds3_sensor_data *sdata, u8 *data)
{
	u8 reg_addr;

	switch(sdata->sindex) {
	case LSM6DS3_ACCEL:
		reg_addr = LSM6DS3_ACCEL_OUT_X_L_ADDR;

		break;
	case LSM6DS3_GYRO:
		reg_addr = LSM6DS3_GYRO_OUT_X_L_ADDR;

		break;
	default:
		dev_err(sdata->cdata->dev, "invalid polling mode for sensor %s\n",
								sdata->name);
		return -1;
	}

	return sdata->cdata->tf->read(sdata->cdata, reg_addr, LSM6DS3_OUT_XYZ_SIZE,
								data, true);
}

static int lsm6ds3_cal_poll_data(struct lsm6ds3_sensor_data* sdata, int* output){
	u8 data[6];
	int tmp;
	if(unlikely(lsm6ds3_get_poll_data(sdata, data) < 0)) {
		dev_err(sdata->cdata->dev, "get %s data failed\n",
								sdata->name);
		return -1;
	} else {
		output[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		output[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		output[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		output[0] *= sdata->c_gain;
		output[1] *= sdata->c_gain;
		output[2] *= sdata->c_gain;
		tmp = output[0];
		output[0] = output[1];
		output[1] = tmp;
	}

	return 0;
	
}

static void poll_function_work(struct work_struct *input_work)
{
	int xyz[3] = { 0 };
	u8 data[6];
	int tmp;
	struct timespec ts;
	struct lsm6ds3_sensor_data *sdata = container_of((struct work_struct *)input_work,
			struct lsm6ds3_sensor_data, input_work);

	if (likely(lsm6ds3_get_poll_data(sdata, data) > 0)) {
		xyz[0] = (s32)((s16)(data[0] | (data[1] << 8)));
		xyz[1] = (s32)((s16)(data[2] | (data[3] << 8)));
		xyz[2] = (s32)((s16)(data[4] | (data[5] << 8)));
		xyz[0] *= sdata->c_gain;
		xyz[1] *= sdata->c_gain;
		xyz[2] *= sdata->c_gain;
		//swap x and y
		tmp = xyz[0];
		xyz[0] = xyz[1];
		xyz[1] = tmp;
		//add calibration data
		xyz[0] += sdata->cali_data[0];
		xyz[1] += sdata->cali_data[1];
		xyz[2] += sdata->cali_data[2];

		input_report_abs(sdata->input_dev, ABS_X, xyz[0]);
		input_report_abs(sdata->input_dev, ABS_Y, xyz[1]);
		input_report_abs(sdata->input_dev, ABS_Z, xyz[2]);
		get_monotonic_boottime(&ts);
		input_event(sdata->input_dev, EV_SYN, SYN_TIME_SEC, ts.tv_sec);
		input_event(sdata->input_dev, EV_SYN, SYN_TIME_NSEC, ts.tv_nsec);
		input_sync(sdata->input_dev);
	}
}
#endif

#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
static void lsm6ds3_parse_fifo_data(struct lsm6ds3_data *cdata, u16 read_len)
{
	u16 fifo_offset = 0, steps_c = 0;
	u8 gyro_sip, accel_sip, step_c_sip;

	while (fifo_offset < read_len) {
		gyro_sip = cdata->sensors[LSM6DS3_GYRO].sample_in_pattern;
		accel_sip = cdata->sensors[LSM6DS3_ACCEL].sample_in_pattern;
		step_c_sip =
			cdata->sensors[LSM6DS3_STEP_COUNTER].sample_in_pattern;

		do {
			if (gyro_sip > 0) {
				if (cdata->sensors[LSM6DS3_GYRO].sample_to_discard > 0)
					cdata->sensors[LSM6DS3_GYRO].sample_to_discard--;
				else
					lsm6ds3_push_data_with_timestamp(
						&cdata->sensors[LSM6DS3_GYRO],
						fifo_offset);

				cdata->sensors[LSM6DS3_GYRO].timestamp +=
					cdata->sensors[LSM6DS3_GYRO].deltatime;
				fifo_offset += LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				gyro_sip--;
			}

			if (accel_sip > 0) {
				if (cdata->sensors[LSM6DS3_ACCEL].sample_to_discard > 0)
					cdata->sensors[LSM6DS3_ACCEL].sample_to_discard--;
				else
					lsm6ds3_push_data_with_timestamp(
						&cdata->sensors[LSM6DS3_ACCEL],
						fifo_offset);

				cdata->sensors[LSM6DS3_ACCEL].timestamp +=
					cdata->sensors[LSM6DS3_ACCEL].deltatime;
				fifo_offset += LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				accel_sip--;
			}

			if (step_c_sip > 0) {
				steps_c = cdata->fifo_data_buffer[fifo_offset + 4] |
					(cdata->fifo_data_buffer[fifo_offset + 5] << 8);
				if (cdata->steps_c != steps_c) {
					lsm6ds3_report_single_event(
						&cdata->sensors[LSM6DS3_STEP_COUNTER],
						steps_c);
					cdata->steps_c = steps_c;
				}
				cdata->sensors[LSM6DS3_STEP_COUNTER].timestamp +=
						cdata->sensors[LSM6DS3_STEP_COUNTER].deltatime;
				fifo_offset += LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
				step_c_sip--;
			}
		} while ((accel_sip > 0) || (gyro_sip > 0) || (step_c_sip > 0));
	}

	return;
}

void lsm6ds3_read_fifo(struct lsm6ds3_data *cdata, bool check_fifo_len)
{
	int err;
	u16 read_len = cdata->fifo_threshold;

	if (!cdata->fifo_data_buffer)
		return;

	if (check_fifo_len) {
		err = cdata->tf->read(cdata, LSM6DS3_FIFO_DIFF_L, 2, (u8 *)&read_len,
									true);
		if (err < 0)
			return;

		if (read_len & LSM6DS3_FIFO_DATA_OVR_2REGS) {
			dev_err(cdata->dev,
				"data fifo overrun, failed to read it.\n");

			return;
		}

		read_len &= LSM6DS3_FIFO_DIFF_MASK;
		read_len *= LSM6DS3_FIFO_BYTE_FOR_CHANNEL;

		if (read_len > cdata->fifo_threshold)
			read_len = cdata->fifo_threshold;
	}

	if (read_len == 0)
		return;

	err = cdata->tf->read(cdata, LSM6DS3_FIFO_DATA_OUT_L, read_len,
						cdata->fifo_data_buffer, true);
	if (err < 0)
		return;

	lsm6ds3_parse_fifo_data(cdata, read_len);
}

static int lsm6ds3_set_fifo_enable(struct lsm6ds3_data *cdata, bool status)
{
	int err;
	u8 reg_value;

	if (status)
		reg_value = LSM6DS3_FIFO_ODR_MAX;
	else
		reg_value = LSM6DS3_FIFO_ODR_OFF;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_ODR_ADDR,
					LSM6DS3_FIFO_ODR_MASK,
					reg_value, true);
	if (err < 0)
		return err;

	cdata->sensors[LSM6DS3_ACCEL].timestamp = lsm6ds3_get_time_ns();
	cdata->sensors[LSM6DS3_GYRO].timestamp =
					cdata->sensors[LSM6DS3_ACCEL].timestamp;
	cdata->sensors[LSM6DS3_STEP_COUNTER].timestamp =
					cdata->sensors[LSM6DS3_ACCEL].timestamp;

	return 0;
}

int lsm6ds3_set_fifo_mode(struct lsm6ds3_data *cdata, enum fifo_mode fm)
{
	int err;
	u8 reg_value;
	bool enable_fifo;

	switch (fm) {
	case BYPASS:
		reg_value = LSM6DS3_FIFO_MODE_BYPASS;
		enable_fifo = false;
		break;
	case CONTINUOS:
		reg_value = LSM6DS3_FIFO_MODE_CONTINUOS;
		enable_fifo = true;
		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_fifo_enable(cdata, enable_fifo);
	if (err < 0)
		return err;

	return lsm6ds3_write_data_with_mask(cdata, LSM6DS3_FIFO_MODE_ADDR,
				LSM6DS3_FIFO_MODE_MASK, reg_value, true);
}

int lsm6ds3_set_fifo_decimators_and_threshold(struct lsm6ds3_data *cdata)
{
	int err;
	unsigned int min_odr = 416, max_odr = 0;
	u8 decimator = 0;
	struct lsm6ds3_sensor_data *sdata_accel, *sdata_gyro, *sdata_step_c;
	u16 fifo_len = 0, fifo_threshold;
	u16 min_num_pattern, num_pattern;

	min_num_pattern = LSM6DS3_MAX_FIFO_SIZE / LSM6DS3_FIFO_ELEMENT_LEN_BYTE;
	sdata_accel = &cdata->sensors[LSM6DS3_ACCEL];
	if (sdata_accel->enabled) {
		min_odr = MIN(min_odr, sdata_accel->c_odr);
		max_odr = MAX(max_odr, sdata_accel->c_odr);
	}

	sdata_gyro = &cdata->sensors[LSM6DS3_GYRO];
	if (sdata_gyro->enabled) {
		min_odr = MIN(min_odr, sdata_gyro->c_odr);
		max_odr = MAX(max_odr, sdata_gyro->c_odr);
	}

	sdata_step_c = &cdata->sensors[LSM6DS3_STEP_COUNTER];
	if (sdata_step_c->enabled) {
		min_odr = MIN(min_odr, sdata_step_c->c_odr);
	}

	if (sdata_accel->enabled) {
		sdata_accel->sample_in_pattern = (sdata_accel->c_odr / min_odr);
		fifo_len += sdata_accel->sample_in_pattern;
		num_pattern = MAX(sdata_accel->fifo_length /
					sdata_accel->sample_in_pattern, 1);
		min_num_pattern = MIN(min_num_pattern, num_pattern);
		sdata_accel->deltatime = (1000000000ULL / sdata_accel->c_odr);
		decimator = max_odr / sdata_accel->c_odr;
	} else {
		sdata_accel->sample_in_pattern = 0;
		decimator = 0;
	}

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_CTRL3_ADDR,
					LSM6DS3_FIFO_ACCEL_DECIMATOR_MASK,
					decimator, true);
	if (err < 0)
		return err;

	if (sdata_gyro->enabled) {
		sdata_gyro->sample_in_pattern = (sdata_gyro->c_odr / min_odr);
		fifo_len += sdata_gyro->sample_in_pattern;
		num_pattern = MAX(sdata_gyro->fifo_length /
					sdata_gyro->sample_in_pattern, 1);
		min_num_pattern = MIN(min_num_pattern, num_pattern);
		sdata_gyro->deltatime = (1000000000ULL / sdata_gyro->c_odr);
		decimator = max_odr / sdata_gyro->c_odr;
	} else {
		sdata_gyro->sample_in_pattern = 0;
		decimator = 0;
	}

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_CTRL3_ADDR,
					LSM6DS3_FIFO_GYRO_DECIMATOR_MASK,
					decimator, true);
	if (err < 0)
		return err;

	if (sdata_step_c->enabled) {
		sdata_step_c->sample_in_pattern = (sdata_step_c->c_odr / min_odr);
		fifo_len += sdata_step_c->sample_in_pattern;
		num_pattern = MAX(sdata_step_c->fifo_length /
					sdata_step_c->sample_in_pattern, 1);
		min_num_pattern = MIN(min_num_pattern, num_pattern);
		sdata_step_c->deltatime = (1000000000ULL / sdata_step_c->c_odr);
		decimator = MAX(max_odr / sdata_step_c->c_odr, 1);
	} else {
		sdata_step_c->sample_in_pattern = 0;
		decimator = 0;
	}

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_CTRL4_ADDR,
					LSM6DS3_FIFO_STEP_C_DECIMATOR_MASK,
					decimator, true);
	if (err < 0)
		return err;

	fifo_len *= (min_num_pattern * LSM6DS3_FIFO_ELEMENT_LEN_BYTE);

	if (fifo_len > 0) {
		fifo_threshold = fifo_len;

		err = cdata->tf->write(cdata,
					LSM6DS3_FIFO_THR_L_ADDR,
					1,
					(u8 *)&fifo_threshold, true);
		if (err < 0)
			return err;

		err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_FIFO_THR_H_ADDR,
					LSM6DS3_FIFO_THR_H_MASK,
					*(((u8 *)&fifo_threshold) + 1), true);
		if (err < 0)
			return err;

		cdata->fifo_threshold = fifo_len;
	}
	if (cdata->fifo_data_buffer) {
		kfree(cdata->fifo_data_buffer);
		cdata->fifo_data_buffer = 0;
	}

	if (fifo_len > 0) {
		cdata->fifo_data_buffer = kmalloc(cdata->fifo_threshold, GFP_KERNEL);
		if (!cdata->fifo_data_buffer)
			return -ENOMEM;
	}

	return fifo_len;
}

int lsm6ds3_reconfigure_fifo(struct lsm6ds3_data *cdata,
						bool disable_irq_and_flush)
{
	int err, fifo_len;

	if (disable_irq_and_flush) {
		disable_irq(cdata->irq);
		lsm6ds3_flush_works();
	}

	mutex_lock(&cdata->fifo_lock);

	lsm6ds3_read_fifo(cdata, true);

	err = lsm6ds3_set_fifo_mode(cdata, BYPASS);
	if (err < 0)
		goto reconfigure_fifo_irq_restore;

	fifo_len = lsm6ds3_set_fifo_decimators_and_threshold(cdata);
	if (fifo_len < 0) {
		err = fifo_len;
		goto reconfigure_fifo_irq_restore;
	}

	if (fifo_len > 0) {
		err = lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
		if (err < 0)
			goto reconfigure_fifo_irq_restore;
	}

reconfigure_fifo_irq_restore:
	mutex_unlock(&cdata->fifo_lock);

	if (disable_irq_and_flush)
		enable_irq(cdata->irq);

	return err;
}
#endif

int lsm6ds3_set_drdy_irq(struct lsm6ds3_sensor_data *sdata, bool state)
{
	u8 reg_addr, mask, value;

	if (state)
		value = LSM6DS3_EN_BIT;
	else
		value = LSM6DS3_DIS_BIT;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
	case LSM6DS3_GYRO:
#if defined (CONFIG_LSM6DS3_POLLING_MODE)
		return 0;
#else
		if ((sdata->cdata->sensors[LSM6DS3_GYRO].enabled) ||
				(sdata->cdata->sensors[LSM6DS3_ACCEL].enabled))
			return 0;

		reg_addr = LSM6DS3_INT1_CTRL_ADDR;
		mask = LSM6DS3_FIFO_THR_IRQ_MASK;
		break;
#endif
	case LSM6DS3_STEP_COUNTER:
	case LSM6DS3_STEP_DETECTOR:
		if ((sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled) &&
				(sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled))
			return 0;

		reg_addr = LSM6DS3_INT1_CTRL_ADDR;
		mask = LSM6DS3_STEP_DETECTOR_DRDY_IRQ_MASK;
		break;
	case LSM6DS3_SIGN_MOTION:
		reg_addr = LSM6DS3_INT1_CTRL_ADDR;
		mask = LSM6DS3_SIGN_MOTION_DRDY_IRQ_MASK;
		break;
	case LSM6DS3_TILT:
		reg_addr = LSM6DS3_MD1_ADDR;
		mask = LSM6DS3_TILT_DRDY_IRQ_MASK;
		break;
	default:
		return -EINVAL;
	}

	return lsm6ds3_write_data_with_mask(sdata->cdata, reg_addr, mask, value,
									true);
}

static int lsm6ds3_set_fs(struct lsm6ds3_sensor_data *sdata, u32 gain)
{
	int err, i;

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++) {
		if (lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain == gain)
			break;
	}

	if (i == LSM6DS3_FS_LIST_NUM)
		return -EINVAL;

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_fs_table[sdata->sindex].addr,
				lsm6ds3_fs_table[sdata->sindex].mask,
				lsm6ds3_fs_table[sdata->sindex].fs_avl[i].value,
				true);
	if (err < 0)
		return err;

	sdata->c_gain = gain;

	return 0;
}

irqreturn_t lsm6ds3_save_timestamp(int irq, void *private)
{
	struct lsm6ds3_data *cdata = private;

	cdata->timestamp = lsm6ds3_get_time_ns();
	queue_work(lsm6ds3_workqueue, &cdata->input_work);

	disable_irq_nosync(irq);

	return IRQ_HANDLED;
}

static int lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata);

static void lsm6ds3_irq_management(struct work_struct *input_work)
{
	struct lsm6ds3_data *cdata;
	u8 src_value = 0x00, src_fifo = 0x00;
	struct lsm6ds3_sensor_data *sdata;
	u16 steps_c;
	int err;

	cdata = container_of((struct work_struct *)input_work,
						struct lsm6ds3_data, input_work);

	cdata->tf->read(cdata, LSM6DS3_SRC_FUNC_ADDR, 1, &src_value, true);
	cdata->tf->read(cdata, LSM6DS3_FIFO_DATA_AVL_ADDR, 1, &src_fifo, true);

#if !defined(CONFIG_LSM6DS3_POLLING_MODE)
	if (src_fifo & LSM6DS3_FIFO_DATA_AVL) {
		if (src_fifo & LSM6DS3_FIFO_DATA_OVR) {
			lsm6ds3_set_fifo_mode(cdata, BYPASS);
			lsm6ds3_set_fifo_mode(cdata, CONTINUOS);
			dev_err(cdata->dev,
				"data fifo overrun, reduce fifo size.\n");
		} else
			lsm6ds3_read_fifo(cdata, false);
	}
#else
	if (src_value & LSM6DS3_SRC_STEP_COUNTER_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DS3_STEP_COUNTER];
		sdata->timestamp = cdata->timestamp;
		err = lsm6ds3_get_step_c_data(sdata, &steps_c);
		if (err < 0) {
			dev_err(cdata->dev,
				"error while reading step counter data\n");
			enable_irq(cdata->irq);

			return;
		}

		lsm6ds3_report_single_event(&cdata->sensors[LSM6DS3_STEP_COUNTER], steps_c);
		cdata->steps_c = steps_c;
	}
#endif

	if (src_value & LSM6DS3_SRC_STEP_DETECTOR_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DS3_STEP_DETECTOR];
		sdata->timestamp = cdata->timestamp;
		lsm6ds3_report_single_event(sdata, 1);

		if (cdata->sign_motion_event_ready) {
			sdata = &cdata->sensors[LSM6DS3_SIGN_MOTION];
			sdata->timestamp = cdata->timestamp;
			lsm6ds3_report_single_event(sdata, 1);
			cdata->sign_motion_event_ready = false;
			lsm6ds3_disable_sensors(sdata);
		}
	}

	if (src_value & LSM6DS3_SRC_TILT_DATA_AVL) {
		sdata = &cdata->sensors[LSM6DS3_TILT];
		sdata->timestamp = cdata->timestamp;
		lsm6ds3_report_single_event(sdata, 1);
	}

	enable_irq(cdata->irq);
	return;
}

int lsm6ds3_interrupt_init(struct lsm6ds3_data *cdata)
{
	int err;

	INIT_WORK(&cdata->input_work, lsm6ds3_irq_management);

	err = request_threaded_irq(cdata->irq, lsm6ds3_save_timestamp, NULL,
			IRQF_TRIGGER_HIGH | IRQF_ONESHOT, cdata->name, cdata);
	if (err)
		return err;
	return 0;
}

static int lsm6ds3_set_extra_dependency(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err;

	if (!(sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled |
				sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled |
				sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled |
				sdata->cdata->sensors[LSM6DS3_TILT].enabled)) {
		if (enable) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_FUNC_EN_ADDR,
						LSM6DS3_FUNC_EN_MASK,
						LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_FUNC_EN_ADDR,
						LSM6DS3_FUNC_EN_MASK,
						LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;
		}
	}

	if (!sdata->cdata->sensors[LSM6DS3_ACCEL].enabled) {
		if (sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled && 
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled) {
			return 0;
		}
		
		if (enable) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
						lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
						lsm6ds3_odr_table.odr_avl[0].value, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
						lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
						LSM6DS3_ODR_POWER_OFF_VAL, true);
			if (err < 0)
				return err;
		}
	}

	return 0;
}

static int lsm6ds3_reset_steps(struct lsm6ds3_data *cdata)
{
	int err;
	u8 reg_value = 0x00;

	err = cdata->tf->read(cdata,
			LSM6DS3_STEP_COUNTER_RES_ADDR, 1, &reg_value, true);
	if (err < 0)
		return err;

	if (reg_value & LSM6DS3_FUNC_EN_MASK)
		reg_value = LSM6DS3_STEP_COUNTER_RES_FUNC_EN;
	else
		reg_value = LSM6DS3_DIS_BIT;

	err = lsm6ds3_write_data_with_mask(cdata,
				LSM6DS3_STEP_COUNTER_RES_ADDR,
				LSM6DS3_STEP_COUNTER_RES_MASK,
				LSM6DS3_STEP_COUNTER_RES_ALL_EN, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
				LSM6DS3_STEP_COUNTER_RES_ADDR,
				LSM6DS3_STEP_COUNTER_RES_MASK,
				reg_value, true);
	if (err < 0)
		return err;

	cdata->reset_steps = true;

	return 0;
}


static int lsm6ds3_enable_pedometer(struct lsm6ds3_sensor_data *sdata,
								bool enable)
{
	int err = 0;
	u8 value = LSM6DS3_DIS_BIT;

	if ((sdata->sindex == LSM6DS3_STEP_COUNTER) && !enable){
		err = lsm6ds3_reset_steps(sdata->cdata);
		if (err < 0)
			return err;
	}
	
	if (sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled &&
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled)
		return 0;

	if (enable)
		value = LSM6DS3_EN_BIT;

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_FIFO_PEDO_E_ADDR,
						LSM6DS3_FIFO_PEDO_E_MASK,
						value, true);
	if (err < 0)
		return err;

	return lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						value, true);
}

static int lsm6ds3_enable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err, i;

	if (sdata->enabled)
		return 0;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
	case LSM6DS3_GYRO:
		for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
			if (lsm6ds3_odr_table.odr_avl[i].hz == sdata->c_odr)
				break;
		}
		if (i == LSM6DS3_ODR_LIST_NUM)
			return -EINVAL;

		if (sdata->sindex == LSM6DS3_ACCEL)
			sdata->sample_to_discard = LSM6DS3_ACCEL_STD +
							LSM6DS3_ACCEL_STD_FROM_PD;

		sdata->cdata->sensors[LSM6DS3_GYRO].sample_to_discard =
							LSM6DS3_GYRO_STD +
							LSM6DS3_GYRO_STD_FROM_PD;

		err = lsm6ds3_write_data_with_mask(sdata->cdata,
				lsm6ds3_odr_table.addr[sdata->sindex],
				lsm6ds3_odr_table.mask[sdata->sindex],
				lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0)
			return err;

		err = lsm6ds3_set_fs(sdata, sdata->c_gain);
		if (err < 0)
			return err;

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
		hrtimer_start(&sdata->hr_timer, sdata->ktime, HRTIMER_MODE_REL);
		sdata->timestamp = lsm6ds3_get_time_ns();
#endif

		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;

		break;
	case LSM6DS3_SIGN_MOTION:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_SIGN_MOTION_EN_ADDR,
						LSM6DS3_SIGN_MOTION_EN_MASK,
						LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		if ((sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled) ||
				(sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled)) {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						LSM6DS3_DIS_BIT, true);
			if (err < 0)
				return err;

			err = lsm6ds3_write_data_with_mask(sdata->cdata,
						LSM6DS3_PEDOMETER_EN_ADDR,
						LSM6DS3_PEDOMETER_EN_MASK,
						LSM6DS3_EN_BIT, true);
			if (err < 0)
				return err;
		} else {
			err = lsm6ds3_enable_pedometer(sdata, true);
			if (err < 0)
				return err;
		}

		sdata->cdata->sign_motion_event_ready = true;

		break;
	case LSM6DS3_STEP_COUNTER:
	case LSM6DS3_STEP_DETECTOR:
		err = lsm6ds3_enable_pedometer(sdata, true);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_TILT:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_TILT_EN_ADDR,
					LSM6DS3_TILT_EN_MASK,
					LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_extra_dependency(sdata, true);
	if (err < 0)
		return err;


	err = lsm6ds3_set_drdy_irq(sdata, true);
	if (err < 0)
		return err;

	sdata->enabled = true;

#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
	err = lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;
#endif

	return 0;
}

static int lsm6ds3_disable_sensors(struct lsm6ds3_sensor_data *sdata)
{
	int err;

	if (!sdata->enabled)
		return 0;

	switch (sdata->sindex) {
	case LSM6DS3_ACCEL:
		if (sdata->cdata->sensors[LSM6DS3_SIGN_MOTION].enabled |
			sdata->cdata->sensors[LSM6DS3_STEP_COUNTER].enabled |
			sdata->cdata->sensors[LSM6DS3_STEP_DETECTOR].enabled |
			sdata->cdata->sensors[LSM6DS3_TILT].enabled) {

			err = lsm6ds3_write_data_with_mask(sdata->cdata,
					lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
					lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
					lsm6ds3_odr_table.odr_avl[0].value, true);
		} else {
			err = lsm6ds3_write_data_with_mask(sdata->cdata,
					lsm6ds3_odr_table.addr[LSM6DS3_ACCEL],
					lsm6ds3_odr_table.mask[LSM6DS3_ACCEL],
					LSM6DS3_ODR_POWER_OFF_VAL, true);
		}
		if (err < 0)
			return err;

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
		hrtimer_cancel(&sdata->hr_timer);
		cancel_work_sync(&sdata->input_work);
#endif

		break;
	case LSM6DS3_GYRO:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					lsm6ds3_odr_table.addr[LSM6DS3_GYRO],
					lsm6ds3_odr_table.mask[LSM6DS3_GYRO],
					LSM6DS3_ODR_POWER_OFF_VAL, true);
		if (err < 0)
			return err;

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
		hrtimer_cancel(&sdata->hr_timer);
		cancel_work_sync(&sdata->input_work);
#endif

		break;
	case LSM6DS3_SIGN_MOTION:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_SIGN_MOTION_EN_ADDR,
					LSM6DS3_SIGN_MOTION_EN_MASK,
					LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		err = lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		sdata->cdata->sign_motion_event_ready = false;

		break;
	case LSM6DS3_STEP_COUNTER:
	case LSM6DS3_STEP_DETECTOR:
		err = lsm6ds3_enable_pedometer(sdata, false);
		if (err < 0)
			return err;

		break;
	case LSM6DS3_TILT:
		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_TILT_EN_ADDR,
					LSM6DS3_TILT_EN_MASK,
					LSM6DS3_DIS_BIT, true);
		if (err < 0)
			return err;

		break;
	default:
		return -EINVAL;
	}

	err = lsm6ds3_set_extra_dependency(sdata, false);
	if (err < 0)
		return err;

	err = lsm6ds3_set_drdy_irq(sdata, false);
	if (err < 0)
		return err;

	sdata->enabled = false;

#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
	err = lsm6ds3_reconfigure_fifo(sdata->cdata, true);
	if (err < 0)
		return err;
#endif

	return 0;
}

static int lsm6ds3_init_sensors(struct lsm6ds3_data *cdata)
{
	int err, i;
	u8 default_reg_value = 0;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->tb.buf_lock);

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];

		err = lsm6ds3_disable_sensors(sdata);
		if (err < 0)
			return err;

		if ((sdata->sindex == LSM6DS3_ACCEL) ||
				(sdata->sindex == LSM6DS3_GYRO)) {
			err = lsm6ds3_set_fs(sdata, sdata->c_gain);
			if (err < 0)
				return err;
		}
	}

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
	hrtimer_init(&cdata->sensors[LSM6DS3_ACCEL].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	hrtimer_init(&cdata->sensors[LSM6DS3_GYRO].hr_timer, CLOCK_MONOTONIC,
						HRTIMER_MODE_REL);
	cdata->sensors[LSM6DS3_ACCEL].hr_timer.function =
						&lsm6ds3_poll_function_read;
	cdata->sensors[LSM6DS3_GYRO].hr_timer.function =
						&lsm6ds3_poll_function_read;
#endif

	cdata->gyro_selftest_status = 0;
	cdata->accel_selftest_status = 0;
	cdata->steps_c = 0;
	cdata->reset_steps = false;

	err = lsm6ds3_write_data_with_mask(cdata, LSM6DS3_RESET_ADDR,
				LSM6DS3_RESET_MASK, LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_LIR_ADDR,
					LSM6DS3_LIR_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_TIMER_EN_ADDR,
					LSM6DS3_TIMER_EN_MASK,
					LSM6DS3_EN_BIT, true);
		if (err < 0)
			return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_BDU_ADDR,
					LSM6DS3_BDU_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
	err = lsm6ds3_set_fifo_enable(sdata->cdata, false);
	if (err < 0)
		return err;
#endif
	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_ROUNDING_ADDR,
					LSM6DS3_ROUNDING_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_write_data_with_mask(cdata,
					LSM6DS3_INT2_ON_INT1_ADDR,
					LSM6DS3_INT2_ON_INT1_MASK,
					LSM6DS3_EN_BIT, true);
	if (err < 0)
		return err;

	err = lsm6ds3_reset_steps(sdata->cdata);
	if (err < 0)
		return err;

	mutex_lock(&cdata->bank_registers_lock);
	err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1,
					&default_reg_value, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto lsm6ds3_init_sensor_mutex_unlock;
	mutex_unlock(&cdata->bank_registers_lock);

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
	cdata->sensors[LSM6DS3_ACCEL].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DS3_ACCEL].poll_interval));
	cdata->sensors[LSM6DS3_GYRO].ktime = ktime_set(0,
			MS_TO_NS(cdata->sensors[LSM6DS3_GYRO].poll_interval));
	INIT_WORK(&cdata->sensors[LSM6DS3_ACCEL].input_work, poll_function_work);
	INIT_WORK(&cdata->sensors[LSM6DS3_GYRO].input_work, poll_function_work);
#else
	err = lsm6ds3_reconfigure_fifo(cdata, false);
	if (err < 0)
		return err;
#endif

	return 0;

lsm6ds3_init_sensor_mutex_unlock:
	mutex_unlock(&cdata->bank_registers_lock);
	return err;
}

static int lsm6ds3_set_odr(struct lsm6ds3_sensor_data *sdata, u32 odr)
{
	int err = 0, i;

	for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
		if (lsm6ds3_odr_table.odr_avl[i].hz >= odr)
			break;
	}
	if (i == LSM6DS3_ODR_LIST_NUM)
		return -EINVAL;

	if (sdata->c_odr == lsm6ds3_odr_table.odr_avl[i].hz)
		return 0;

	if (sdata->enabled) {
		disable_irq(sdata->cdata->irq);
		lsm6ds3_flush_works();

		if (sdata->sindex == LSM6DS3_ACCEL)
			sdata->cdata->sensors[LSM6DS3_ACCEL].sample_to_discard +=
							LSM6DS3_ACCEL_STD;

		if (sdata->cdata->sensors[LSM6DS3_GYRO].enabled)
			sdata->cdata->sensors[LSM6DS3_GYRO].sample_to_discard +=
							LSM6DS3_GYRO_STD;

		err = lsm6ds3_write_data_with_mask(sdata->cdata,
					lsm6ds3_odr_table.addr[sdata->sindex],
					lsm6ds3_odr_table.mask[sdata->sindex],
					lsm6ds3_odr_table.odr_avl[i].value, true);
		if (err < 0) {
			enable_irq(sdata->cdata->irq);

			return err;
		}

		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;
#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
		err = lsm6ds3_reconfigure_fifo(sdata->cdata, false);
		if (err < 0)
			return err;
#endif
		enable_irq(sdata->cdata->irq);
	} else
		sdata->c_odr = lsm6ds3_odr_table.odr_avl[i].hz;

	return err;
}

static ssize_t get_enable(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->enabled);
}

static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int err;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);
	unsigned long enable;

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable)
		err = lsm6ds3_enable_sensors(sdata);
	else
		err = lsm6ds3_disable_sensors(sdata);

	return count;
}

#if defined (CONFIG_LSM6DS3_POLLING_MODE)
static ssize_t get_polling_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->poll_interval);
}

static ssize_t set_polling_rate(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int polling_rate;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &polling_rate);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	/*
	 * Polling interval is in msec, then we have to convert it in Hz to
	 * configure ODR through lsm6ds3_set_odr
	 */
	err = lsm6ds3_set_odr(sdata, 1000 / polling_rate);
	if (!(err < 0)) {
		sdata->poll_interval = polling_rate;
		sdata->ktime = ktime_set(0, MS_TO_NS(polling_rate));
	}
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}
#endif

static ssize_t get_sampling_freq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->c_odr);
}

static ssize_t set_sampling_freq(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int odr;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &odr);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	err = lsm6ds3_set_odr(sdata, odr);
	mutex_unlock(&sdata->input_dev->mutex);

	return (err < 0 ? err : count);
}

#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
static ssize_t get_fifo_length(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", sdata->fifo_length);
}

static ssize_t set_fifo_length(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int fifo_length;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &fifo_length);
	if (err < 0)
		return err;

	mutex_lock(&sdata->input_dev->mutex);
	sdata->fifo_length = fifo_length;
	mutex_unlock(&sdata->input_dev->mutex);

	lsm6ds3_reconfigure_fifo(sdata->cdata, true);

	return count;
}

static ssize_t get_hw_fifo_lenght(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", LSM6DS3_MAX_FIFO_LENGHT);
}

static ssize_t flush_fifo(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	disable_irq(sdata->cdata->irq);
	lsm6ds3_flush_works();

	mutex_lock(&sdata->cdata->fifo_lock);
	lsm6ds3_read_fifo(sdata->cdata, true);
	mutex_unlock(&sdata->cdata->fifo_lock);

	enable_irq(sdata->cdata->irq);

	return size;
}
#endif

static ssize_t reset_steps(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned int reset;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &reset);
	if (err < 0)
		return err;

	lsm6ds3_reset_steps(sdata->cdata);

	return count;
}

static ssize_t set_max_delivery_rate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 duration;
	int err, err2;
	unsigned int max_delivery_rate;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtouint(buf, 10, &max_delivery_rate);
	if (err < 0)
		return -EINVAL;

	if (max_delivery_rate == sdata->c_odr)
		return size;

	duration = max_delivery_rate / LSM6DS3_MIN_DURATION_MS;

	mutex_lock(&sdata->cdata->bank_registers_lock);

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_EN_BIT, false);
	if (err < 0)
		goto lsm6ds3_set_max_delivery_rate_mutex_unlock;

	err = sdata->cdata->tf->write(sdata->cdata,
					LSM6DS3_STEP_COUNTER_DURATION_ADDR,
					1, &duration, false);
	if (err < 0)
		goto lsm6ds3_set_max_delivery_rate_restore_bank;

	err = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_DIS_BIT, false);
	if (err < 0)
		goto lsm6ds3_set_max_delivery_rate_restore_bank;

	mutex_unlock(&sdata->cdata->bank_registers_lock);

	sdata->c_odr = max_delivery_rate;

	return size;

lsm6ds3_set_max_delivery_rate_restore_bank:
	do {
		err2 = lsm6ds3_write_data_with_mask(sdata->cdata,
					LSM6DS3_FUNC_CFG_ACCESS_ADDR,
					LSM6DS3_FUNC_CFG_REG2_MASK,
					LSM6DS3_DIS_BIT, false);

		msleep(500);
	} while (err2 < 0);

lsm6ds3_set_max_delivery_rate_mutex_unlock:
	mutex_unlock(&sdata->cdata->bank_registers_lock);
	return err;
}

static ssize_t get_max_delivery_rate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sdata->c_odr);
}

static ssize_t get_sampling_frequency_avail(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int i, len = 0;

	for (i = 0; i < LSM6DS3_ODR_LIST_NUM; i++) {
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
					lsm6ds3_odr_table.odr_avl[i].hz);
	}
	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_scale_avail(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, len = 0;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			lsm6ds3_fs_table[sdata->sindex].fs_avl[i].urv);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t get_cur_scale(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++)
		if (sdata->c_gain == lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain)
			break;

	return sprintf(buf, "%d\n",
			lsm6ds3_fs_table[sdata->sindex].fs_avl[i].urv);
}

static ssize_t set_cur_scale(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int i, urv, err;
	struct lsm6ds3_sensor_data *sdata = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &urv);
	if (err < 0)
		return err;

	for (i = 0; i < LSM6DS3_FS_LIST_NUM; i++)
		if (urv == lsm6ds3_fs_table[sdata->sindex].fs_avl[i].urv)
			break;

	if (i == LSM6DS3_FS_LIST_NUM)
		return -EINVAL;

	err = lsm6ds3_set_fs(sdata,
				lsm6ds3_fs_table[sdata->sindex].fs_avl[i].gain);
	if (err < 0)
		return err;

	return count;
}


static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, get_enable, set_enable);
static DEVICE_ATTR(sampling_freq, S_IWUSR | S_IRUGO, get_sampling_freq,
							set_sampling_freq);
#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
static DEVICE_ATTR(fifo_length, S_IWUSR | S_IRUGO, get_fifo_length,
							set_fifo_length);
static DEVICE_ATTR(get_hw_fifo_lenght, S_IRUGO, get_hw_fifo_lenght, NULL);
static DEVICE_ATTR(flush_fifo, S_IWUSR, NULL, flush_fifo);
#else
static DEVICE_ATTR(polling_rate, S_IWUSR | S_IRUGO, get_polling_rate,
							set_polling_rate);
#endif
static DEVICE_ATTR(reset_steps, S_IWUSR, NULL, reset_steps);
static DEVICE_ATTR(max_delivery_rate, S_IWUSR | S_IRUGO, get_max_delivery_rate,
							set_max_delivery_rate);
static DEVICE_ATTR(sampling_freq_avail, S_IRUGO, get_sampling_frequency_avail, NULL);
static DEVICE_ATTR(scale_avail, S_IRUGO, get_scale_avail, NULL);
static DEVICE_ATTR(scale, S_IWUSR | S_IRUGO, get_cur_scale, set_cur_scale);

static struct attribute *lsm6ds3_accel_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
#if defined (CONFIG_LSM6DS3_POLLING_MODE)
	&dev_attr_polling_rate.attr,
#else
	&dev_attr_fifo_length.attr,
	&dev_attr_get_hw_fifo_lenght.attr,
	&dev_attr_flush_fifo.attr,
#endif
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm6ds3_gyro_attribute[] = {
	&dev_attr_enable.attr,
	&dev_attr_sampling_freq.attr,
#if defined (CONFIG_LSM6DS3_POLLING_MODE)
	&dev_attr_polling_rate.attr,
#else
	&dev_attr_fifo_length.attr,
	&dev_attr_get_hw_fifo_lenght.attr,
	&dev_attr_flush_fifo.attr,
#endif
	&dev_attr_sampling_freq_avail.attr,
	&dev_attr_scale_avail.attr,
	&dev_attr_scale.attr,
	NULL,
};

static struct attribute *lsm6ds3_sign_m_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm6ds3_step_c_attribute[] = {
	&dev_attr_enable.attr,
#if !defined (CONFIG_LSM6DS3_POLLING_MODE)
	&dev_attr_fifo_length.attr,
	&dev_attr_get_hw_fifo_lenght.attr,
	&dev_attr_flush_fifo.attr,
#endif
	&dev_attr_reset_steps.attr,
	&dev_attr_max_delivery_rate.attr,
	NULL,
};

static struct attribute *lsm6ds3_step_d_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute *lsm6ds3_tilt_attribute[] = {
	&dev_attr_enable.attr,
	NULL,
};

static const struct attribute_group lsm6ds3_attribute_groups[] = {
	[LSM6DS3_ACCEL] = {
		.attrs = lsm6ds3_accel_attribute,
		.name = "accel",
	},
	[LSM6DS3_GYRO] = {
		.attrs = lsm6ds3_gyro_attribute,
		.name = "gyro",
	},
	[LSM6DS3_SIGN_MOTION] = {
		.attrs = lsm6ds3_sign_m_attribute,
		.name = "sign_m",
	},
	[LSM6DS3_STEP_COUNTER] = {
		.attrs = lsm6ds3_step_c_attribute,
		.name = "step_c",
	},
	[LSM6DS3_STEP_DETECTOR] = {
		.attrs = lsm6ds3_step_d_attribute,
		.name = "step_d",
	},
	[LSM6DS3_TILT] = {
		.attrs = lsm6ds3_tilt_attribute,
		.name = "tilt",
	},
};

static int	lsm6ds3_devenable(struct sensors_classdev *sensors_cdev,
					unsigned int enabled){
	struct lsm6ds3_sensor_data *sdata = container_of(sensors_cdev,
			                struct lsm6ds3_sensor_data, class_dev);
	int ret;

	mutex_lock(&sdata->input_dev->mutex);
	if (enabled)
		ret = lsm6ds3_enable_sensors(sdata);
	else
		ret = lsm6ds3_disable_sensors(sdata);
	mutex_unlock(&sdata->input_dev->mutex);
	return ret;
}

static int	lsm6ds3_devdelay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec){
	struct lsm6ds3_sensor_data *sdata = container_of(sensors_cdev,
			                struct lsm6ds3_sensor_data, class_dev);				

	mutex_lock(&sdata->input_dev->mutex);
	/*
	 * Polling interval is in msec, then we have to convert it in Hz to
	 * configure ODR through lsm6ds3_set_odr
	 */
	if (!(lsm6ds3_set_odr(sdata, 1000 / delay_msec) < 0)) {
		sdata->poll_interval = delay_msec;
		sdata->ktime = ktime_set(0, MS_TO_NS(delay_msec));
	}
	mutex_unlock(&sdata->input_dev->mutex);
	return 0;
}

static int lsm6ds3_calibrate(struct sensors_classdev *sensors_cdev,
		int axis, int apply_now){
	int32_t ret,data[3] = {0, 0, 0};
	long int sum_x = 0, sum_y = 0, sum_z = 0;
	int32_t i = 0;
	int rept = CAL_REPT;
	int32_t max_data[3] = {0, 0, 0};
	int32_t min_data[3] = {0, 0, 0};
	unsigned int cali_delay = 5;//make the test fastest
	unsigned int old_delay;
	unsigned int cali_enable = 1;

	struct lsm6ds3_sensor_data *sdata = container_of(sensors_cdev,
		                struct lsm6ds3_sensor_data, class_dev);	

	old_delay = sensors_cdev->delay_msec;
	lsm6ds3_devdelay(sensors_cdev, cali_delay);
	if (!sensors_cdev->enabled)
	   lsm6ds3_devenable(sensors_cdev, cali_enable);

	for (i = 0; i < rept; i++) {
	   msleep(sdata->poll_interval);
	   if (unlikely(lsm6ds3_cal_poll_data(sdata, data) < 0)) {
		  dev_err(sdata->cdata->dev, "get %s data error!\n", sdata->name);
		  return ret;
	   }

	   if (i == 0){
		   max_data[0] = data[0];
		   max_data[1] = data[1];
		   max_data[2] = data[2];
		   
		   min_data[0] = data[0];
		   min_data[1] = data[1];
		   min_data[2] = data[2];
	   }

	   if (max_data[0] < data[0])
			   max_data[0] = data[0];
	   if (max_data[1] < data[1])
			   max_data[1] = data[1];
	   if (max_data[2] < data[2])
			   max_data[2] = data[2];

	   if (min_data[0] > data[0])
			   min_data[0] = data[0];
	   if (min_data[1] > data[1])
			   min_data[1] = data[1];
	   if (min_data[2] > data[2])
			   min_data[2] = data[2];

	   sum_x += data[0];
	   sum_y += data[1];
	   sum_z += data[2];
		   
	}

	sum_x -= (max_data[0] + min_data[0]);
	sum_y -= (max_data[1] + min_data[1]);
	sum_z -= (max_data[2] + min_data[2]);

	data[0] = sum_x / (rept - 2);
	data[1] = sum_y / (rept - 2);
	data[2] = sum_z / (rept - 2);

	sdata->cali_data[0] = -data[0];
	sdata->cali_data[1] = -data[1];
	if (sdata->sindex == LSM6DS3_ACCEL) {
		sdata->cali_data[2] = GRAVITY_HAL_UNIT - ABS(data[2]);
	} else if (sdata->sindex == LSM6DS3_GYRO) {
		sdata->cali_data[2] = -data[2];
	}

	if (sensors_cdev->params == NULL) {
	  sensors_cdev->params = devm_kzalloc(sdata->cdata->dev, 128, GFP_KERNEL);
	  if (sensors_cdev->params == NULL) {
			  dev_err(sdata->cdata->dev, "sensor_cdev allocate memory error!\n");
			  return -EBUSY;
	  }
	}
	snprintf(sensors_cdev->params, 64, "%d,%d,%d", 
						sdata->cali_data[0],
						sdata->cali_data[1], 
						sdata->cali_data[2]);
	dev_dbg(sdata->cdata->dev, "sensor calibration offset:%s\n",sensors_cdev->params);

	lsm6ds3_devdelay(sensors_cdev, old_delay);
	if (!sensors_cdev->enabled)
	   lsm6ds3_devenable(sensors_cdev, !cali_enable);

	return 0;
}

static int	lsm6ds3_write_cali_params(struct sensors_classdev
		*sensors_cdev, struct cal_result_t *cal_result){
	
	struct lsm6ds3_sensor_data *sdata = container_of(sensors_cdev,
							struct lsm6ds3_sensor_data, class_dev); 
	
	if (sensors_cdev->params == NULL) {
	   sensors_cdev->params = devm_kzalloc(sdata->cdata->dev, 128, GFP_KERNEL);
	   if (sensors_cdev->params == NULL) {
	           dev_err(sdata->cdata->dev, "sensor_cdev allocate memory error!\n");
	           return -EBUSY;
	   }
	}
	sdata->cali_data[0] = cal_result->offset[0];
	sdata->cali_data[1] = cal_result->offset[1];
	sdata->cali_data[2] = cal_result->offset[2];

	snprintf(sensors_cdev->params, 64, "%d,%d,%d", 
				sdata->cali_data[0],
				sdata->cali_data[1],
				sdata->cali_data[2]);
	
	return 0;
}

static int	lsm6ds3_flush(struct sensors_classdev *sensors_cdev){
	struct timespec ts;
	struct lsm6ds3_sensor_data *sdata = container_of(sensors_cdev,
							struct lsm6ds3_sensor_data, class_dev); 

	struct input_dev  * input  	= sdata->input_dev;

	get_monotonic_boottime(&ts);
	input_event(input, EV_SYN, SYN_TIME_SEC, ts.tv_sec);
	input_event(input, EV_SYN, SYN_TIME_NSEC, ts.tv_nsec);
	input_sync(input);
	return 0;
}

static int lsm6ds3_sysclass_register(struct device *dev, 
                          struct lsm6ds3_sensor_data *sdata){
	int ret;
	
	switch (sdata->sindex){
	case LSM6DS3_ACCEL:
		sdata->class_dev = lsm6ds3_acc_classdev;
		break;
	case LSM6DS3_GYRO:
		sdata->class_dev = lsm6ds3_gyr_classdev;
		break;
	case LSM6DS3_STEP_COUNTER:
		sdata->class_dev = lsm6ds3_step_counter_classdev;
		break;
	case LSM6DS3_STEP_DETECTOR:
		sdata->class_dev = lsm6ds3_step_detector_classdev;
		break;
	default:
		return -1;
	}
	
	sdata->class_dev.sensors_enable= lsm6ds3_devenable;
	sdata->class_dev.sensors_flush = lsm6ds3_flush;

	if ((sdata->sindex == LSM6DS3_ACCEL) || (sdata->sindex == LSM6DS3_GYRO)){
		sdata->class_dev.sensors_poll_delay = lsm6ds3_devdelay;
		sdata->class_dev.sensors_calibrate = lsm6ds3_calibrate;
		sdata->class_dev.sensors_write_cal_params= lsm6ds3_write_cali_params;
	}
	
	ret = sensors_classdev_register(dev, &sdata->class_dev);
	if (ret)
		return -1;
	return 0;
}
#if 0
static int lsm6ds3_pinctrl_init(struct lsm6ds3_data *cdata){

	cdata->pinctrl = devm_pinctrl_get(cdata->dev);
	if (IS_ERR_OR_NULL(cdata->pinctrl)) {
		dev_err(cdata->dev, "Failed to get pinctrl\n");
		return PTR_ERR(cdata->pinctrl);
	}

	cdata->pin_default = pinctrl_lookup_state(cdata->pinctrl, "default");
	if (IS_ERR_OR_NULL(cdata->pin_default)) {
		dev_err(cdata->dev, "Failed to look up default state\n");
		return PTR_ERR(cdata->pin_default);
	}

	cdata->pin_sleep = pinctrl_lookup_state(cdata->pinctrl, "sleep");
	if (IS_ERR_OR_NULL(cdata->pin_sleep)) {
		dev_err(cdata->dev, "Failed to look up sleep state\n");
		return PTR_ERR(cdata->pin_sleep);
	}

	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_dt_id[] = {
	{.compatible = "st,lsm6ds3",},
	{},
};
MODULE_DEVICE_TABLE(of, lsm6ds3_dt_id);

static u32 lsm6ds3_parse_dt(struct lsm6ds3_data *cdata)
{
	u32 val;
	struct device_node *np;

	np = cdata->dev->of_node;
	if (!np)
		return -EINVAL;

	if (!of_property_read_u32(np, "st,drdy-int-pin", &val) &&
							(val <= 2) && (val > 0))
		cdata->drdy_int_pin = (u8)val;
	else
		cdata->drdy_int_pin = 1;

	val = of_get_named_gpio_flags(np, "st,intr1", 0, NULL);
	if (val < 0) {
		dev_err(cdata->dev, "Unable to read intr\n");
		return val;
	}
	cdata->gpio_irq1 = val;

	if (gpio_request_one(cdata->gpio_irq1, GPIOF_DIR_IN, "lsm6ds3_gpio_intr1")){
		dev_err(cdata->dev,	"%s: GPIO %d Request Fail\n", __func__, cdata->gpio_irq1);
		return -1;
	}
	return 0;
}

#else
#endif

int lsm6ds3_common_probe(struct lsm6ds3_data *cdata, int irq, u16 bustype)
{
	/* TODO: add errors management */
	int32_t err, i;
	u8 wai =0x00;
	struct lsm6ds3_sensor_data *sdata;

	mutex_init(&cdata->bank_registers_lock);
	mutex_init(&cdata->fifo_lock);
	cdata->fifo_data_buffer = 0;

	err = cdata->tf->read(cdata, LSM6DS3_WHO_AM_I, 1, &wai, true);
	if (err < 0) {
		dev_err(cdata->dev, "failed to read Who-Am-I register.\n");
		return err;
	}
	if (wai != LSM6DS3_WHO_AM_I_DEF) {
		dev_err(cdata->dev, "Who-Am-I value not valid.\n");
		return -ENODEV;
	}

	mutex_init(&cdata->lock);

	if (irq > 0) {
#ifdef CONFIG_OF
		err = lsm6ds3_parse_dt(cdata);
		if (err < 0)
			return err;
#else /* CONFIG_OF */
		if (cdata->dev->platform_data) {
			cdata->drdy_int_pin = ((struct lsm6ds3_platform_data *)
					cdata->dev->platform_data)->drdy_int_pin;

			if ((cdata->drdy_int_pin > 2) || (cdata->drdy_int_pin < 1))
				cdata->drdy_int_pin = 1;
		} else
			cdata->drdy_int_pin = 1;
#endif /* CONFIG_OF */

		dev_info(cdata->dev, "driver use DRDY int pin %d\n",
							cdata->drdy_int_pin);
	}
#if 0
	/* initialize pinctrl */
	if (!lsm6ds3_pinctrl_init(cdata)) {
		err = pinctrl_select_state(cdata->pinctrl, cdata->pin_default);
		if (err) {
			dev_err(cdata->dev, "Can't select pinctrl state\n");
			return err;
		}
	}
#endif
	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		sdata->enabled = false;
		sdata->cdata = cdata;
		sdata->sindex = i;
		sdata->name = lsm6ds3_sensor_name[i].name;
		sdata->fifo_length = 1;
		if (i == LSM6DS3_ACCEL) {
			sdata->c_odr = lsm6ds3_odr_table.odr_avl[0].hz;
			sdata->c_gain = lsm6ds3_fs_table[i].fs_avl[0].gain;
#if defined (CONFIG_LSM6DS3_POLLING_MODE)
			sdata->poll_interval = 1000 / sdata->c_odr;
#endif
		}

		if  (i == LSM6DS3_GYRO) {
			sdata->c_odr = lsm6ds3_odr_table.odr_avl[0].hz;
			sdata->c_gain = lsm6ds3_fs_table[i].fs_avl[1].gain;
#if defined (CONFIG_LSM6DS3_POLLING_MODE)
			sdata->poll_interval = 1000 / sdata->c_odr;
#endif
		}
		
		if (i == LSM6DS3_STEP_COUNTER) {
			sdata->c_odr = LSM6DS3_MIN_DURATION_MS;
		}

		lsm6ds3_input_init(sdata, bustype, lsm6ds3_sensor_name[i].description);

		if (sysfs_create_group(&sdata->input_dev->dev.kobj,
						&lsm6ds3_attribute_groups[i])) {
			dev_err(cdata->dev, "failed to create sysfs group for sensor %s",
								sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
		//compatiable qcom interface
		err = lsm6ds3_sysclass_register(&sdata->input_dev->dev, sdata);
		if (err < 0) {
			dev_err(cdata->dev, "failed to create sensor class for sensor %s", sdata->name);
			input_unregister_device(sdata->input_dev);
			sdata->input_dev = NULL;
		}
		
	}

	if(lsm6ds3_workqueue == 0)
		lsm6ds3_workqueue = alloc_workqueue("lsm6ds3_workqueue", WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	//		create_workqueue("lsm6ds3_workqueue");

	err = lsm6ds3_init_sensors(cdata);
	if (err < 0)
		return err;
	if (irq > 0)
		cdata->irq = irq;

	if (irq > 0) {
		err = lsm6ds3_interrupt_init(cdata);
		if (err < 0)
			return err;
	}

	cdata->fifo_data_buffer = kmalloc(LSM6DS3_MAX_FIFO_SIZE, GFP_KERNEL);
	if (!cdata->fifo_data_buffer)
		return -ENOMEM;

	cdata->fifo_data_size = LSM6DS3_MAX_FIFO_SIZE;

	dev_info(cdata->dev, "%s: probed\n", LSM6DS3_ACC_GYR_DEV_NAME);
	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_probe);

void lsm6ds3_common_remove(struct lsm6ds3_data *cdata, int irq)
{
	u8 i;

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		lsm6ds3_disable_sensors(&cdata->sensors[i]);
		lsm6ds3_input_cleanup(&cdata->sensors[i]);
		sensors_classdev_unregister(&cdata->sensors[i].class_dev);
	}

	//remove_sysfs_interfaces(cdata->dev);

	if(!lsm6ds3_workqueue) {
		flush_workqueue(lsm6ds3_workqueue);
		destroy_workqueue(lsm6ds3_workqueue);
	}

	kfree(cdata->fifo_data_buffer);
}
EXPORT_SYMBOL(lsm6ds3_common_remove);

#ifdef CONFIG_PM
int lsm6ds3_common_suspend(struct lsm6ds3_data *cdata)
{
	int i, ret;
	struct lsm6ds3_sensor_data *sdata; 

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		if (sdata->enabled) {
			ret = lsm6ds3_disable_sensors(sdata);
			if (ret) {
				dev_err(cdata->dev, "%s failed\n", __FUNCTION__);
				return ret;
			}
		}
	}
	dev_info(cdata->dev, "%s\n", __FUNCTION__);	
	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_suspend);

int lsm6ds3_common_resume(struct lsm6ds3_data *cdata)
{
	int i, ret = 0;
	struct lsm6ds3_sensor_data *sdata;

	for (i = 0; i < LSM6DS3_SENSORS_NUMB; i++) {
		sdata = &cdata->sensors[i];
		if (sdata->class_dev.enabled) {
			ret = lsm6ds3_enable_sensors(sdata);
			if (ret) {
				dev_err(cdata->dev, "%s failed\n", __FUNCTION__);
				return ret;
			}
		}
	}
	dev_info(cdata->dev, "%s\n", __FUNCTION__);	
	return 0;
}
EXPORT_SYMBOL(lsm6ds3_common_resume);
#endif /* CONFIG_PM */

