#ifndef IT7236_TOUCHKEY_H
#define IT7236_TOUCHKEY_H
#define MAX_BUFFER_SIZE		144
#define DEVICE_NAME		"IT7236"
#define DEVICE_VENDOR		0
#define DEVICE_PRODUCT		0
#define DEVICE_VERSION		0
#define VERSION_ABOVE_ANDROID_20
#define IOC_MAGIC		'd'
//#define IOCTL_SET 		_IOW(IOC_MAGIC, 1, struct ioctl_cmd168)
//#define IOCTL_GET 		_IOR(IOC_MAGIC, 2, struct ioctl_cmd168)

#define IOCTL_SET 		1093035009
#define IOCTL_GET 		-2128190462

struct ioctl_cmd168 {
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

struct IT7236_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
	#endif
	struct workqueue_struct *ts_workqueue;
	struct mutex device_mode_mutex;
	const char *name;
	bool reg_enable;
	u32 enable_gpio;
	u32 enable_gpio_flags;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 panel_miny;
	u32 panel_maxy;
	u32 y_min;
	u32 y_max;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
	uint8_t debug_log_level;
	const char *gold_sample;
	int gold_sample_size;
};

static struct IT7236_ts_data *gl_ts;

#endif