
#ifndef __LINUX_ATMEL_MXT_PLUG_CAL
#define __LINUX_ATMEL_MXT_PLUG_CAL

#include <linux/types.h>

enum {
	MOV_LEFT = 0,
	MOV_UP,
	MOV_RIGHT,
	MOV_DOWN,
	MOV_NUM
};

ssize_t plugin_cal_show(struct plugin_cal *p, char *buf, size_t count);
int plugin_cal_store(struct plugin_cal *p, const char *buf, size_t count);

int plugin_interface_cal_init(struct plugin_cal *p);

#endif


